#!/usr/bin/env python3
"""
S500 DISARMED SITL MAVLink Entegrasyon Test Koşucusu (test_sitl_disarmed.py)
-----------------------------------------------------------------------------
Bu script, ArduPilot SITL ortamında s500_mission.lua durum makinesinin
araç YERDE ve DISARMED iken gerçek MAVLink soketi üzerinden:
1. SESSION_START ve APP_ACK el sıkışmasını,
2. Aktif oturumdan seq=0 canlılık mesajı kabulünü,
3. Geçersiz kaynak/hedef, sürüm ve ondalıklı alanların reddini (otopilot log kanıtıyla),
4. Tekrar mesajları ve aynı seq ile çakışan içerik reddini,
5. Yeni oturum geçişini ve kapatılmış oturum reddini,
6. Tüm bu süreç boyunca aracın DISARMED kaldığını (heartbeat sürekli denetimiyle)
doğrular.

GÜVENİLİRLİK İLKELERİ:
- Tek MAVLink alıcısı (background receiver thread); heartbeat, log ve ACK kuyruklarına dağıtım.
- Otopilottan (sys=1, comp=1) güncel heartbeat zorunluluğu; heartbeat kaybı, armed görülmesi
  veya kritik RX hatası koşuyu KALICI olarak başarısız yapar (sonradan toparlanma silemez).
- APP_ACK alanlarının round() ile yuvarlanmadan, kesin tam sayı ve adres doğrulamasıyla denetimi.
- Canlılık sonrası el sıkışması yanıtı yalnızca alıcının çalıştığının kanıtıdır; iç zaman damgası
  telemetriden doğrudan gözlenemediğinde PASS yerine UNVERIFIED olarak kaydedilir.
- Genel başarı için 6 senaryonun tamamının PASS olması ve kritik hata bulunmaması şarttır.
- SITL stdout/stderr dosyaya kaydedilir; çıkışta süreç ve iş parçacıkları temizlenir.
"""

import os
import sys
import time
import math
import json
import signal
import queue
import hashlib
import threading
import subprocess
from typing import Optional, Dict, Any, List, Tuple
from pymavlink import mavutil

MAV_CMD_USER_1 = 31010
STATUS_NO_DETECTION     = 0
STATUS_START_FOUND      = 1
STATUS_GOAL_FOUND       = 2
STATUS_GOTO_OBSERVATION = 3
STATUS_ROUTE_READY      = 4
STATUS_GCS_DELIVERED    = 5
STATUS_LIVELINESS        = 170 # 0xAA
STATUS_APP_ACK           = 171 # 0xAB
STATUS_SESSION_START     = 172 # 0xAC

def get_file_sha256(filepath: str) -> str:
    if not os.path.exists(filepath):
        return "DOSYA_YOK"
    h = hashlib.sha256()
    with open(filepath, "rb") as f:
        while chunk := f.read(8192):
            h.update(chunk)
    return h.hexdigest()

class StatustextAssembler:
    """MAVLink parçalı STATUSTEXT mesajlarını bayt düzeyinde birleştiren yardımcı sınıf."""
    def __init__(self):
        self.chunks: Dict[int, Dict[int, bytes]] = {}
        self.final_seqs: Dict[int, int] = {}

    def feed(self, msg) -> Optional[str]:
        chunk_id = getattr(msg, 'id', 0)
        chunk_seq = getattr(msg, 'chunk_seq', 0)

        if hasattr(msg, '_text_raw') and msg._text_raw is not None:
            raw_payload = bytes(msg._text_raw)
        elif isinstance(msg.text, bytes):
            raw_payload = msg.text
        elif isinstance(msg.text, str):
            raw_payload = msg.text.encode('utf-8', errors='ignore')
        else:
            raw_payload = b""

        if b'\x00' in raw_payload:
            chunk_bytes = raw_payload.split(b'\x00', 1)[0]
            is_last = True
        else:
            chunk_bytes = raw_payload
            is_last = (len(chunk_bytes) < 50)

        if chunk_id == 0:
            try:
                return chunk_bytes.decode('utf-8', errors='replace').strip()
            except Exception:
                return chunk_bytes.decode('latin-1', errors='ignore').strip()

        if chunk_id not in self.chunks:
            self.chunks[chunk_id] = {}

        self.chunks[chunk_id][chunk_seq] = chunk_bytes
        if is_last:
            self.final_seqs[chunk_id] = chunk_seq

        if chunk_id in self.final_seqs:
            final_seq = self.final_seqs[chunk_id]
            if all(s in self.chunks[chunk_id] for s in range(final_seq + 1)):
                all_bytes = b"".join(self.chunks[chunk_id][s] for s in range(final_seq + 1))
                del self.chunks[chunk_id]
                del self.final_seqs[chunk_id]
                try:
                    return all_bytes.decode('utf-8', errors='replace').strip()
                except Exception:
                    return all_bytes.decode('latin-1', errors='ignore').strip()

        return None

class SitlDisarmedTestRunner:
    def __init__(self, sitl_dir: str = "/workspace/sitl_test_run"):
        self.sitl_dir = sitl_dir
        self.proc: Optional[subprocess.Popen] = None
        self.sitl_log_file = None
        self.mav = None
        self.running = True
        self.rx_thread: Optional[threading.Thread] = None

        # Tek alıcı kuyrukları ve durum değişkenleri
        self.assembler = StatustextAssembler()
        self.raw_logs: List[Dict[str, Any]] = []
        self.ack_queue: queue.Queue = queue.Queue()
        self.last_heartbeat_mono: float = 0.0
        self.is_armed: Optional[bool] = None

        # Kalıcı Hata ve İhlal Bayrakları
        self.critical_failure_occurred: bool = False
        self.armed_violation_seen: bool = False
        self.heartbeat_loss_detected: bool = False
        self.rx_fatal_error: bool = False

        # Senaryo sonuçları ve hata kayıtları
        self.scenario_results: Dict[str, Dict[str, Any]] = {}
        self.failure_reasons: List[str] = []
        self.t_start_mono: float = 0.0

    def _rx_worker(self):
        """Tek MAVLink alıcı döngüsü: gelen tüm mesajları ayrıştırır ve ilgili kuyruklara dağıtır."""
        while self.running and self.mav:
            try:
                msg = self.mav.recv_match(blocking=True, timeout=0.05)
                if not msg:
                    continue

                now_mono = time.monotonic()
                mtype = msg.get_type()
                src_sys = msg.get_srcSystem()
                src_comp = msg.get_srcComponent()

                # Sadece hedef otopilottan (sys=1, comp=1) gelen mesajları işle
                if src_sys != 1 or src_comp != 1:
                    continue

                if mtype == 'HEARTBEAT':
                    self.last_heartbeat_mono = now_mono
                    armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                    self.is_armed = armed
                    if armed:
                        self.armed_violation_seen = True
                        self.critical_failure_occurred = True
                        err = f"GÜVENLİK İHLALİ: Otopilot beklenmedik şekilde ARMED durumuna geçti! (base_mode=0x{msg.base_mode:X})"
                        print(f"[RX-ALICI] {err}", flush=True)
                        self.failure_reasons.append(err)

                elif mtype == 'STATUSTEXT':
                    text = self.assembler.feed(msg)
                    if text:
                        entry = {"timestamp_mono": now_mono, "text": text}
                        self.raw_logs.append(entry)
                        print(f"  [STATUSTEXT] {text}", flush=True)

                elif mtype == 'COMMAND_LONG':
                    self.ack_queue.put((now_mono, msg))

            except Exception as e:
                if self.running:
                    self.rx_fatal_error = True
                    self.critical_failure_occurred = True
                    err = f"RX worker kritik hatası: {e}"
                    print(f"[RX-ALICI] {err}", flush=True)
                    self.failure_reasons.append(err)

    def check_heartbeat_liveness(self, max_age_s: float = 3.0) -> bool:
        """Otopilottan güncel heartbeat alınıp alınmadığını ve armed olunmadığını doğrular."""
        if self.critical_failure_occurred:
            return False
        if self.last_heartbeat_mono == 0.0:
            return False
        age = time.monotonic() - self.last_heartbeat_mono
        if age > max_age_s:
            self.heartbeat_loss_detected = True
            self.critical_failure_occurred = True
            err = f"Heartbeat kaybı tespit edildi! ({age:.2f}s > {max_age_s:.1f}s)"
            print(f"[GÜVENLİK-DENETİMİ] {err}", flush=True)
            self.failure_reasons.append(err)
            return False
        return True

    def start_sitl(self):
        log_path = os.path.join(self.sitl_dir, "sitl_stdout_disarmed.log")
        self.sitl_log_file = open(log_path, "w")
        cmd = [
            "/opt/ardupilot/build/sitl/bin/arducopter",
            "--model", "quad",
            "--home", "-35.363261,149.165230,584,353",
            "--defaults", f"/opt/ardupilot/Tools/autotest/default_params/copter.parm,{self.sitl_dir}/extra_params.parm"
        ]
        print(f"[SITL] Süreç başlatılıyor: {' '.join(cmd)}", flush=True)
        print(f"[SITL] Çıktı yönlendiriliyor: {log_path}", flush=True)
        self.proc = subprocess.Popen(
            cmd,
            cwd=self.sitl_dir,
            stdout=self.sitl_log_file,
            stderr=subprocess.STDOUT,
            preexec_fn=os.setsid
        )

        # Port 5760'a Jetson companion kimliği (sysid=1, compid=191) ile bağlan
        t0 = time.monotonic()
        conn_ok = False
        while (time.monotonic() - t0) < 25.0:
            try:
                temp_mav = mavutil.mavlink_connection("tcp:127.0.0.1:5760", source_system=1, source_component=191)
                hb = temp_mav.wait_heartbeat(timeout=1.0)
                if hb and hb.get_srcSystem() == 1 and hb.get_srcComponent() == 1:
                    self.mav = temp_mav
                    self.last_heartbeat_mono = time.monotonic()
                    conn_ok = True
                    print(f"[SITL] Gerçek Heartbeat teyit edildi! (Otopilot SysID={hb.get_srcSystem()}, CompID={hb.get_srcComponent()})", flush=True)
                    break
                else:
                    if temp_mav:
                        temp_mav.close()
            except Exception:
                time.sleep(0.5)

        if not conn_ok or not self.mav:
            raise RuntimeError("SITL MAVLink bağlantısı ve ilk Heartbeat 25s içinde kurulamadı!")

        # Tek alıcı iş parçacığını başlat
        self.rx_thread = threading.Thread(target=self._rx_worker, daemon=True)
        self.rx_thread.start()

        # Telemetri ve log akışını talep et
        self.mav.mav.request_data_stream_send(1, 1, mavutil.mavlink.MAV_DATA_STREAM_ALL, 10, 1)

    def stop_sitl(self):
        self.running = False
        if self.rx_thread and self.rx_thread.is_alive():
            try:
                self.rx_thread.join(timeout=1.0)
            except Exception:
                pass
            self.rx_thread = None
        if self.proc:
            try:
                os.killpg(os.getpgid(self.proc.pid), signal.SIGTERM)
                self.proc.wait(timeout=3.0)
            except Exception:
                try:
                    os.killpg(os.getpgid(self.proc.pid), signal.SIGKILL)
                    self.proc.wait(timeout=2.0)
                except Exception:
                    pass
            self.proc = None
        if self.mav:
            try:
                self.mav.close()
            except Exception:
                pass
            self.mav = None
        if self.sitl_log_file:
            try:
                self.sitl_log_file.close()
            except Exception:
                pass
            self.sitl_log_file = None

    def send_cmd(self, p1=0.0, p2=0.0, p3=0.0, p4=0.0, p5=0.0, p6=0.0, p7=1.0,
                 target_sys=1, target_comp=1, src_sys=1, src_comp=191):
        """Belirtilen kaynak/hedef adresleri ve parametrelerle MAV_CMD_USER_1 gönderir."""
        old_sys = self.mav.mav.srcSystem
        old_comp = self.mav.mav.srcComponent
        self.mav.mav.srcSystem = src_sys
        self.mav.mav.srcComponent = src_comp
        try:
            self.mav.mav.command_long_send(
                target_sys, target_comp,
                MAV_CMD_USER_1,
                0,
                float(p1), float(p2), float(p3), float(p4),
                float(p5), float(p6), float(p7)
            )
        finally:
            self.mav.mav.srcSystem = old_sys
            self.mav.mav.srcComponent = old_comp

    def wait_for_lua_boot(self, timeout_s: float = 35.0) -> bool:
        """Lua durum makinesinin BEKLEME durumuna geçtiğini veya hata verdiğini tespit eder."""
        print(f"[SITL] Lua görevinin yüklenmesi bekleniyor (En fazla {timeout_s}s)...", flush=True)
        t0 = time.monotonic()
        cursor = 0
        while (time.monotonic() - t0) < timeout_s:
            if not self.check_heartbeat_liveness():
                return False
            while cursor < len(self.raw_logs):
                text = self.raw_logs[cursor]["text"]
                cursor += 1
                if "S500 Otonom Gorev Scripti yuklendi" in text or "BEKLEME durumunda tetikleme bekleniyor" in text:
                    print(f"[SITL] Lua başarıyla yüklendi: '{text}'", flush=True)
                    return True
                if "attempt to index a nil value" in text or ("error" in text.lower() and "lua" in text.lower()):
                    err = f"Lua çalışma zamanı kritik hatası: '{text}'"
                    print(f"[SITL KRİTİK HATA] {err}", flush=True)
                    self.failure_reasons.append(err)
                    self.critical_failure_occurred = True
                    return False
            time.sleep(0.05)
        err = f"Lua yükleme mesajı {timeout_s}s içinde tespit edilemedi!"
        print(f"[SITL] {err}", flush=True)
        self.failure_reasons.append(err)
        return False

    def validate_app_ack_payload(self, msg, expected_type: int, expected_seq: int, expected_sess: int) -> Tuple[bool, Optional[int], str]:
        """
        APP_ACK mesajını round() kullanmadan, kesin tam sayı ve adres doğrulamasıyla inceler.
        Döner: (gecerli_mi, result, hata_veya_aciklama)
        """
        if msg.get_srcSystem() != 1 or msg.get_srcComponent() != 1:
            return False, None, f"Gecersiz kaynak adresi: sys={msg.get_srcSystem()}, comp={msg.get_srcComponent()}"
        if msg.target_system != 1 or msg.target_component != 191:
            return False, None, f"Gecersiz hedef adresi: sys={msg.target_system}, comp={msg.target_component}"
        if msg.command != MAV_CMD_USER_1:
            return False, None, f"Gecersiz komut id: {msg.command}"

        fields = [
            ("param1 (acked_type)", msg.param1),
            ("param2 (result)", msg.param2),
            ("param4 (acked_seq)", msg.param4),
            ("param5 (status_ack)", msg.param5),
            ("param6 (session_id)", msg.param6),
            ("param7 (protocol_version)", msg.param7)
        ]
        for name, val in fields:
            if not math.isfinite(val):
                return False, None, f"{name} degeri sonlu sayi degil ({val})"
            if not float(val).is_integer():
                return False, None, f"{name} degeri tamsayi degil, ondalikli ({val})"

        p5 = int(msg.param5)
        p7 = float(msg.param7)
        if p5 != STATUS_APP_ACK:
            return False, None, f"param5 STATUS_APP_ACK (171) degil: {p5}"
        if p7 != 1.0:
            return False, None, f"param7 PROTOCOL_VERSION (1.0) degil: {p7}"

        acked_type = int(msg.param1)
        result = int(msg.param2)
        acked_seq = int(msg.param4)
        acked_sess = int(msg.param6)

        if result not in (0, 1):
            return False, None, f"Gecersiz result degeri: {result} (0 veya 1 olmali)"

        if acked_type != expected_type:
            return False, None, f"Beklenen type {expected_type} fakat {acked_type} alindi"
        if acked_seq != expected_seq:
            return False, None, f"Beklenen seq {expected_seq} fakat {acked_seq} alindi"
        if acked_sess != expected_sess:
            return False, None, f"Beklenen session {expected_sess} fakat {acked_sess} alindi"

        return True, result, "Gecerli APP_ACK"

    def wait_for_app_ack(self, expected_type: int, expected_seq: int, expected_sess: int, timeout_s: float = 3.0) -> Tuple[bool, Optional[int], str]:
        """Kuyruktan beklenen APP_ACK paketini çeker."""
        t0 = time.monotonic()
        while (time.monotonic() - t0) < timeout_s:
            if not self.check_heartbeat_liveness():
                return False, None, "Heartbeat kaybedildi veya ARMED tespit edildi"
            try:
                _, msg = self.ack_queue.get(timeout=0.1)
                ok, res, desc = self.validate_app_ack_payload(msg, expected_type, expected_seq, expected_sess)
                if ok:
                    return True, res, desc
            except queue.Empty:
                pass
        return False, None, f"APP_ACK {timeout_s:.1f}s icinde alinamadi"

    def check_log_contains(self, substring: str, start_index: int = 0) -> Tuple[bool, Optional[str]]:
        """start_index sonrasındaki loglarda alt dizgiyi arar."""
        for i in range(start_index, len(self.raw_logs)):
            if substring in self.raw_logs[i]["text"]:
                return True, self.raw_logs[i]["text"]
        return False, None

    def run_tests(self) -> bool:
        self.t_start_mono = time.monotonic()
        print("="*75)
        print("S500 DISARMED SITL MAVLink Entegrasyon Testi Başlatılıyor")
        print("="*75, flush=True)

        all_scenario_names = [
            "1_handshake",
            "2_liveliness",
            "3_invalid_rejection",
            "4_dedup_and_conflict",
            "5_session_management",
            "6_disarmed_safety"
        ]

        try:
            self.start_sitl()

            lua_ok = self.wait_for_lua_boot(timeout_s=35.0)
            if not lua_ok:
                print("\n[BLOCKED] Lua scripti SITL ortamında yüklenemedi / hata verdi; testler bloke oldu.", flush=True)
                for s in all_scenario_names:
                    self.scenario_results[s] = {
                        "status": "BLOCKED",
                        "reason": "Lua runtime hatası veya yükleme zaman aşımı",
                        "evidence": self.failure_reasons
                    }
                return False

            time.sleep(1.0)
            active_session = 12345

            # -----------------------------------------------------------------
            # SENARYO 1: SESSION_START ve APP_ACK El Sıkışması
            # -----------------------------------------------------------------
            print("\n[SENARYO 1] SESSION_START (0xAC) el sıkışması gönderiliyor...", flush=True)
            self.send_cmd(p4=1.0, p5=STATUS_SESSION_START, p6=active_session, p7=1.0)
            ok, res, desc = self.wait_for_app_ack(STATUS_SESSION_START, 1, active_session, timeout_s=3.0)
            if ok and res == 0:
                print("  [PASS] SENARYO 1: SESSION_START başarıyla kabul edildi (APP_ACK result=0).", flush=True)
                self.scenario_results["1_handshake"] = {"status": "PASS", "result": res, "desc": desc}
            else:
                print(f"  [FAIL] SENARYO 1: {desc} (res={res})", flush=True)
                self.scenario_results["1_handshake"] = {"status": "FAIL", "reason": desc, "result": res}

            # -----------------------------------------------------------------
            # SENARYO 2: Aktif Oturumdan seq=0 Canlılık Mesajı Kabulü
            # -----------------------------------------------------------------
            print("\n[SENARYO 2] Aktif oturumdan seq=0 canlılık mesajı (0xAA) gönderiliyor...", flush=True)
            log_cursor = len(self.raw_logs)
            self.send_cmd(p1=1.0, p4=0.0, p5=STATUS_LIVELINESS, p6=active_session, p7=1.0)
            time.sleep(0.5)

            # Doğrulama a: Canlılık için red ACK'si veya hata logu OLMAMALI
            has_err, _ = self.check_log_contains("Reddedildi", log_cursor)
            has_warn, _ = self.check_log_contains("[WARN]", log_cursor)

            # Doğrulama b (Alıcı Canlılık Sondası): Alıcı hattının çalıştığını teyit et
            self.send_cmd(p4=1.0, p5=STATUS_SESSION_START, p6=active_session, p7=1.0)
            probe_ok, probe_res, _ = self.wait_for_app_ack(STATUS_SESSION_START, 1, active_session, timeout_s=2.0)

            # Doğruluk kuralı: İç zaman damgası (last_msg_time_ms) telemetriden doğrudan gözlenemediği için
            # alıcı yanıtı tek başına PASS için yeterli sayılmaz; UNVERIFIED raporlanır.
            if (not has_err) and (not has_warn) and probe_ok and probe_res == 0:
                print("  [UNVERIFIED] SENARYO 2: Canlılık alıcı hattını tıkamadı (sonda ACK=0 alındı), ancak telemetriden iç zaman damgası doğrudan okunamadığı için protokole uygun olarak UNVERIFIED kaydedildi.", flush=True)
                self.scenario_results["2_liveliness"] = {
                    "status": "UNVERIFIED",
                    "reason": "Alıcı sondası teyit edildi, fakat iç zaman damgası güncellemesi telemetriden doğrudan gözlenemedi"
                }
            else:
                print("  [FAIL] SENARYO 2: Canlılık sonrası alıcı sondası başarısız veya hata logu görüldü.", flush=True)
                self.scenario_results["2_liveliness"] = {
                    "status": "FAIL",
                    "reason": "Canlılık sonrası alıcı yanıt vermedi veya hata üretti"
                }

            # -----------------------------------------------------------------
            # SENARYO 3: Geçersiz Kaynak/Hedef, Sürüm ve Ondalıklı Alanların Reddi
            # -----------------------------------------------------------------
            print("\n[SENARYO 3] Geçersiz alan ve kaynak denetimleri (otopilot log kanıtlı)...", flush=True)
            s3_tests = [
                ("3a_unauthorized_source", {"p4": 2.0, "p5": STATUS_START_FOUND, "p6": active_session, "p7": 1.0, "src_sys": 2, "src_comp": 100}, "Yetkisiz kaynak adresi"),
                ("3b_invalid_target",      {"p4": 2.0, "p5": STATUS_START_FOUND, "p6": active_session, "p7": 1.0, "target_sys": 1, "target_comp": 5}, "Gecersiz hedef adresi"),
                ("3c_unsupported_version", {"p4": 2.0, "p5": STATUS_START_FOUND, "p6": active_session, "p7": 2.0}, "Desteklenmeyen protokol surumu"),
                ("3d_decimal_version",     {"p4": 2.0, "p5": STATUS_START_FOUND, "p6": active_session, "p7": 1.2}, "Desteklenmeyen protokol surumu"),
                ("3e_decimal_seq",         {"p4": 2.2, "p5": STATUS_START_FOUND, "p6": active_session, "p7": 1.0}, "Gecersiz seq"),
                ("3f_decimal_status",      {"p4": 2.0, "p5": 3.2, "p6": active_session, "p7": 1.0}, "Bilinmeyen veya gecersiz status"),
                ("3g_decimal_session",     {"p4": 2.0, "p5": STATUS_START_FOUND, "p6": 12345.2, "p7": 1.0}, "Gecersiz session_id"),
            ]

            s3_results = {}
            for name, kwargs, expected_log_sub in s3_tests:
                l_cursor = len(self.raw_logs)
                self.send_cmd(**kwargs)
                time.sleep(0.4)
                ack_seen, ack_res, _ = self.wait_for_app_ack(int(kwargs.get("p5", 1)), int(kwargs.get("p4", 2)), active_session, timeout_s=0.5)
                log_seen, log_text = self.check_log_contains(expected_log_sub, l_cursor)

                if (not ack_seen or ack_res == 1) and log_seen:
                    s3_results[name] = {"status": "PASS", "evidence_log": log_text}
                elif not ack_seen and not log_seen:
                    s3_results[name] = {"status": "UNVERIFIED", "reason": f"Paket ACK almadı ancak beklenen '{expected_log_sub}' logu gözlenemedi"}
                else:
                    s3_results[name] = {"status": "FAIL", "reason": f"Geçersiz paket başarı ACK aldı (res={ack_res})"}

            all_s3_pass = all(v["status"] == "PASS" for v in s3_results.values())
            self.scenario_results["3_invalid_rejection"] = {
                "status": "PASS" if all_s3_pass else "FAIL",
                "subtests": s3_results
            }
            print(f"  [{'PASS' if all_s3_pass else 'FAIL'}] SENARYO 3 tamamlandı. Alt testler: { {k: v['status'] for k, v in s3_results.items()} }", flush=True)

            # -----------------------------------------------------------------
            # SENARYO 4: Tekrar Mesajlar ve Çakışan İçerik Reddi
            # -----------------------------------------------------------------
            print("\n[SENARYO 4] Tekrar mesaj ve içerik çakışması denetimleri...", flush=True)
            self.send_cmd(p1=10.0, p2=5.0, p3=0.90, p4=2.0, p5=STATUS_START_FOUND, p6=active_session, p7=1.0)
            ok_a, res_a, _ = self.wait_for_app_ack(STATUS_START_FOUND, 2, active_session, timeout_s=3.0)

            self.send_cmd(p1=10.0, p2=5.0, p3=0.90, p4=2.0, p5=STATUS_START_FOUND, p6=active_session, p7=1.0)
            ok_b, res_b, _ = self.wait_for_app_ack(STATUS_START_FOUND, 2, active_session, timeout_s=3.0)

            self.send_cmd(p1=10.0, p2=5.0, p3=0.50, p4=2.0, p5=STATUS_GOAL_FOUND, p6=active_session, p7=1.0)
            ok_c, res_c, _ = self.wait_for_app_ack(STATUS_GOAL_FOUND, 2, active_session, timeout_s=3.0)

            if (ok_a and res_a == 0) and (ok_b and res_b == 0) and (ok_c and res_c == 1):
                print("  [PASS] SENARYO 4: İlk paket kabul edildi (0), tekrar teyit edildi (0), çakışan seq reddedildi (1).", flush=True)
                self.scenario_results["4_dedup_and_conflict"] = {
                    "status": "PASS",
                    "evidence": "4a(ACK=0), 4b(Tekrar ACK=0), 4c(Çakışma ACK=1)"
                }
            else:
                print(f"  [FAIL] SENARYO 4: (a: ok={ok_a}, res={res_a}), (b: ok={ok_b}, res={res_b}), (c: ok={ok_c}, res={res_c})", flush=True)
                self.scenario_results["4_dedup_and_conflict"] = {
                    "status": "FAIL",
                    "details": {"4a": (ok_a, res_a), "4b": (ok_b, res_b), "4c": (ok_c, res_c)}
                }

            # -----------------------------------------------------------------
            # SENARYO 5: Yeni Oturum Geçişi ve Kapatılmış Oturum Reddi
            # -----------------------------------------------------------------
            print("\n[SENARYO 5] Yeni oturum geçişi ve kapatılmış oturum denetimleri...", flush=True)
            new_session = 99999
            self.send_cmd(p4=1.0, p5=STATUS_SESSION_START, p6=new_session, p7=1.0)
            ok_new, res_new, _ = self.wait_for_app_ack(STATUS_SESSION_START, 1, new_session, timeout_s=3.0)

            self.send_cmd(p4=3.0, p5=STATUS_START_FOUND, p6=active_session, p7=1.0)
            ok_old, res_old, _ = self.wait_for_app_ack(STATUS_START_FOUND, 3, active_session, timeout_s=3.0)

            if (ok_new and res_new == 0) and (ok_old and res_old == 1):
                print("  [PASS] SENARYO 5: Yeni oturum açıldı (0), kapatılmış eski oturum reddedildi (1).", flush=True)
                self.scenario_results["5_session_management"] = {
                    "status": "PASS",
                    "evidence": "Yeni oturum ACK=0, eski oturum ACK=1"
                }
            else:
                print(f"  [FAIL] SENARYO 5: (yeni: ok={ok_new}, res={res_new}), (eski: ok={ok_old}, res={res_old})", flush=True)
                self.scenario_results["5_session_management"] = {
                    "status": "FAIL",
                    "details": {"new_session": (ok_new, res_new), "retired_session": (ok_old, res_old)}
                }

            # -----------------------------------------------------------------
            # SENARYO 6: Handshake ve Test Boyunca DISARMED Kalma Güvenliği
            # -----------------------------------------------------------------
            print("\n[SENARYO 6] DISARMED telemetri ve güvenlik denetimi...", flush=True)
            hb_alive = self.check_heartbeat_liveness(max_age_s=3.0)
            if hb_alive and (not self.critical_failure_occurred) and (self.is_armed is False):
                print("  [PASS] SENARYO 6: Test süresince araç kesinlikle DISARMED kaldı; arm veya kalkış tetiklenmedi.", flush=True)
                self.scenario_results["6_disarmed_safety"] = {
                    "status": "PASS",
                    "evidence": "Heartbeat sürekli taze, is_armed=False, armed_violation=False"
                }
            else:
                print(f"  [FAIL] SENARYO 6: Güvenlik ihlali! (hb_alive={hb_alive}, critical_failure={self.critical_failure_occurred}, is_armed={self.is_armed})", flush=True)
                self.scenario_results["6_disarmed_safety"] = {
                    "status": "FAIL",
                    "reason": "Heartbeat kaybı veya araç armed duruma geçti"
                }

            overall_pass = (
                not self.critical_failure_occurred and
                len(self.scenario_results) == 6 and
                all(res.get("status") == "PASS" for res in self.scenario_results.values())
            )
            return overall_pass

        except Exception as e:
            err = f"run_tests içinde beklenmeyen istisna: {e}"
            print(f"[RUNNER KRİTİK HATA] {err}", flush=True)
            self.failure_reasons.append(err)
            self.critical_failure_occurred = True
            for s in all_scenario_names:
                if s not in self.scenario_results:
                    self.scenario_results[s] = {
                        "status": "BLOCKED",
                        "reason": f"İstisna nedeniyle çalıştırılamadı: {e}"
                    }
            return False

        finally:
            self.stop_sitl()
            self.save_report()

    def save_report(self):
        duration = time.monotonic() - self.t_start_mono if self.t_start_mono > 0 else 0.0
        overall_pass = (
            not self.critical_failure_occurred and
            len(self.scenario_results) == 6 and
            all(res.get("status") == "PASS" for res in self.scenario_results.values())
        )
        report_data = {
            "timestamp": time.strftime("%Y-%m-%d %H:%M:%S"),
            "duration_s": round(duration, 2),
            "overall_passed": overall_pass,
            "critical_failure_occurred": self.critical_failure_occurred,
            "lua_sha256": get_file_sha256(os.path.join(self.sitl_dir, "scripts/s500_mission.lua")),
            "failure_reasons": self.failure_reasons,
            "scenarios": self.scenario_results,
            "raw_log_count": len(self.raw_logs),
            "raw_logs": self.raw_logs
        }
        report_path = os.path.join(self.sitl_dir, "report_disarmed_sitl.json")
        try:
            with open(report_path, "w") as f:
                json.dump(report_data, f, indent=2)
            print(f"\n[RAPOR] Test sonuçları JSON dosyasına kaydedildi: {report_path}", flush=True)
        except Exception as e:
            print(f"[RAPOR HATA] Rapor dosyası yazılamadı: {e}", flush=True)

if __name__ == "__main__":
    runner = SitlDisarmedTestRunner()
    success = runner.run_tests()
    print("\n" + "="*75)
    print("TEST SONUÇLARI ÖZETİ:")
    for k, v in runner.scenario_results.items():
        st = v.get("status", "UNKNOWN")
        print(f"  - {k}: {st}")
    print("="*75)
    sys.exit(0 if success else 1)
