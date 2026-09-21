#!/usr/bin/env python3
"""
S500 AP_Scripting SITL Kapsamlı Doğrulama ve Test Koşucusu (run_scenario_in_docker.py)
---------------------------------------------------------------------------------------
ArduPilot SITL (ArduCopter V4.7.1) ortamında s500_mission.lua durum makinesini ve
telemetrisini uçtan uca doğrular.

MİMARİ VE GÜVENLİK İLKELERİ:
1. Her sonuçta (başarı, zaman aşımı, istisna) eksiksiz JSON raporu üretimi.
2. SITL çıktısının dosyaya yönlendirilmesi (pipe deadlock engeli) ve tam süreç temizliği.
3. Monotonik zaman, tazelik ve math.isfinite kontrollü telemetri doğrulaması.
4. Başlangıçta is_armed=None, uçuşta armed teyidi, inişte fiziksel irtifa + disarm teyidi.
5. Seyir irtifa bandının (31-35m) yalnızca seyir durumlarında denetlenmesi ve ihlalde başarısızlık.
6. MAVLink 2 parçalı STATUSTEXT birleştirme ve katı log_cursor sırası.
7. Çıkış kodu başarısızlıkta deterministik olarak 1, başarıda 0.
"""

import os
import sys
import time
import math
import json
import signal
import hashlib
import argparse
import subprocess
import threading
from typing import Optional, Dict, Any, List, Tuple
from pymavlink import mavutil

MAV_CMD_USER_1 = 31010

STATUS_NO_DETECTION     = 0
STATUS_START_FOUND      = 1
STATUS_GOAL_FOUND       = 2
STATUS_GOTO_OBSERVATION = 3
STATUS_ROUTE_READY      = 4
STATUS_GCS_DELIVERED    = 5

STATUS_NAMES = {
    0: "NO_DETECTION",
    1: "START_FOUND",
    2: "GOAL_FOUND",
    3: "GOTO_OBSERVATION",
    4: "ROUTE_READY",
    5: "GCS_DELIVERED"
}

def get_file_sha256(filepath: str) -> str:
    if not os.path.exists(filepath):
        return "DOSYA_YOK"
    h = hashlib.sha256()
    with open(filepath, "rb") as f:
        while chunk := f.read(8192):
            h.update(chunk)
    return h.hexdigest()

def get_sitl_version(binary_path: str = "/opt/ardupilot/build/sitl/bin/arducopter") -> str:
    """SITL ikili dosyasından veya kaynak dosyalarından gerçek sürüm bilgisini okur."""
    if os.path.exists(binary_path):
        try:
            res = subprocess.run(
                ["strings", binary_path],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                check=False
            )
            for line in res.stdout.splitlines():
                if "ArduCopter V" in line and "(" in line and ")" in line:
                    return line.strip()
        except Exception:
            pass

    version_str = "ArduCopter"
    vh_path = "/opt/ardupilot/ArduCopter/version.h"
    if os.path.exists(vh_path):
        try:
            with open(vh_path, "r") as f:
                for line in f:
                    if "#define THISFIRMWARE" in line:
                        version_str = line.split('"')[1]
                        break
        except Exception:
            pass

    try:
        git_res = subprocess.run(
            ["git", "-C", "/opt/ardupilot", "rev-parse", "--short", "HEAD"],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            check=False
        )
        if git_res.returncode == 0:
            commit = git_res.stdout.strip()
            return f"{version_str} (Commit {commit})"
    except Exception:
        pass

    return version_str

class StatustextAssembler:
    """MAVLink parçalı STATUSTEXT mesajlarını ham bayt düzeyinde birleştiren yardımcı sınıf."""
    def __init__(self):
        self.chunks: Dict[int, Dict[int, bytes]] = {}
        self.final_seqs: Dict[int, int] = {}

    def feed(self, msg) -> Optional[str]:
        chunk_id = getattr(msg, 'id', 0)
        chunk_seq = getattr(msg, 'chunk_seq', 0)

        # Ham bayt verisini al (UTF-8 karakter sayısı yerine ham bayt uzunluğu ile ölçüm)
        if hasattr(msg, '_text_raw') and msg._text_raw is not None:
            raw_payload = bytes(msg._text_raw)
        elif isinstance(msg.text, bytes):
            raw_payload = msg.text
        elif isinstance(msg.text, str):
            raw_payload = msg.text.encode('utf-8', errors='ignore')
        else:
            raw_payload = b""

        # Null sonlandırıcıya kadar olan baytları ayıkla
        if b'\x00' in raw_payload:
            chunk_bytes = raw_payload.split(b'\x00', 1)[0]
            is_last = True
        else:
            chunk_bytes = raw_payload
            is_last = (len(chunk_bytes) < 50)

        # Tek parça mesaj (id == 0)
        if chunk_id == 0:
            try:
                return chunk_bytes.decode('utf-8', errors='replace').strip()
            except Exception:
                return chunk_bytes.decode('latin-1', errors='ignore').strip()

        # Parçalı mesaj (id > 0)
        if chunk_id not in self.chunks:
            self.chunks[chunk_id] = {}

        self.chunks[chunk_id][chunk_seq] = chunk_bytes

        if is_last:
            self.final_seqs[chunk_id] = chunk_seq

        # Eğer son parçanın seq numarası biliniyorsa ve 0..final_seq arasındaki tüm parçalar eksiksiz mevcutsa birleştir
        if chunk_id in self.final_seqs:
            final_seq = self.final_seqs[chunk_id]
            if all(s in self.chunks[chunk_id] for s in range(final_seq + 1)):
                all_bytes = b"".join(self.chunks[chunk_id][s] for s in range(final_seq + 1))
                del self.chunks[chunk_id]
                del self.final_seqs[chunk_id]
                try:
                    full_text = all_bytes.decode('utf-8', errors='replace').strip()
                except Exception:
                    full_text = all_bytes.decode('latin-1', errors='ignore').strip()
                return full_text

        return None

class SitlScenarioRunner:
    def __init__(self, scenario: str, sitl_dir: str = "/workspace/sitl_test_run"):
        self.scenario = scenario
        self.sitl_dir = sitl_dir
        self.sitl_proc: Optional[subprocess.Popen] = None
        self.sitl_log_file = None
        self.mav = None
        self.rx_thread: Optional[threading.Thread] = None
        self.running = True

        # Telemetri Durumu (Monotonik zaman ve geçerlilik takipli)
        self.current_ned_pos: Optional[Tuple[float, float, float]] = None # (x, y, z)
        self.last_pos_mono: float = 0.0
        self.current_yaw_rad: Optional[float] = None
        self.last_yaw_mono: float = 0.0
        self.current_vel_ned: Optional[Tuple[float, float, float]] = None

        self.is_armed: Optional[bool] = None # Başlangıçta kesinlikle bilinmiyor
        self.last_heartbeat_mono: float = 0.0
        self.was_armed_seen: bool = False

        self.active_disarm_delay: Optional[float] = None

        # Uçuş Ölçümleri ve Referanslar
        self.takeoff_freeze_pos: Optional[Tuple[float, float, float]] = None
        self.takeoff_freeze_yaw: Optional[float] = None
        self.takeoff_reference_frozen: bool = False

        # Seyir İrtifa Bandı İzleme (Yalnızca seyir durumlarında aktifleştirilir)
        self.cruise_monitoring_active: bool = False
        self.min_cruise_alt: Optional[float] = None
        self.max_cruise_alt: Optional[float] = None
        self.cruise_altitude_breached: bool = False

        self.kalkis_dist_achieved: Optional[float] = None
        self.kalkis_alt_achieved: Optional[float] = None
        self.obs_dist_achieved: Optional[float] = None
        self.return_dist_achieved: Optional[float] = None
        self.touchdown_alt_achieved: Optional[float] = None
        self.final_disarmed_confirmed: bool = False

        # Log Takibi (Monotonik Cursor)
        self.assembler = StatustextAssembler()
        self.received_statustexts: List[Tuple[float, str]] = []
        self.log_cursor: int = 0
        self.failure_reasons: List[str] = []
        self.rx_fatal_error: bool = False
        self.test_passed: bool = False

    def log_statustext(self, text: str):
        now = time.monotonic()
        self.received_statustexts.append((now, text))
        print(f"  [GCS-LOG] {text}", flush=True)

    def _rx_worker(self):
        while self.running and self.mav:
            try:
                msg = self.mav.recv_match(blocking=True, timeout=0.2)
                if not msg:
                    continue

                # Yalnızca hedef otopilot (SysID=1, CompID=1) mesajlarını işle
                src_sys = msg.get_srcSystem()
                src_comp = msg.get_srcComponent()
                if src_sys != 1 or src_comp != 1:
                    continue

                now_mono = time.monotonic()
                mtype = msg.get_type()

                if mtype == 'STATUSTEXT':
                    assembled = self.assembler.feed(msg)
                    if assembled and "[S500 LUA]" in assembled:
                        self.log_statustext(assembled)

                elif mtype == 'LOCAL_POSITION_NED':
                    if (math.isfinite(msg.x) and math.isfinite(msg.y) and math.isfinite(msg.z) and
                        math.isfinite(msg.vx) and math.isfinite(msg.vy) and math.isfinite(msg.vz)):
                        self.current_ned_pos = (float(msg.x), float(msg.y), float(msg.z))
                        self.current_vel_ned = (float(msg.vx), float(msg.vy), float(msg.vz))
                        self.last_pos_mono = now_mono

                        # Seyir irtifa bandı izleme (Sadece kalkış bittikten inişe kadar)
                        if self.cruise_monitoring_active and self.takeoff_freeze_pos:
                            rel_alt = -(self.current_ned_pos[2] - self.takeoff_freeze_pos[2])
                            if math.isfinite(rel_alt):
                                if self.min_cruise_alt is None or rel_alt < self.min_cruise_alt:
                                    self.min_cruise_alt = rel_alt
                                if self.max_cruise_alt is None or rel_alt > self.max_cruise_alt:
                                    self.max_cruise_alt = rel_alt
                                # 31.0 - 35.0m bandı ihlal kontrolü
                                if rel_alt < 31.0 or rel_alt > 35.0:
                                    self.cruise_altitude_breached = True

                elif mtype == 'ATTITUDE':
                    if math.isfinite(msg.yaw):
                        self.current_yaw_rad = float(msg.yaw)
                        self.last_yaw_mono = now_mono

                elif mtype == 'HEARTBEAT':
                    armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                    self.is_armed = armed
                    self.last_heartbeat_mono = now_mono
                    if armed:
                        self.was_armed_seen = True

                elif mtype == 'PARAM_VALUE':
                    p_id = msg.param_id
                    if isinstance(p_id, bytes):
                        p_id = p_id.decode('utf-8', errors='ignore')
                    p_id = p_id.strip('\x00').strip()
                    if p_id == 'DISARM_DELAY' and math.isfinite(msg.param_value):
                        self.active_disarm_delay = float(msg.param_value)

            except Exception as e:
                if self.running:
                    err = f"Kritik MAVLink RX İş Parçacığı Hatası: {e}"
                    print(f"[RUNNER] {err}", flush=True)
                    self.failure_reasons.append(err)
                    self.rx_fatal_error = True

    def is_pos_fresh(self, max_age_s: float = 2.0) -> bool:
        return (self.current_ned_pos is not None and 
                (time.monotonic() - self.last_pos_mono) <= max_age_s and
                math.isfinite(self.current_ned_pos[0]) and
                math.isfinite(self.current_ned_pos[1]) and
                math.isfinite(self.current_ned_pos[2]))

    def is_yaw_fresh(self, max_age_s: float = 2.0) -> bool:
        return (self.current_yaw_rad is not None and 
                (time.monotonic() - self.last_yaw_mono) <= max_age_s and
                math.isfinite(self.current_yaw_rad))

    def wait_for_statustext(self, substring: str, timeout_s: float) -> bool:
        t0 = time.monotonic()
        print(f"[RUNNER] Bekleniyor ({timeout_s:.1f}s, cursor={self.log_cursor}): '{substring}'", flush=True)

        while (time.monotonic() - t0) < timeout_s:
            while self.log_cursor < len(self.received_statustexts):
                entry_time, text = self.received_statustexts[self.log_cursor]
                self.log_cursor += 1
                if substring in text:
                    print(f"[RUNNER] Teyit Edildi -> '{text}'", flush=True)
                    return True
            time.sleep(0.05)

        err_msg = f"Zaman aşımı ({timeout_s:.1f}s)! Beklenen durum logu bulunamadı: '{substring}'"
        print(f"[RUNNER] HATA: {err_msg}", flush=True)
        self.failure_reasons.append(err_msg)
        return False

    def send_user_cmd(self, seq: int, status: int, x: float = 0.0, y: float = 0.0, conf: float = 1.0):
        s_name = STATUS_NAMES.get(status, str(status))
        print(f"[TX] MAV_CMD_USER_1 -> Seq: {seq}, Status: {status} ({s_name}), X: {x:.1f}m, Y: {y:.1f}m, Conf: {conf:.2f}", flush=True)
        self.mav.mav.command_long_send(
            1, 1,
            MAV_CMD_USER_1,
            0,
            float(x), float(y), float(conf), float(seq), float(status),
            0.0, 0.0
        )

    def run(self) -> bool:
        t_start_mono = time.monotonic()
        self.test_passed = False

        lua_path = os.path.join(self.sitl_dir, "scripts/s500_mission.lua")
        lua_hash = get_file_sha256(lua_path)

        sitl_stdout_log_path = os.path.join(self.sitl_dir, f"sitl_stdout_{self.scenario}.log")
        report_file = os.path.join(self.sitl_dir, f"report_{self.scenario}.json")

        print("="*75)
        print(f"S500 AP_Scripting SITL Doğrulama Başlatılıyor: Senaryo '{self.scenario}'")
        print(f"Lua Script Yolu : {lua_path}")
        print(f"Lua SHA256 Hash : {lua_hash}")
        print(f"SITL Çıktı Logu : {sitl_stdout_log_path}")
        print("="*75, flush=True)

        try:
            # Kontrollü Hata Testi Modu:
            # Gerçek SITL sürecini başlatır, ancak gerçekleşmeyecek bir logu kasıtlı olarak bekleyip
            # çıkış kodunun 1, passed=false ve sürecin temizlendiğini doğrulamak için kullanılır.
            if self.scenario == "test_failure":
                print("[TEST_FAILURE] Kontrollü başarısızlık testi: Kısa zaman aşımı ile hata yolu tetikleniyor...", flush=True)
                self.sitl_log_file = open(sitl_stdout_log_path, "w")
                self.sitl_proc = subprocess.Popen(
                    [
                        "/opt/ardupilot/build/sitl/bin/arducopter",
                        "--model", "quad",
                        "--home", "-35.363261,149.165230,584,353",
                        "--defaults", "/opt/ardupilot/Tools/autotest/default_params/copter.parm,/workspace/sitl_test_run/extra_params.parm"
                    ],
                    cwd=self.sitl_dir,
                    stdout=self.sitl_log_file,
                    stderr=subprocess.STDOUT,
                    preexec_fn=os.setsid
                )
                time.sleep(1.0)
                # İmkansız bir logu 2 saniye bekle -> zaman aşımına uğramalı ve başarısız olmalı
                if not self.wait_for_statustext("GERCEKLESMEZ_KASITLI_HATA_LOGU", timeout_s=2.0):
                    self.test_passed = False
                    return False

            # 1. SITL Sürecini Başlat (Çıktıyı doğrudan log dosyasına akıt - pipe deadlock engeli)
            self.sitl_log_file = open(sitl_stdout_log_path, "w")
            sitl_cmd = [
                "/opt/ardupilot/build/sitl/bin/arducopter",
                "--model", "quad",
                "--home", "-35.363261,149.165230,584,353",
                "--defaults", "/opt/ardupilot/Tools/autotest/default_params/copter.parm,/workspace/sitl_test_run/extra_params.parm"
            ]
            print(f"[RUNNER] SITL başlatılıyor: {' '.join(sitl_cmd)}", flush=True)
            self.sitl_proc = subprocess.Popen(
                sitl_cmd,
                cwd=self.sitl_dir,
                stdout=self.sitl_log_file,
                stderr=subprocess.STDOUT,
                preexec_fn=os.setsid
            )

            # 2. MAVLink Bağlantısı ve Heartbeat Doğrulaması (Kullanılmayan soketleri kapat)
            t_conn_start = time.monotonic()
            conn_ok = False
            while (time.monotonic() - t_conn_start) < 20.0:
                temp_mav = None
                try:
                    temp_mav = mavutil.mavlink_connection('tcp:127.0.0.1:5760')
                    hb = temp_mav.wait_heartbeat(timeout=2.0)
                    if hb and hb.get_srcSystem() == 1 and hb.get_srcComponent() == 1:
                        self.mav = temp_mav
                        conn_ok = True
                        print(f"[RUNNER] Heartbeat alındı! (SysID={self.mav.target_system}, CompID={self.mav.target_component})", flush=True)
                        break
                    else:
                        if temp_mav:
                            temp_mav.close()
                except Exception:
                    if temp_mav:
                        temp_mav.close()
                    time.sleep(0.5)

            if not conn_ok or not self.mav:
                err = "SITL port 5760 üzerinden Heartbeat alınamadı (20s zaman aşımı)!"
                print(f"[RUNNER] HATA: {err}", flush=True)
                self.failure_reasons.append(err)
                return False

            # Telemetri dinleyicisini başlat
            self.rx_thread = threading.Thread(target=self._rx_worker, daemon=True)
            self.rx_thread.start()

            # Telemetri yayın sıklıklarını talep et (Tüm yayınlar + MsgID 32: LOCAL_POSITION_NED @ 10Hz, MsgID 30: ATTITUDE @ 10Hz)
            try:
                self.mav.mav.request_data_stream_send(1, 1, mavutil.mavlink.MAV_DATA_STREAM_ALL, 10, 1)
                self.mav.mav.command_long_send(1, 1, mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0, 32, 100000, 0, 0, 0, 0, 0)
                self.mav.mav.command_long_send(1, 1, mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0, 30, 100000, 0, 0, 0, 0, 0)
            except Exception as e:
                err = f"Mesaj aralığı komutu gönderilemedi: {e}"
                print(f"[RUNNER] UYARI: {err}", flush=True)

            # 3. DISARM_DELAY Parametre Değerini Otopilottan Sorgula ve Doğrula
            print("[RUNNER] DISARM_DELAY parametresi otopilottan sorgulanıyor...", flush=True)
            self.mav.mav.param_request_read_send(1, 1, b'DISARM_DELAY', -1)
            t_param = time.monotonic()
            while (time.monotonic() - t_param) < 10.0 and self.active_disarm_delay is None:
                time.sleep(0.2)

            if self.active_disarm_delay is None:
                err = "DISARM_DELAY parametresi otopilottan okunamadı!"
                print(f"[RUNNER] HATA: {err}", flush=True)
                self.failure_reasons.append(err)
                return False

            print(f"[RUNNER] Otopilottan Okunan Etkin DISARM_DELAY: {self.active_disarm_delay:.1f}s", flush=True)
            if self.active_disarm_delay <= 0.0:
                err = f"DISARM_DELAY={self.active_disarm_delay} geçersiz! Otomatik iniş disarmı için > 0 olmalıdır."
                print(f"[RUNNER] HATA: {err}", flush=True)
                self.failure_reasons.append(err)
                return False

            # 4. Lua Script Yükleme ve İlk BEKLEME Durumu Teyidi
            if not self.wait_for_statustext("BEKLEME durumunda tetikleme bekleniyor", timeout_s=30.0):
                return False

            # 5. EKF ve Konum Hazırlığını Dinamik Olarak Bekle (SITL EKF origin oturması ~35-45s sürer)
            print("[RUNNER] EKF ve taze yerel telemetri bekleniyor...", flush=True)
            t_ekf = time.monotonic()
            ekf_ready = False
            while (time.monotonic() - t_ekf) < 60.0:
                if self.is_pos_fresh(max_age_s=2.0) and self.is_yaw_fresh(max_age_s=2.0):
                    ekf_ready = True
                    break
                time.sleep(0.5)

            if not ekf_ready:
                err = "EKF origin / LOCAL_POSITION_NED taze verisi 60s içinde hazır olmadı!"
                print(f"[RUNNER] HATA: {err}", flush=True)
                self.failure_reasons.append(err)
                return False

            # 6. Tetikleme: GUIDED Moda Geçiş
            print("[RUNNER] Görev tetikleniyor: GUIDED moda geçiş komutu gönderiliyor...", flush=True)
            self.mav.set_mode('GUIDED')

            # 7. HAZIRLIK Durumu ve Örnekleme Doğrulaması
            if not self.wait_for_statustext("Durum Gecisi: BEKLEME -> HAZIRLIK", timeout_s=15.0):
                return False

            # hazirlik_samples boyutunun tam 5'te sabitlendiğini doğrula
            if not self.wait_for_statustext("hazirlik_samples boyutu: 5, ornekleme durduruldu", timeout_s=30.0):
                return False

            # Yerde HAREKETSİZKEN kalkış fiziksel koordinatlarını ve yaw açısını dondur
            if not self.is_pos_fresh(max_age_s=2.0) or not self.is_yaw_fresh(max_age_s=2.0):
                err = "HAZIRLIK aşamasında dondurulacak telemetri verisi bayat veya geçersiz!"
                print(f"[RUNNER] HATA: {err}", flush=True)
                self.failure_reasons.append(err)
                return False

            self.takeoff_freeze_pos = self.current_ned_pos
            self.takeoff_freeze_yaw = self.current_yaw_rad
            self.takeoff_reference_frozen = True
            print(f"[RUNNER] Kalkış Referansı YERDE Donduruldu: NED={self.takeoff_freeze_pos}, Yaw={math.degrees(self.takeoff_freeze_yaw):.1f}°", flush=True)

            # Arm Olma ve KALKIŞ Durumuna Geçiş
            if not self.wait_for_statustext("Durum Gecisi: HAZIRLIK -> KALKIS", timeout_s=30.0):
                return False

            # Uçuş sırasında gerçekten armed olunduğunu doğrula
            time.sleep(1.0)
            if not self.was_armed_seen:
                err = "KALKIS durumuna geçildi ancak otopilot HEARTBEAT telemetrisinde armed görülmedi!"
                print(f"[RUNNER] HATA: {err}", flush=True)
                self.failure_reasons.append(err)
                return False

            # 8. Birleşik 3D Kalkışın Tamamlanması ve HEDEF_BEKLE Geçişi
            if not self.wait_for_statustext("Durum Gecisi: KALKIS -> HEDEF_BEKLE", timeout_s=75.0):
                return False

            # FİZİKSEL KALKIŞ TELEMETRİ DOĞRULAMASI:
            # Hedef: 40m ileri (ILERI_YAW) + 33m irtifa
            if not self.is_pos_fresh(max_age_s=2.0):
                err = "Kalkış noktasına varıldığında taze telemetri konumu alınamadı!"
                print(f"[RUNNER] HATA: {err}", flush=True)
                self.failure_reasons.append(err)
                return False

            t_x, t_y, t_z = self.takeoff_freeze_pos
            exp_kalkis_x = t_x + 40.0 * math.cos(self.takeoff_freeze_yaw)
            exp_kalkis_y = t_y + 40.0 * math.sin(self.takeoff_freeze_yaw)
            cur_x, cur_y, cur_z = self.current_ned_pos
            dx = cur_x - exp_kalkis_x
            dy = cur_y - exp_kalkis_y
            horiz_dist_kalkis = math.sqrt(dx*dx + dy*dy)
            rel_alt_kalkis = -(cur_z - t_z)

            self.kalkis_dist_achieved = horiz_dist_kalkis
            self.kalkis_alt_achieved = rel_alt_kalkis

            print(f"[FİZİKSEL-DENETİM] Kalkış Varış Hatası: Yatay Mesafe={horiz_dist_kalkis:.2f}m (Tolerans < 2.0m), İrtifa={rel_alt_kalkis:.2f}m (31-35m bandı)", flush=True)

            if horiz_dist_kalkis > 2.0:
                err = f"Fiziksel kalkış varış hatası tolerans dışı: {horiz_dist_kalkis:.2f}m > 2.0m!"
                print(f"[RUNNER] HATA: {err}", flush=True)
                self.failure_reasons.append(err)
                return False

            if abs(rel_alt_kalkis - 33.0) > 0.5 or rel_alt_kalkis < 31.0 or rel_alt_kalkis > 35.0:
                err = f"Fiziksel kalkış irtifası tolerans dışı: {rel_alt_kalkis:.2f}m (Beklenen: 33.0 ±0.5m)!"
                print(f"[RUNNER] HATA: {err}", flush=True)
                self.failure_reasons.append(err)
                return False

            print("[FİZİKSEL-DENETİM] Kalkış telemetrisi başarıyla doğrulandı!", flush=True)

            # Seyir irtifa bandı denetimini aktifleştir (Kalkış bitti, seyir başladı)
            self.cruise_monitoring_active = True
            self.min_cruise_alt = rel_alt_kalkis
            self.max_cruise_alt = rel_alt_kalkis

            # =================================================================
            # SENARYO ÖZEL AKIŞLARI
            # =================================================================

            if self.scenario == "a":
                # Senaryo A (Basit Yol): status 0 -> 1 -> 4
                time.sleep(2.0)
                self.send_user_cmd(seq=1, status=STATUS_NO_DETECTION, conf=0.0)
                time.sleep(2.0)
                self.send_user_cmd(seq=2, status=STATUS_START_FOUND, conf=0.92)
                time.sleep(2.0)
                self.send_user_cmd(seq=3, status=STATUS_ROUTE_READY, conf=1.0)

                # ROUTE_READY alindi ve doğrudan DÖNÜŞ'e geçiş teyidi
                if not self.wait_for_statustext("ROUTE_READY alindi (Seq: 3). Dogrudan DONUS durumuna geciliyor.", timeout_s=15.0):
                    return False

            elif self.scenario == "b":
                # Senaryo B (Gözlem Konumlu Yol): status 0 -> 1 -> 3 (x=10, y=5) -> [varış] -> 1 -> 4
                time.sleep(2.0)
                self.send_user_cmd(seq=10, status=STATUS_NO_DETECTION, conf=0.0)
                time.sleep(1.5)
                self.send_user_cmd(seq=11, status=STATUS_START_FOUND, conf=0.88)
                time.sleep(1.5)

                # status=3 (GOTO_OBSERVATION, x=10.0m, y=5.0m)
                target_obs_x, target_obs_y = 10.0, 5.0
                self.send_user_cmd(seq=12, status=STATUS_GOTO_OBSERVATION, x=target_obs_x, y=target_obs_y, conf=0.95)

                if not self.wait_for_statustext("GOTO_OBSERVATION alindi (Seq: 12, Hedef: [10.0, 5.0], Conf: 0.95)", timeout_s=15.0):
                    return False

                # Hedefe varış ve HEDEF_KONUMUNDA_BEKLE geçişi
                if not self.wait_for_statustext("Durum Gecisi: HEDEFE_GIT -> HEDEF_KONUMUNDA_BEKLE", timeout_s=75.0):
                    return False

                # FİZİKSEL GÖZLEM NOKTASI VARIŞ TELEMETRİ DENETİMİ:
                if not self.is_pos_fresh(max_age_s=2.0):
                    err = "Gözlem noktasında taze telemetri alınamadı!"
                    print(f"[RUNNER] HATA: {err}", flush=True)
                    self.failure_reasons.append(err)
                    return False

                cur_x, cur_y, _ = self.current_ned_pos
                dx_obs = cur_x - target_obs_x
                dy_obs = cur_y - target_obs_y
                dist_obs = math.sqrt(dx_obs*dx_obs + dy_obs*dy_obs)
                self.obs_dist_achieved = dist_obs
                print(f"[FİZİKSEL-DENETİM] Gözlem Noktası Varış Hatası: Mesafe={dist_obs:.2f}m (Tolerans < 1.0m)", flush=True)

                if dist_obs > 1.0:
                    err = f"Gözlem noktasına fiziksel varış hatası tolerans dışı: {dist_obs:.2f}m > 1.0m!"
                    print(f"[RUNNER] HATA: {err}", flush=True)
                    self.failure_reasons.append(err)
                    return False

                time.sleep(2.0)
                self.send_user_cmd(seq=13, status=STATUS_START_FOUND, conf=0.90)
                time.sleep(1.5)
                self.send_user_cmd(seq=14, status=STATUS_ROUTE_READY, conf=1.0)

                if not self.wait_for_statustext("ROUTE_READY alindi (Seq: 14). DONUS durumuna geciliyor.", timeout_s=15.0):
                    return False

            elif self.scenario == "failsafe":
                # Failsafe: Hiç mesaj gönderilmez, 90s beklenir
                print("[RUNNER] FAILSAFE MODU: Mesaj iletilmiyor, 90s zaman aşımı bekleniyor...", flush=True)
                if not self.wait_for_statustext("HEDEF_BEKLE 90s zaman asimi! Ne status=3 ne status=4 alindi. Failsafe DONUS durumuna geciliyor.", timeout_s=105.0):
                    return False

            # =================================================================
            # DÖNÜŞ, İNİŞ VE DİSARM DOĞRULAMASI (TÜM SENARYOLAR İÇİN ZORUNLU)
            # =================================================================

            if not self.wait_for_statustext("Durum Gecisi: DONUS -> INIS", timeout_s=90.0):
                return False

            # Seyir bitti, iniş başladı: seyir irtifa denetimini sonlandır
            self.cruise_monitoring_active = False

            # Seyir irtifa ihlali kontrolü
            if self.cruise_altitude_breached:
                err = f"Seyir sırasında irtifa ihlali tespit edildi! [Min: {self.min_cruise_alt:.2f}m, Max: {self.max_cruise_alt:.2f}m, İzin Verilen: 31-35m]"
                print(f"[RUNNER] HATA: {err}", flush=True)
                self.failure_reasons.append(err)
                return False

            # FİZİKSEL DÖNÜŞ NOKTASI VARIŞ DENETİMİ:
            if not self.is_pos_fresh(max_age_s=2.0):
                err = "Kalkış noktasına geri dönüşte taze telemetri alınamadı!"
                print(f"[RUNNER] HATA: {err}", flush=True)
                self.failure_reasons.append(err)
                return False

            cur_x, cur_y, _ = self.current_ned_pos
            dx_ret = cur_x - t_x
            dy_ret = cur_y - t_y
            dist_ret = math.sqrt(dx_ret*dx_ret + dy_ret*dy_ret)
            self.return_dist_achieved = dist_ret
            print(f"[FİZİKSEL-DENETİM] Kalkış Noktasına Geri Dönüş Hatası: {dist_ret:.2f}m (Tolerans < 1.0m)", flush=True)

            if dist_ret > 1.0:
                err = f"Kalkış noktasına fiziksel dönüş hatası tolerans dışı: {dist_ret:.2f}m > 1.0m!"
                print(f"[RUNNER] HATA: {err}", flush=True)
                self.failure_reasons.append(err)
                return False

            # İniş ve "GÖREV TAMAMLANDI" Log Teyidi
            if not self.wait_for_statustext("GÖREV TAMAMLANDI", timeout_s=60.0):
                return False

            # FİZİKSEL YERE İNİŞ VE DİSARM TELEMETRİ KONTROLÜ (Madde 4):
            # Otomatik disarm için (DISARM_DELAY=10s) beklenir, zorla disarm ASLA gönderilmez!
            print(f"[RUNNER] Otomatik disarm ve yere iniş bekleniyor (DISARM_DELAY={self.active_disarm_delay}s)...", flush=True)
            t_disarm = time.monotonic()
            disarmed = False
            touchdown_verified = False

            while (time.monotonic() - t_disarm) < 30.0:
                if self.is_pos_fresh(max_age_s=2.0):
                    rel_alt_ground = -(self.current_ned_pos[2] - t_z)
                    self.touchdown_alt_achieved = rel_alt_ground
                    # Yere temas: kalkış irtifasına göre < 0.5m
                    if abs(rel_alt_ground) < 0.5:
                        touchdown_verified = True

                # Disarm kontrolü: HEARTBEAT taze ve armed=False
                if (time.monotonic() - self.last_heartbeat_mono) < 3.0 and self.is_armed is False:
                    disarmed = True
                    if touchdown_verified:
                        break
                time.sleep(0.5)

            self.final_disarmed_confirmed = disarmed
            print(f"[FİZİKSEL-DENETİM] Yere Temas Teyidi : {'BAŞARILI' if touchdown_verified else 'BAŞARISIZ'} (İrtifa: {self.touchdown_alt_achieved}m)", flush=True)
            print(f"[FİZİKSEL-DENETİM] Disarm Telemetrisi: {'BAŞARILI' if disarmed else 'BAŞARISIZ'}", flush=True)

            if not touchdown_verified:
                err = f"Yere temas telemetrisi doğrulanamadı! (Son irtifa: {self.touchdown_alt_achieved}m)"
                print(f"[RUNNER] HATA: {err}", flush=True)
                self.failure_reasons.append(err)
                return False

            if not disarmed:
                err = f"Otopilot iniş sonrası 30s içinde otomatik disarm olmadı! (DISARM_DELAY={self.active_disarm_delay}s)"
                print(f"[RUNNER] HATA: {err}", flush=True)
                self.failure_reasons.append(err)
                return False

            # Alıcı iş parçacığı veya test aşamalarında herhangi bir hata kaydedildiyse başarısız say
            if self.rx_fatal_error or len(self.failure_reasons) > 0:
                err = f"Test akışı tamamlandı ancak süreç/alıcı hataları tespit edildi ({len(self.failure_reasons)} hata)!"
                print(f"[RUNNER] HATA: {err}", flush=True)
                self.test_passed = False
                return False

            # Bütün adımlar, loglar ve telemetri sınırları eksiksiz sağlandı!
            self.test_passed = True
            return True

        except Exception as e:
            err = f"Beklenmeyen Test İstisnası: {e}"
            print(f"[RUNNER] KRİTİK İSTİSNA: {err}", flush=True)
            self.failure_reasons.append(err)
            self.test_passed = False
            return False

        finally:
            print("\n[RUNNER] Temizlik işlemleri yürütülüyor...", flush=True)
            self.running = False

            # 1. MAVLink Bağlantısını Kapat ve Alıcı İş Parçacığını Bekle
            if self.mav:
                try:
                    self.mav.close()
                except Exception:
                    pass
                self.mav = None

            if self.rx_thread and self.rx_thread.is_alive():
                self.rx_thread.join(timeout=2.0)

            # 2. SITL Sürecini ve Süreç Grubunu Kapat
            if self.sitl_proc:
                try:
                    os.killpg(os.getpgid(self.sitl_proc.pid), signal.SIGTERM)
                    self.sitl_proc.wait(timeout=4.0)
                except Exception:
                    try:
                        os.killpg(os.getpgid(self.sitl_proc.pid), signal.SIGKILL)
                        self.sitl_proc.wait(timeout=2.0)
                    except Exception:
                        pass
                self.sitl_proc = None

            # 3. SITL Log Dosyasını Kapat
            if self.sitl_log_file:
                try:
                    self.sitl_log_file.close()
                except Exception:
                    pass

            duration_s = time.monotonic() - t_start_mono
            print(f"[RUNNER] Test Süresi: {duration_s:.1f}s | Nihai Sonuç: {'BAŞARILI (PASSED)' if self.test_passed else 'BAŞARISIZ (FAILED)'}", flush=True)

            # 4. HER DURUMDA ve İSTİSNADA EKSİKSİZ JSON RAPORU ÜRETİMİ (Madde 1)
            report = {
                "scenario": self.scenario,
                "passed": self.test_passed,
                "duration_s": round(duration_s, 2),
                "lua_sha256": lua_hash,
                "sitl_version": get_sitl_version(),
                "active_disarm_delay_s": self.active_disarm_delay,
                "failure_reasons": self.failure_reasons,
                "telemetry_metrics": {
                    "takeoff_freeze_pos_ned": self.takeoff_freeze_pos,
                    "takeoff_freeze_yaw_deg": round(math.degrees(self.takeoff_freeze_yaw), 2) if self.takeoff_freeze_yaw else None,
                    "kalkis_dist_achieved_m": round(self.kalkis_dist_achieved, 3) if self.kalkis_dist_achieved is not None else None,
                    "kalkis_alt_achieved_m": round(self.kalkis_alt_achieved, 3) if self.kalkis_alt_achieved is not None else None,
                    "obs_dist_achieved_m": round(self.obs_dist_achieved, 3) if self.obs_dist_achieved is not None else None,
                    "return_dist_achieved_m": round(self.return_dist_achieved, 3) if self.return_dist_achieved is not None else None,
                    "touchdown_alt_achieved_m": round(self.touchdown_alt_achieved, 3) if self.touchdown_alt_achieved is not None else None,
                    "min_cruise_alt_m": round(self.min_cruise_alt, 3) if self.min_cruise_alt is not None else None,
                    "max_cruise_alt_m": round(self.max_cruise_alt, 3) if self.max_cruise_alt is not None else None,
                    "cruise_altitude_breached": self.cruise_altitude_breached,
                    "was_armed_seen": self.was_armed_seen,
                    "final_disarmed_confirmed": self.final_disarmed_confirmed
                },
                "log_count": len(self.received_statustexts)
            }

            try:
                with open(report_file, "w") as rf:
                    json.dump(report, rf, indent=2)
                print(f"[RUNNER] Doğrulama raporu kaydedildi -> {report_file}", flush=True)
            except Exception as re:
                print(f"[RUNNER] Rapor kaydetme hatası: {re}", flush=True)

        return self.test_passed

def main() -> bool:
    parser = argparse.ArgumentParser(description="S500 AP_Scripting SITL Comprehensive Runner")
    parser.add_argument("--scenario", type=str, choices=["a", "b", "failsafe", "test_failure"], required=True)
    parser.add_argument("--sitl-dir", type=str, default="/workspace/sitl_test_run")
    args = parser.parse_args()

    runner = SitlScenarioRunner(scenario=args.scenario, sitl_dir=args.sitl_dir)
    return runner.run()

if __name__ == "__main__":
    success = main()
    sys.exit(0 if success else 1)
