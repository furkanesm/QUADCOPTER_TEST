#!/usr/bin/env python3
"""
S500 AP_Scripting Görev Durum Makinesi Uçtan Uca Test Scripti (test_s500_mission.py)
-----------------------------------------------------------------------------------
Bu script, ArduPilot SITL / Cube Orange üzerinde çalışan s500_mission.lua durum
makinesini test etmek üzere sahte Jetson MAVLink mesajları (MAV_CMD_USER_1 / 31010) gönderir
ve otopilotun durum geçiş loglarını (STATUSTEXT) anlık olarak ekrana basar.

KULLANIM:
  python3 test_s500_mission.py --master tcp:127.0.0.1:5760 --scenario a
  python3 test_s500_mission.py --master tcp:127.0.0.1:5760 --scenario b
  python3 test_s500_mission.py --master tcp:127.0.0.1:5760 --scenario failsafe
"""

import argparse
import sys
import time
import threading
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

class S500MissionTester:
    def __init__(self, master_str: str, target_sys: int = 1, target_comp: int = 1):
        self.master_str = master_str
        self.target_sys = target_sys
        self.target_comp = target_comp
        self.running = True
        self.current_fsm_state = "BILINMIYOR"
        self.received_logs = []
        self.last_statustext = ""

        print(f"[TESTER] Bağlantı kuruluyor: {self.master_str} ...", flush=True)
        self.mav = mavutil.mavlink_connection(self.master_str)
        self.mav.wait_heartbeat(timeout=30)
        print(f"[TESTER] Otopilot tespit edildi (SysID={self.mav.target_system}, CompID={self.mav.target_component})", flush=True)

        # Arka planda telemetri ve STATUSTEXT dinleyici
        self.rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
        self.rx_thread.start()

    def _rx_loop(self):
        while self.running:
            try:
                msg = self.mav.recv_match(blocking=True, timeout=0.5)
                if not msg:
                    continue

                msg_type = msg.get_type()
                if msg_type == 'STATUSTEXT':
                    text = msg.text
                    if isinstance(text, bytes):
                        text = text.decode('utf-8', errors='ignore')
                    text = text.strip('\x00').strip()
                    if "[S500 LUA]" in text:
                        self.received_logs.append(text)
                        self.last_statustext = text
                        print(f"  >>> [STATUSTEXT] {text}", flush=True)
                        if "Durum Gecisi:" in text:
                            parts = text.split("->")
                            if len(parts) >= 2:
                                self.current_fsm_state = parts[1].strip().split()[0]
            except Exception as e:
                if self.running:
                    print(f"[TESTER] RX Hata: {e}", flush=True)

    def send_user_cmd(self, seq: int, status: int, x: float = 0.0, y: float = 0.0, conf: float = 1.0):
        name = STATUS_NAMES.get(status, f"BILINMEYEN({status})")
        print(f"[TESTER-TX] Gönderiliyor -> Seq: {seq}, Status: {status} ({name}), X: {x:.1f}m, Y: {y:.1f}m, Conf: {conf:.2f}", flush=True)
        self.mav.mav.command_long_send(
            self.target_sys,
            self.target_comp,
            MAV_CMD_USER_1,
            0, # confirmation
            float(x),
            float(y),
            float(conf),
            float(seq),
            float(status),
            0.0,
            0.0
        )

    def wait_for_log_containing(self, substring: str, timeout_s: float = 60.0) -> bool:
        t0 = time.time()
        print(f"[TESTER] Bekleniyor (En fazla {timeout_s}s): '{substring}' ...", flush=True)
        checked_idx = 0
        while time.time() - t0 < timeout_s:
            while checked_idx < len(self.received_logs):
                if substring in self.received_logs[checked_idx]:
                    print(f"[TESTER] Bulundu! -> '{self.received_logs[checked_idx]}'", flush=True)
                    return True
                checked_idx += 1
            time.sleep(0.2)
        print(f"[TESTER] Zaman aşımı! '{substring}' loglarda tespit edilemedi.", flush=True)
        return False

    def trigger_mission_guided(self):
        print("[TESTER] Otopilot GUIDED moda geçiriliyor...", flush=True)
        self.mav.set_mode('GUIDED')

    def run_scenario_a(self):
        """
        Senaryo A (Basit Yol):
        status: 0 (NO_DETECTION) -> 1 (START_FOUND) -> 4 (ROUTE_READY)
        Beklenen: KALKIŞ -> HEDEF_BEKLE -> doğrudan DÖNÜŞ -> İNİŞ
        """
        print("\n" + "="*70)
        print("  SENARYO A BAŞLATILIYOR (Basit Yol: Hiç GOTO olmadan ROUTE_READY)")
        print("="*70, flush=True)

        self.trigger_mission_guided()

        # 1. HEDEF_BEKLE durumuna ulaşılmasını bekle
        if not self.wait_for_log_containing("Durum Gecisi: KALKIS -> HEDEF_BEKLE", timeout_s=90.0):
            print("[TESTER] HATA: Drone HEDEF_BEKLE durumuna gelemedi!", flush=True)
            return False

        print("\n[TESTER] Drone HEDEF_BEKLE durumunda! Mesaj senaryosu gönderiliyor...", flush=True)
        time.sleep(2.0)

        # 2. status=0 (NO_DETECTION) gönder
        self.send_user_cmd(seq=1, status=STATUS_NO_DETECTION, conf=0.0)
        time.sleep(2.0)

        # 3. status=1 (START_FOUND) gönder
        self.send_user_cmd(seq=2, status=STATUS_START_FOUND, conf=0.92)
        time.sleep(2.0)

        # 4. status=4 (ROUTE_READY) gönder
        self.send_user_cmd(seq=3, status=STATUS_ROUTE_READY, conf=1.0)

        # 5. ROUTE_READY alindi ve doğrudan DÖNÜŞ durumuna geçişi doğrula
        ok_recv = self.wait_for_log_containing("ROUTE_READY alindi (Seq: 3). Dogrudan DONUS durumuna geciliyor.", timeout_s=10.0)
        if not ok_recv:
            print("[TESTER] KRİTİK HATA: ROUTE_READY alindi logu gelmedi!", flush=True)
            return False

        # 6. DÖNÜŞ -> İNİŞ -> TAMAMLANDI akışını izle
        self.wait_for_log_containing("Durum Gecisi: DONUS -> INIS", timeout_s=90.0)
        ok_finish = self.wait_for_log_containing("GÖREV TAMAMLANDI", timeout_s=60.0)
        print("\n[TESTER] Senaryo A Tamamlandı. Başarı Durumu:", "BAŞARILI (PASSED)" if ok_finish else "BAŞARISIZ (FAILED)")
        return ok_finish

    def run_scenario_b(self):
        """
        Senaryo B (Gözlem Konumlu Yol):
        status: 0 -> 1 -> 3 (x=10.0, y=5.0) -> [drone gider] -> 1 -> 4
        Beklenen: KALKIŞ -> HEDEF_BEKLE -> HEDEFE_GİT -> HEDEF_KONUMUNDA_BEKLE -> DÖNÜŞ -> İNİŞ
        """
        print("\n" + "="*70)
        print("  SENARYO B BAŞLATILIYOR (Gözlem Konumlu Yol: status=3 -> HEDEFE_GIT -> status=4)")
        print("="*70, flush=True)

        self.trigger_mission_guided()

        # 1. HEDEF_BEKLE durumuna ulaşılmasını bekle
        if not self.wait_for_log_containing("Durum Gecisi: KALKIS -> HEDEF_BEKLE", timeout_s=90.0):
            print("[TESTER] HATA: Drone HEDEF_BEKLE durumuna gelemedi!", flush=True)
            return False

        print("\n[TESTER] Drone HEDEF_BEKLE durumunda! Gözlem isteği iletiliyor...", flush=True)
        time.sleep(2.0)

        # 2. status=0 ve status=1 gönder
        self.send_user_cmd(seq=10, status=STATUS_NO_DETECTION, conf=0.0)
        time.sleep(1.5)
        self.send_user_cmd(seq=11, status=STATUS_START_FOUND, conf=0.88)
        time.sleep(1.5)

        # 3. status=3 (GOTO_OBSERVATION) gönder (x=10.0m, y=5.0m)
        target_x, target_y = 10.0, 5.0
        self.send_user_cmd(seq=12, status=STATUS_GOTO_OBSERVATION, x=target_x, y=target_y, conf=0.95)

        # 4. GOTO_OBSERVATION alindi logunu doğrula
        ok_goto = self.wait_for_log_containing(f"GOTO_OBSERVATION alindi (Seq: 12, Hedef: [{target_x:.1f}, {target_y:.1f}], Conf: 0.95)", timeout_s=10.0)
        if not ok_goto:
            print("[TESTER] KRİTİK HATA: GOTO_OBSERVATION alindi logu gelmedi!", flush=True)
            return False

        # 5. Drone'un hedef konuma varıp HEDEF_KONUMUNDA_BEKLE durumuna geçmesini bekle
        if not self.wait_for_log_containing("Durum Gecisi: HEDEFE_GIT -> HEDEF_KONUMUNDA_BEKLE", timeout_s=90.0):
            print("[TESTER] HATA: Drone hedef konumuna varıp bekleme durumuna gecemedi!", flush=True)
            return False

        print("\n[TESTER] Drone HEDEF_KONUMUNDA_BEKLE durumuna gecti! Rota hazir bildirimi iletiliyor...", flush=True)
        time.sleep(2.0)

        # 6. status=1 (bilgi) ve status=4 (ROUTE_READY) gönder
        self.send_user_cmd(seq=13, status=STATUS_START_FOUND, conf=0.90)
        time.sleep(1.5)
        self.send_user_cmd(seq=14, status=STATUS_ROUTE_READY, conf=1.0)

        # 7. ROUTE_READY alindi logunu doğrula
        ok_rr = self.wait_for_log_containing("ROUTE_READY alindi (Seq: 14). DONUS durumuna geciliyor.", timeout_s=10.0)
        if not ok_rr:
            print("[TESTER] KRİTİK HATA: ROUTE_READY alindi logu gelmedi!", flush=True)
            return False

        # 8. DÖNÜŞ -> İNİŞ -> TAMAMLANDI akışını izle
        self.wait_for_log_containing("Durum Gecisi: DONUS -> INIS", timeout_s=90.0)
        ok_finish = self.wait_for_log_containing("GÖREV TAMAMLANDI", timeout_s=60.0)
        print("\n[TESTER] Senaryo B Tamamlandı. Başarı Durumu:", "BAŞARILI (PASSED)" if ok_finish else "BAŞARISIZ (FAILED)")
        return ok_finish

    def run_scenario_failsafe(self):
        """
        Failsafe Testi:
        Hiç mesaj göndermeden HEDEF_BEKLE durumunda 90 saniye beklenir.
        Beklenen: 90s sonra otomatik DONUS durumuna gecilmesi.
        """
        print("\n" + "="*70)
        print("  FAILSAFE TESTİ BAŞLATILIYOR (90s Zaman Aşımı Doğrulaması)")
        print("="*70, flush=True)

        self.trigger_mission_guided()

        if not self.wait_for_log_containing("Durum Gecisi: KALKIS -> HEDEF_BEKLE", timeout_s=90.0):
            print("[TESTER] HATA: Drone HEDEF_BEKLE durumuna gelemedi!", flush=True)
            return False

        print("[TESTER] Drone HEDEF_BEKLE durumunda. Hiçbir mesaj gönderilmeyecek, 90s failsafe bekleniyor...", flush=True)
        ok_failsafe = self.wait_for_log_containing("HEDEF_BEKLE 90s zaman asimi! Ne status=3 ne status=4 alindi. Failsafe DONUS durumuna geciliyor.", timeout_s=105.0)
        
        if ok_failsafe:
            print("[TESTER] Failsafe başarıyla devreye girdi! Drone inişe yönlendiriliyor...", flush=True)
            self.wait_for_log_containing("Durum Gecisi: DONUS -> INIS", timeout_s=90.0)
            self.wait_for_log_containing("GÖREV TAMAMLANDI", timeout_s=60.0)
        
        print("\n[TESTER] Failsafe Testi Sonucu:", "BAŞARILI (PASSED)" if ok_failsafe else "BAŞARISIZ (FAILED)")
        return ok_failsafe

    def stop(self):
        self.running = False


def main():
    parser = argparse.ArgumentParser(description="S500 Lua Mission Pymavlink Test Runner")
    parser.add_argument("--master", type=str, default="tcp:127.0.0.1:5760", help="MAVLink bağlantı adresi (örn: tcp:127.0.0.1:5760 veya udp:127.0.0.1:14550)")
    parser.add_argument("--scenario", type=str, choices=["a", "b", "failsafe"], required=True, help="Test senaryosu: a (basit yol), b (gözlem konumlu), failsafe (90s zaman aşımı)")
    args = parser.parse_args()

    tester = S500MissionTester(master_str=args.master)
    try:
        if args.scenario == "a":
            res = tester.run_scenario_a()
        elif args.scenario == "b":
            res = tester.run_scenario_b()
        elif args.scenario == "failsafe":
            res = tester.run_scenario_failsafe()
        
        sys.exit(0 if res else 1)
    finally:
        tester.stop()


if __name__ == "__main__":
    main()
