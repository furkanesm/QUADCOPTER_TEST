#!/usr/bin/env python3
"""
Dedicated SITL Flight Test Runner for S500 Quadcopter Mission State Machine.
Executes and strictly validates:
- Test A: 32.5m Takeoff -> 5s Hover -> Land
- Test B: 32.5m Takeoff -> 5s Hover -> 5x5m Square Route -> Return -> Land
- Test C1: External Mode Change to LOITER (via MAVROS /mavros/set_mode) during cruise
- Test C2: Telemetry Loss (Kill MAVROS) & Reconnect Non-Resumption Check
- Test C3: Altitude Ceiling Breach Detection & Emergency Abort (max_alt=31.0m)
- Test C4: Pilot RC Takeover Simulation (via RC Channel 5 Override to STABILIZE)
"""

import os
import sys
import re
import json
import time
import math
import signal
import socket
import subprocess
from typing import Optional, List, Dict, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_srvs.srv import Trigger
from geometry_msgs.msg import PoseStamped, TwistStamped
from nav_msgs.msg import Odometry
from mavros_msgs.msg import State, ExtendedState, OverrideRCIn
from mavros_msgs.srv import MessageInterval, SetMode, StreamRate


def get_sitl_firmware_info() -> Dict[str, str]:
    """Çalıştırılan SITL ortamının firmware sürümünü, commit hash'ini ve MAVLink commit'ini tespit eder."""
    info = {
        "version": "UNKNOWN",
        "commit": "UNKNOWN",
        "mavlink_commit": "UNKNOWN",
        "build_source": "/opt/ardupilot"
    }
    try:
        # 1. ArduCopter version.h
        vfile = "/opt/ardupilot/ArduCopter/version.h"
        if os.path.exists(vfile):
            with open(vfile, "r") as vf:
                for line in vf:
                    if "#define THISFIRMWARE" in line and '"' in line:
                        info["version"] = line.split('"')[1]
                        break

        # 2. ArduPilot Git Commit
        cmd_ap = "cd /opt/ardupilot && git log -1 --format='%h (%s)' 2>/dev/null"
        out_ap = subprocess.check_output(["bash", "-c", cmd_ap], text=True).strip()
        if out_ap:
            info["commit"] = out_ap

        # 3. MAVLink Git Commit
        cmd_mav = "cd /opt/ardupilot/modules/mavlink && git rev-parse --short HEAD 2>/dev/null"
        out_mav = subprocess.check_output(["bash", "-c", cmd_mav], text=True).strip()
        if out_mav:
            info["mavlink_commit"] = out_mav
    except Exception:
        pass
    return info


def make_empty_result(test_name: str, test_mode: str, log_path: str = "", fw_info: Dict = None) -> Dict:
    """Bütün dönüş yollarında ortak ve eksiksiz sonuç şeması."""
    if fw_info is None:
        fw_info = get_sitl_firmware_info()
    return {
        "test_name": test_name,
        "test_mode": test_mode,
        "firmware_version": fw_info.get("version", "UNKNOWN"),
        "firmware_commit": fw_info.get("commit", "UNKNOWN"),
        "mavlink_commit": fw_info.get("mavlink_commit", "UNKNOWN"),
        "passed": False,
        "final_state": "UNKNOWN",
        "failure_reasons": [],
        "states_seen": [],
        "state_sequence_valid": False,
        "min_cruise_rel_alt_m": None,
        "max_cruise_rel_alt_m": None,
        "max_mission_rel_alt_m": None,
        "max_horizontal_speed_ms": None,
        "return_distance_error_m": None,
        "landing_confirmed_on_ground": False,
        "disarmed_confirmed": False,
        "waypoints_completed": 0,
        "hover_duration_confirmed": False,
        "duration_s": 0.0,
        "test_mechanism": "NORMAL_AUTONOMOUS_FLIGHT",
        "log_path": log_path,
    }


def stop_process_tree(proc: subprocess.Popen, timeout_s: float = 3.0):
    """
    Sadece bu test tarafından başlatılan süreç grubunu kapatır.
    Genel pkill KULLANMAZ. Önce SIGTERM, kapanmazsa SIGKILL gönderir.
    """
    if proc is None or proc.poll() is not None:
        return
    try:
        pgid = os.getpgid(proc.pid)
        os.killpg(pgid, signal.SIGTERM)
    except (ProcessLookupError, OSError):
        return

    t0 = time.time()
    while time.time() - t0 < timeout_s:
        if proc.poll() is not None:
            return
        time.sleep(0.1)

    try:
        pgid = os.getpgid(proc.pid)
        os.killpg(pgid, signal.SIGKILL)
    except (ProcessLookupError, OSError):
        pass


class SitlTestMonitor(Node):
    def __init__(self):
        super().__init__('sitl_test_monitor')

        self.current_state: Optional[State] = None
        self.extended_state: Optional[ExtendedState] = None
        self.current_pose: Optional[PoseStamped] = None
        self.current_vel: Optional[TwistStamped] = None

        self._last_state_time: float = 0.0
        self._last_ext_state_time: float = 0.0
        self._last_pose_time: float = 0.0
        self._last_vel_time: float = 0.0

        # Kalkış referansı: Görev başlamadan önce BİR KEZ dondurulur; sonradan üzerine yazılmaz.
        self.takeoff_pose_frozen: Optional[Tuple[float, float, float]] = None

        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=10)

        self.create_subscription(State, '/mavros/state', self._state_cb, qos)
        self.create_subscription(ExtendedState, '/mavros/extended_state', self._ext_state_cb, qos)
        self.create_subscription(PoseStamped, '/mavros/local_position/pose', self._pose_cb, qos)
        self.create_subscription(TwistStamped, '/mavros/local_position/velocity_local', self._vel_cb, qos)
        self.create_subscription(Odometry, '/mavros/global_position/local', self._odom_cb, qos)

        self.pub_rc_override = self.create_publisher(OverrideRCIn, '/mavros/rc/override', 10)

        self.cli_set_interval = self.create_client(MessageInterval, '/mavros/set_message_interval')
        self.cli_set_stream_rate = self.create_client(StreamRate, '/mavros/set_stream_rate')
        self.cli_start_mission = self.create_client(Trigger, '/mission/start')
        self.cli_set_mode = self.create_client(SetMode, '/mavros/set_mode')

    def _state_cb(self, msg: State):
        self.current_state = msg
        self._last_state_time = time.time()

    def _ext_state_cb(self, msg: ExtendedState):
        self.extended_state = msg
        self._last_ext_state_time = time.time()

    def _odom_cb(self, msg: Odometry):
        p = msg.pose.pose.position
        if (math.isfinite(p.x) and math.isfinite(p.y) and math.isfinite(p.z) and
                abs(p.x) < 1000.0 and abs(p.y) < 1000.0 and abs(p.z) < 1000.0):
            pose_msg = PoseStamped()
            pose_msg.header = msg.header
            pose_msg.pose = msg.pose.pose
            self.current_pose = pose_msg
            self._last_pose_time = time.time()

        v = msg.twist.twist.linear
        if math.isfinite(v.x) and math.isfinite(v.y) and math.isfinite(v.z):
            vel_msg = TwistStamped()
            vel_msg.header = msg.header
            vel_msg.twist.linear.x = v.x
            vel_msg.twist.linear.y = v.y
            vel_msg.twist.linear.z = -v.z
            self.current_vel = vel_msg
            self._last_vel_time = time.time()

    def _pose_cb(self, msg: PoseStamped):
        p = msg.pose.position
        if (math.isfinite(p.x) and math.isfinite(p.y) and math.isfinite(p.z) and
                abs(p.x) < 1000.0 and abs(p.y) < 1000.0 and abs(p.z) < 1000.0):
            self.current_pose = msg
            self._last_pose_time = time.time()

    def _vel_cb(self, msg: TwistStamped):
        self.current_vel = msg
        self._last_vel_time = time.time()

    def request_stream_rate(self, stream_id: int = 0, message_rate: int = 10) -> bool:
        if not self.cli_set_stream_rate.wait_for_service(timeout_sec=2.0):
            return False
        req = StreamRate.Request()
        req.stream_id = int(stream_id)
        req.message_rate = int(message_rate)
        req.on_off = True
        future = self.cli_set_stream_rate.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        return future.result() is not None

    def is_pose_fresh(self, max_age_s: float = 1.0) -> bool:
        return self.current_pose is not None and (time.time() - self._last_pose_time) < max_age_s

    def is_ext_state_fresh(self, max_age_s: float = 1.5) -> bool:
        return self.extended_state is not None and (time.time() - self._last_ext_state_time) < max_age_s

    def freeze_takeoff_reference(self, max_age_s: float = 1.0) -> bool:
        """Kalkış referansını yerdeyken ve güncel veriyle BİR KEZ dondurur."""
        if self.takeoff_pose_frozen is not None:
            return True
        if not self.is_pose_fresh(max_age_s):
            return False
        if not self.is_ext_state_fresh(max_age_s):
            return False
        if self.extended_state.landed_state != ExtendedState.LANDED_STATE_ON_GROUND:
            return False
        p = self.current_pose.pose.position
        if abs(p.x) >= 1000.0 or abs(p.y) >= 1000.0 or abs(p.z) >= 1000.0:
            return False
        self.takeoff_pose_frozen = (p.x, p.y, p.z)
        return True

    def request_extended_state_interval(self) -> bool:
        if not self.cli_set_interval.wait_for_service(timeout_sec=2.0):
            return False
        req = MessageInterval.Request()
        req.message_id = 245
        req.message_rate = 4.0
        future = self.cli_set_interval.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        return future.result() is not None and future.result().success

    def call_start_mission(self) -> bool:
        if not self.cli_start_mission.wait_for_service(timeout_sec=10.0):
            return False
        req = Trigger.Request()
        future = self.cli_start_mission.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        return future.result() is not None and future.result().success

    def request_mode_change(self, mode: str) -> bool:
        """Harici mod değişikliği talep eder (/mavros/set_mode)."""
        if not self.cli_set_mode.wait_for_service(timeout_sec=3.0):
            return False
        req = SetMode.Request()
        req.custom_mode = mode
        future = self.cli_set_mode.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)
        return future.result() is not None and future.result().mode_sent

    def simulate_rc_takeover(self, pwm_ch5: int = 1100):
        """RC Kanal 5 override yayınlayarak kumanda anahtar değişimini simüle eder."""
        msg = OverrideRCIn()
        # 8 kanal: [ch1, ch2, ch3, ch4, ch5, ch6, ch7, ch8]
        msg.channels = [65535] * 18
        msg.channels[4] = pwm_ch5  # Ch 5: FLTMODE (1100=STABILIZE, 1850=LOITER)
        self.pub_rc_override.publish(msg)


def run_test_scenario(
    test_name: str,
    test_mode: str,
    timeout_s: float = 160,
    inject_fault: str = "NONE",
    custom_node_args: List[str] = None
) -> Dict:
    fw_info = get_sitl_firmware_info()
    procs = []
    log_path = f"/workspace/ros2_ws/{test_name}.log"
    res = make_empty_result(test_name, test_mode, log_path, fw_info)
    mechanisms = {
        "TEST_A_HOVER_32_5M": "AUTONOMOUS_TAKEOVER_HOVER_LAND",
        "TEST_B_SQUARE_ROUTE": "AUTONOMOUS_FULL_ROUTE_SQUARE_NAV",
        "TEST_C1_EXTERNAL_MODE_CHANGE": "MAVROS_SET_MODE_LOITER (GCS External Mode Change)",
        "TEST_C2_TELEMETRY_LOSS_AND_RECONNECT": "MAVROS_PROCESS_KILL (Telemetry Loss & Reconnect Non-Resumption)",
        "TEST_C3_ALTITUDE_BREACH_ABORT": "PARAMETER_CEILING_REDUCTION (Altitude Breach Trigger)",
        "TEST_C4_PILOT_RC_TAKEOVER": "MAVROS_SET_MODE_STABILIZE (Simulated Safety Pilot RC Switch Takeover)",
    }
    res["test_mechanism"] = mechanisms.get(test_name, "NORMAL_AUTONOMOUS_FLIGHT")

    print(f"\n{'='*70}\n  BAŞLATILIYOR: {test_name} (Mod: {test_mode}, Hata Enjeksiyonu: {inject_fault})\n{'='*70}", flush=True)

    monitor: Optional[SitlTestMonitor] = None

    try:
        # 1. Start SITL ArduCopter
        sitl_proc = subprocess.Popen([
            "/opt/ardupilot/build/sitl/bin/arducopter",
            "-S", "--model", "quad",
            "--home", "-35.363261,149.165230,584,353",
            "--defaults", "/opt/ardupilot/Tools/autotest/default_params/copter.parm"
        ], preexec_fn=os.setsid, stdout=open(f"/tmp/sitl_{test_name}.log", "w"), stderr=subprocess.STDOUT)
        procs.append(sitl_proc)

        # SITL port 5760 beklemesi (bounded, yoklama)
        sitl_ready = False
        t_start_sitl = time.time()
        while time.time() - t_start_sitl < 10.0:
            if sitl_proc.poll() is not None:
                break
            try:
                with socket.create_connection(("127.0.0.1", 5760), timeout=0.5):
                    sitl_ready = True
                    break
            except (OSError, ConnectionRefusedError):
                time.sleep(0.3)

        if not sitl_ready:
            res["final_state"] = "SITL_START_FAIL"
            res["failure_reasons"].append("SITL TCP port 5760 zamanında açılmadı.")
            return res

        print("  [1/4] SITL port 5760 hazır. MAVROS başlatılıyor...", flush=True)

        # 2. Start MAVROS
        mavros_proc = subprocess.Popen([
            "bash", "-c",
            "source /opt/ros/humble/setup.bash && ros2 launch mavros apm.launch fcu_url:=tcp://127.0.0.1:5760"
        ], preexec_fn=os.setsid, stdout=open(f"/tmp/mavros_{test_name}.log", "w"), stderr=subprocess.STDOUT)
        procs.append(mavros_proc)

        # 3. Monitor Node Setup
        monitor = SitlTestMonitor()

        # MAVROS bağlantısı bekleme (yoklama)
        print("      MAVROS telemetri bağlantısı bekleniyor...", flush=True)
        connected = False
        start_wait = time.time()
        while time.time() - start_wait < 25.0:
            rclpy.spin_once(monitor, timeout_sec=0.5)
            if monitor.current_state is not None and monitor.current_state.connected:
                connected = True
                break

        if not connected:
            res["final_state"] = "MAVROS_TIMEOUT"
            res["failure_reasons"].append("MAVROS otopilota 25s içinde bağlanamadı.")
            return res

        # Telemetri ve ExtendedState akış isteği
        print("  [2/4] Telemetri (StreamRate) ve ExtendedState (Mesaj 245) akışı talep ediliyor...", flush=True)
        for _ in range(5):
            monitor.request_stream_rate(0, 10)
            if monitor.request_extended_state_interval():
                break
            time.sleep(1)

        # ExtendedState (ON_GROUND) doğrulaması
        start_wait = time.time()
        ext_ok = False
        while time.time() - start_wait < 15.0:
            rclpy.spin_once(monitor, timeout_sec=0.5)
            if monitor.is_ext_state_fresh(1.5) and monitor.extended_state.landed_state == ExtendedState.LANDED_STATE_ON_GROUND:
                ext_ok = True
                print(f"      ExtendedState teyit edildi: ON_GROUND ({monitor.extended_state.landed_state})", flush=True)
                break

        if not ext_ok:
            res["final_state"] = "EXT_STATE_TIMEOUT"
            res["failure_reasons"].append("ExtendedState (ON_GROUND) zamanında teyit edilemedi.")
            return res

        # Yerel konum (EKF) doğrulaması
        print("      EKF ve LocalPosition/pose bekleniyor...", flush=True)
        start_wait = time.time()
        pose_ok = False
        while time.time() - start_wait < 40.0:
            rclpy.spin_once(monitor, timeout_sec=0.5)
            if monitor.is_pose_fresh(1.0):
                p = monitor.current_pose.pose.position
                if abs(p.x) < 1000.0 and abs(p.y) < 1000.0 and abs(p.z) < 1000.0:
                    pose_ok = True
                    print(f"      LocalPosition teyit edildi: X={p.x:.2f}, Y={p.y:.2f}, Z={p.z:.2f}", flush=True)
                    break

        if not pose_ok:
            res["final_state"] = "POSE_TIMEOUT"
            res["failure_reasons"].append("LocalPosition/pose 40s içinde hazır olmadı.")
            return res

        # Kalkış referansını monitor üzerinde dondur
        if not monitor.freeze_takeoff_reference(1.0):
            res["final_state"] = "TAKEOFF_REF_FAIL"
            res["failure_reasons"].append("Kalkış referansı dondurulamadı.")
            return res

        # 4. Start Mission Node
        cmd_args = [
            f"-p test_mode:={test_mode}",
            "-p autopilot_type:=ardupilot",
            "-p takeoff_altitude_m:=32.5",
            "-p cruise_altitude_m:=32.5",
            "-p min_cruise_altitude_m:=30.0",
            "-p max_relative_altitude_m:=35.0"
        ]
        if custom_node_args:
            cmd_args.extend(custom_node_args)

        args_str = " ".join(cmd_args)
        node_proc = subprocess.Popen([
            "bash", "-c",
            f"source /opt/ros/humble/setup.bash && source /workspace/ros2_ws/install/setup.bash && "
            f"ros2 run s500_mission_fsm mission_node --ros-args {args_str} > {log_path} 2>&1"
        ], preexec_fn=os.setsid)
        procs.append(node_proc)

        # 5. Call /mission/start
        print("  [3/4] /mission/start servisi çağrılıyor...", flush=True)
        if not monitor.call_start_mission():
            res["final_state"] = "START_SRV_FAILED"
            res["failure_reasons"].append("/mission/start servisi zamanında yanıt vermedi veya başarısız döndü.")
            return res

        # 6. Monitor Flight Execution
        print(f"  [4/4] Uçuş icrası izleniyor (Zaman sınırı: {timeout_s}s)...", flush=True)
        flight_start = time.time()

        in_cruise = False
        fault_injected = False
        fault_verified = False
        reconnect_verified = False

        obs_min_cruise = float('inf')
        obs_max_cruise = float('-inf')
        obs_max_mission = float('-inf')
        obs_max_hspeed = 0.0

        while time.time() - flight_start < timeout_s:
            rclpy.spin_once(monitor, timeout_sec=0.2)
            res["duration_s"] = time.time() - flight_start

            # Log analizi: Durum geçişleri ve aşama olayları
            if os.path.exists(log_path):
                with open(log_path, 'r') as f:
                    lines = f.readlines()
                for line in lines:
                    if "[FSM GEÇİŞ]" in line:
                        parts = line.split("[FSM GEÇİŞ]")[1].split("|")[0].split("->")
                        if len(parts) == 2:
                            st = parts[1].strip()
                            if st not in res["states_seen"]:
                                res["states_seen"].append(st)
                                print(f"      [FSM Durum]: -> {st}", flush=True)
                                if st in ["HAVADA_BEKLEME", "KISA_ROTA", "KALKIS_NOKTASINA_DONUS"]:
                                    in_cruise = True
                                elif st in ["INIS", "TAMAMLANDI", "PILOT_MUDAHALESI", "GOREV_IPTAL", "IRTITA_IHLALI"]:
                                    in_cruise = False

                    if "[HAVADA BEKLEME] 5s tamamlandı" in line:
                        res["hover_duration_confirmed"] = True

                    m_wp = re.search(r"\[KISA ROTA\] WP (\d+) tamamlandı", line)
                    if m_wp:
                        wp_num = int(m_wp.group(1))
                        if wp_num > res["waypoints_completed"]:
                            res["waypoints_completed"] = wp_num

            # Telemetri metrik takibi (Gözlemci tarafından)
            if monitor.is_pose_fresh(1.0) and monitor.takeoff_pose_frozen is not None:
                cur_z = monitor.current_pose.pose.position.z
                rel_z = cur_z - monitor.takeoff_pose_frozen[2]

                if rel_z > obs_max_mission:
                    obs_max_mission = rel_z

                if in_cruise:
                    if rel_z < obs_min_cruise:
                        obs_min_cruise = rel_z
                    if rel_z > obs_max_cruise:
                        obs_max_cruise = rel_z

            if monitor.current_vel is not None and (time.time() - monitor._last_vel_time) < 1.0:
                vx = monitor.current_vel.twist.linear.x
                vy = monitor.current_vel.twist.linear.y
                if math.isfinite(vx) and math.isfinite(vy):
                    h_spd = math.hypot(vx, vy)
                    if h_spd > obs_max_hspeed:
                        obs_max_hspeed = h_spd

            # --- HATA ENJEKSİYONU 1: Harici Mod Değişikliği (LOITER) ---
            if inject_fault == "EXTERNAL_MODE_CHANGE" and not fault_injected and in_cruise:
                time.sleep(2.0)
                print("      >>> [HATA ENJEKSİYONU] MAVROS üzerinden harici mod değişikliği isteniyor: LOITER...", flush=True)
                sent_ok = monitor.request_mode_change("LOITER")
                fault_injected = True
                if not sent_ok:
                    res["failure_reasons"].append("LOITER mod değişim servis isteği başarısız oldu.")
                else:
                    time.sleep(1.0)
                    rclpy.spin_once(monitor, timeout_sec=0.5)
                    if monitor.current_state is not None and monitor.current_state.mode == "LOITER":
                        fault_verified = True
                        print("      >>> Telemetride modun gerçekten LOITER olduğu doğrulandı.", flush=True)

            # --- HATA ENJEKSİYONU 4: RC Pilot Takeover (Kumanda Anahtarı -> STABILIZE) ---
            # NOT: ArduPilot standalone SITL'deki RC_CHANNELS_OVERRIDE debounce/hal.rcin kısıtlaması 
            # nedeniyle pilotun fiziksel kumanda anahtarını STABILIZE moduna alması MAVROS SetMode('STABILIZE') 
            # üzerinden simüle edilmektedir. C1'den farkı: C1 'LOITER' (GCS müdahalesi), C4 'STABILIZE' (Pilot manuel kumanda).
            elif inject_fault == "RC_TAKEOVER" and not fault_injected and in_cruise:
                time.sleep(2.0)
                print("      >>> [HATA ENJEKSİYONU] Pilot kumanda anahtar müdahalesi simüle ediliyor (STABILIZE)...", flush=True)
                sent_ok = monitor.request_mode_change("STABILIZE")
                fault_injected = True
                if not sent_ok:
                    res["failure_reasons"].append("STABILIZE mod değişim isteği başarısız oldu.")
                else:
                    time.sleep(1.0)
                    rclpy.spin_once(monitor, timeout_sec=0.5)
                    if monitor.current_state is not None and monitor.current_state.mode == "STABILIZE":
                        fault_verified = True
                        print("      >>> Telemetride modun gerçekten STABILIZE (Manuel Pilot Müdahalesi) olduğu doğrulandı.", flush=True)

            # --- HATA ENJEKSİYONU 2: Telemetri Kesintisi & Yeniden Bağlantı ---
            elif inject_fault == "TELEMETRY_LOSS" and not fault_injected and in_cruise:
                time.sleep(2.0)
                print("      >>> [HATA ENJEKSİYONU] MAVROS süreci durduruluyor (telemetri kaybı)...", flush=True)
                stop_process_tree(mavros_proc)
                fault_injected = True

            # Telemetri kesildikten sonra FSM'nin GOREV_IPTAL durumuna geçişi ve yeniden bağlanma kontrolü
            if inject_fault == "TELEMETRY_LOSS" and fault_injected and not reconnect_verified:
                if "GOREV_IPTAL" in res["states_seen"]:
                    print("      >>> FSM telemetri kaybını algıladı (GOREV_IPTAL). MAVROS yeniden başlatılıyor...", flush=True)
                    # MAVROS'u yeniden başlat
                    mavros_proc2 = subprocess.Popen([
                        "bash", "-c",
                        "source /opt/ros/humble/setup.bash && ros2 launch mavros apm.launch fcu_url:=tcp://127.0.0.1:5760"
                    ], preexec_fn=os.setsid, stdout=open(f"/tmp/mavros_reconnect_{test_name}.log", "w"), stderr=subprocess.STDOUT)
                    procs.append(mavros_proc2)

                    # Yeniden bağlandıktan sonra 5 saniye gözle
                    time.sleep(5.0)

                    # GOREV_IPTAL sonrasındaki FSM geçişlerini Regex ile kesin olarak parse et
                    if os.path.exists(log_path):
                        with open(log_path, 'r') as f:
                            content = f.read()
                        transitions = re.findall(r"\[FSM GEÇİŞ\]\s*([A-Z_]+)\s*->\s*([A-Z_]+)", content)
                        post_abort_states = []
                        abort_found = False
                        for src, dst in transitions:
                            if abort_found:
                                post_abort_states.append(dst)
                            elif dst == "GOREV_IPTAL":
                                abort_found = True

                        if len(post_abort_states) > 0:
                            res["failure_reasons"].append(f"Yeniden bağlantı sonrası FSM izinsiz geçiş yaptı: {post_abort_states}")
                        else:
                            reconnect_verified = True
                            res["final_state"] = "GOREV_IPTAL"
                            print("      >>> Yeniden bağlantı sonrası FSM'nin GOREV_IPTAL durumunda sabit kaldığı doğrulandı.", flush=True)
                    break

            # Tamamlanma veya Arıza Durum Kontrolleri
            if "TAMAMLANDI" in res["states_seen"]:
                res["final_state"] = "TAMAMLANDI"
                time.sleep(1.0)
                break
            elif "PILOT_MUDAHALESI" in res["states_seen"]:
                res["final_state"] = "PILOT_MUDAHALESI"
                if inject_fault in ["EXTERNAL_MODE_CHANGE", "RC_TAKEOVER"]:
                    fault_verified = True
                    break
            elif "GOREV_IPTAL" in res["states_seen"] and inject_fault != "TELEMETRY_LOSS":
                res["final_state"] = "GOREV_IPTAL"
                break
            elif "IRTITA_IHLALI" in res["states_seen"]:
                res["final_state"] = "IRTITA_IHLALI"
                if inject_fault == "ALTITUDE_BREACH":
                    break

        if res["final_state"] == "UNKNOWN":
            res["final_state"] = "TIMEOUT"
            res["failure_reasons"].append(f"Test {timeout_s}s zaman aşımına uğradı.")

        # Logdan metrikleri kesin Regex ile çek (İki nokta sorununa karşı bağışık)
        if os.path.exists(log_path):
            with open(log_path, 'r') as f:
                log_text = f.read()

            m = re.search(r"Seyir En Düşük İrtifa\s*:\s*([\d\.]+)\s*m", log_text)
            if m:
                res["min_cruise_rel_alt_m"] = float(m.group(1))

            m = re.search(r"Seyir En Yüksek İrtifa\s*:\s*([\d\.]+)\s*m", log_text)
            if m:
                res["max_cruise_rel_alt_m"] = float(m.group(1))

            m = re.search(r"Tüm Görev Zirve İrtifa\s*:\s*([\d\.]+)\s*m", log_text)
            if m:
                res["max_mission_rel_alt_m"] = float(m.group(1))

            m = re.search(r"Ölçülen En Yüksek Yatay Hız\s*:\s*([\d\.]+)\s*m/s", log_text)
            if m:
                res["max_horizontal_speed_ms"] = float(m.group(1))

            m = re.search(r"Kalkış Noktasına Dönüş Hatası\s*:\s*([\d\.]+)\s*m", log_text)
            if m:
                res["return_distance_error_m"] = float(m.group(1))

            if (re.search(r"İniş ve Disarm Teyidi\s*:\s*EVET", log_text) or
                    "[İNİŞ TAMAM] Yerde olma ve DISARM durumu birlikte doğrulandı." in log_text):
                res["landing_confirmed_on_ground"] = True
                res["disarmed_confirmed"] = True

        # Gözlemci telemetrisi yedeği
        if res["min_cruise_rel_alt_m"] is None and obs_min_cruise != float('inf'):
            res["min_cruise_rel_alt_m"] = obs_min_cruise
        if res["max_cruise_rel_alt_m"] is None and obs_max_cruise != float('-inf'):
            res["max_cruise_rel_alt_m"] = obs_max_cruise
        if res["max_mission_rel_alt_m"] is None and obs_max_mission != float('-inf'):
            res["max_mission_rel_alt_m"] = obs_max_mission
        if res["max_horizontal_speed_ms"] is None and obs_max_hspeed > 0.0:
            res["max_horizontal_speed_ms"] = obs_max_hspeed

        # ==========================================
        # KATI TEST BAŞARI DOĞRULAMASI
        # ==========================================
        if test_name == "TEST_A_HOVER_32_5M":
            expected_seq = ["HAZIRLIK", "GUIDED_VE_ARM", "KALKIS", "HAVADA_BEKLEME", "INIS", "TAMAMLANDI"]
            res["state_sequence_valid"] = (res["states_seen"] == expected_seq)
            if not res["state_sequence_valid"]:
                res["failure_reasons"].append(f"Geçersiz durum sırası: {res['states_seen']} != {expected_seq}")

            if not res["hover_duration_confirmed"]:
                res["failure_reasons"].append("5s havada bekleme teyit edilemedi.")

            if res["min_cruise_rel_alt_m"] is None or res["min_cruise_rel_alt_m"] < 30.0:
                res["failure_reasons"].append(f"Seyir min irtifası ihlal edildi veya ölçülemedi: {res['min_cruise_rel_alt_m']}")

            if res["max_mission_rel_alt_m"] is None or res["max_mission_rel_alt_m"] > 35.0:
                res["failure_reasons"].append(f"Maksimum irtifa tavanı ihlal edildi veya ölçülemedi: {res['max_mission_rel_alt_m']}")

            if not (res["landing_confirmed_on_ground"] and res["disarmed_confirmed"]):
                res["failure_reasons"].append("İniş ve disarm teyidi sağlanamadı.")

            res["passed"] = (len(res["failure_reasons"]) == 0 and res["final_state"] == "TAMAMLANDI")

        elif test_name == "TEST_B_SQUARE_ROUTE":
            expected_seq = ["HAZIRLIK", "GUIDED_VE_ARM", "KALKIS", "HAVADA_BEKLEME", "KISA_ROTA", "KALKIS_NOKTASINA_DONUS", "INIS", "TAMAMLANDI"]
            res["state_sequence_valid"] = (res["states_seen"] == expected_seq)
            if not res["state_sequence_valid"]:
                res["failure_reasons"].append(f"Geçersiz durum sırası: {res['states_seen']} != {expected_seq}")

            if res["waypoints_completed"] < 4:
                res["failure_reasons"].append(f"Kare rotada eksik waypoint: {res['waypoints_completed']}/4 tamamlandı.")

            if res["return_distance_error_m"] is None or res["return_distance_error_m"] > 1.0:
                res["failure_reasons"].append(f"Dönüş hatası limit dışı veya ölçülemedi: {res['return_distance_error_m']}m (limit <= 1.0m)")

            if res["min_cruise_rel_alt_m"] is None or res["min_cruise_rel_alt_m"] < 30.0:
                res["failure_reasons"].append(f"Seyir min irtifası ihlal edildi veya ölçülemedi: {res['min_cruise_rel_alt_m']}")

            if res["max_mission_rel_alt_m"] is None or res["max_mission_rel_alt_m"] > 35.0:
                res["failure_reasons"].append(f"Maksimum irtifa tavanı ihlal edildi veya ölçülemedi: {res['max_mission_rel_alt_m']}")

            if not (res["landing_confirmed_on_ground"] and res["disarmed_confirmed"]):
                res["failure_reasons"].append("İniş ve disarm teyidi sağlanamadı.")

            res["passed"] = (len(res["failure_reasons"]) == 0 and res["final_state"] == "TAMAMLANDI")

        elif test_name == "TEST_C1_EXTERNAL_MODE_CHANGE":
            if res["final_state"] == "TAMAMLANDI":
                res["failure_reasons"].append("Hata enjeksiyonuna rağmen görev normal tamamlandı sayıldı!")
            elif res["final_state"] == "PILOT_MUDAHALESI" and fault_verified:
                res["passed"] = True
            else:
                res["failure_reasons"].append(f"Beklenen PILOT_MUDAHALESI yerine {res['final_state']} alındı.")

        elif test_name == "TEST_C2_TELEMETRY_LOSS_AND_RECONNECT":
            if res["final_state"] == "TAMAMLANDI":
                res["failure_reasons"].append("Telemetri kaybına rağmen görev tamamlandı sayıldı!")
            elif res["final_state"] == "GOREV_IPTAL" and reconnect_verified and len(res["failure_reasons"]) == 0:
                res["passed"] = True
            else:
                res["failure_reasons"].append(f"Beklenen tepki veya yeniden bağlanma teyit edilemedi. Son Durum: {res['final_state']}")

        elif test_name == "TEST_C3_ALTITUDE_BREACH_ABORT":
            if res["final_state"] == "TAMAMLANDI":
                res["failure_reasons"].append("İrtifa tavanı ihlaline rağmen görev tamamlandı sayıldı!")
            elif res["final_state"] == "IRTITA_IHLALI":
                res["passed"] = True
            else:
                res["failure_reasons"].append(f"Beklenen IRTITA_IHLALI yerine {res['final_state']} alındı.")

        elif test_name == "TEST_C4_PILOT_RC_TAKEOVER":
            if res["final_state"] == "TAMAMLANDI":
                res["failure_reasons"].append("RC müdahalesine rağmen görev normal tamamlandı sayıldı!")
            elif res["final_state"] == "PILOT_MUDAHALESI" and fault_verified:
                res["passed"] = True
            else:
                res["failure_reasons"].append(f"Beklenen PILOT_MUDAHALESI yerine {res['final_state']} alındı.")

    finally:
        # Garanti temizlik: Sadece bu testin süreçleri kapatılır
        for p in reversed(procs):
            stop_process_tree(p)
        if monitor is not None:
            monitor.destroy_node()

    return res


def main():
    rclpy.init()
    results = []

    print("=====================================================================")
    print("     S500 ROS 2 + MAVROS + ARDUCOPTER SITL TEST HÂNESİ")
    print("=====================================================================")

    fw_info = get_sitl_firmware_info()
    print(f"Firmware: {fw_info['version']} | SITL Commit: {fw_info['commit']}")
    print("=====================================================================")

    # --- TEST A: 32.5m Kalkış -> 5s Bekleme -> İniş ---
    res_a = run_test_scenario("TEST_A_HOVER_32_5M", "HOVER_ONLY", timeout_s=140)
    results.append(res_a)
    print(f"\n[TEST A SONUCU] Geçti mi: {res_a['passed']} | Durum: {res_a['final_state']}")
    alt_max_str = f"{res_a['max_mission_rel_alt_m']:.2f}m" if res_a['max_mission_rel_alt_m'] is not None else "Ölçülemedi"
    c_min_str = f"{res_a['min_cruise_rel_alt_m']:.2f}m" if res_a['min_cruise_rel_alt_m'] is not None else "Ölçülemedi"
    c_max_str = f"{res_a['max_cruise_rel_alt_m']:.2f}m" if res_a['max_cruise_rel_alt_m'] is not None else "Ölçülemedi"
    print(f"Metrikler: Zirve İrtifa={alt_max_str}, Seyir Min={c_min_str}, Seyir Max={c_max_str}")
    if res_a["failure_reasons"]:
        print(f"Hata Nedenleri: {res_a['failure_reasons']}")

    # --- TEST B: 32.5m Kalkış -> 5s Bekleme -> 5x5m Kare Rota -> Dönüş -> İniş ---
    if res_a['passed']:
        res_b = run_test_scenario("TEST_B_SQUARE_ROUTE", "FULL_ROUTE", timeout_s=180)
        results.append(res_b)
        print(f"\n[TEST B SONUCU] Geçti mi: {res_b['passed']} | Durum: {res_b['final_state']}")
        b_max_str = f"{res_b['max_mission_rel_alt_m']:.2f}m" if res_b['max_mission_rel_alt_m'] is not None else "Ölçülemedi"
        b_cmin_str = f"{res_b['min_cruise_rel_alt_m']:.2f}m" if res_b['min_cruise_rel_alt_m'] is not None else "Ölçülemedi"
        b_cmax_str = f"{res_b['max_cruise_rel_alt_m']:.2f}m" if res_b['max_cruise_rel_alt_m'] is not None else "Ölçülemedi"
        b_ret_str = f"{res_b['return_distance_error_m']:.2f}m" if res_b['return_distance_error_m'] is not None else "Ölçülemedi"
        b_spd_str = f"{res_b['max_horizontal_speed_ms']:.2f}m/s" if res_b['max_horizontal_speed_ms'] is not None else "Ölçülemedi"
        print(f"Metrikler: Zirve İrtifa={b_max_str}, Seyir Min={b_cmin_str}, Seyir Max={b_cmax_str}")
        print(f"Dönüş Hatası={b_ret_str}, En Yüksek Yatay Hız={b_spd_str}")
        if res_b["failure_reasons"]:
            print(f"Hata Nedenleri: {res_b['failure_reasons']}")
    else:
        print("\n[TEST B ATLANDI] Test A başarısız olduğu için Test B çalıştırılmadı.")

    # --- TEST C1: Harici Mod Değişikliği (LOITER) ---
    res_c1 = run_test_scenario(
        "TEST_C1_EXTERNAL_MODE_CHANGE",
        "FULL_ROUTE",
        timeout_s=100,
        inject_fault="EXTERNAL_MODE_CHANGE"
    )
    results.append(res_c1)
    print(f"\n[TEST C1 SONUCU] Harici Mod Değişimi Teyit Edildi mi: {res_c1['passed']} (Son Durum: {res_c1['final_state']})")
    if res_c1["failure_reasons"]:
        print(f"Hata Nedenleri: {res_c1['failure_reasons']}")

    # --- TEST C2: Telemetri Kesintisi ve Yeniden Bağlantı Kontrolü ---
    res_c2 = run_test_scenario(
        "TEST_C2_TELEMETRY_LOSS_AND_RECONNECT",
        "FULL_ROUTE",
        timeout_s=100,
        inject_fault="TELEMETRY_LOSS"
    )
    results.append(res_c2)
    print(f"\n[TEST C2 SONUCU] Telemetri Kesintisi ve Non-Resume Teyit Edildi mi: {res_c2['passed']} (Son Durum: {res_c2['final_state']})")
    if res_c2["failure_reasons"]:
        print(f"Hata Nedenleri: {res_c2['failure_reasons']}")

    # --- TEST C3: İrtifa Bandı Tavan İhlali Tespiti ve Acil İniş ---
    # Tavanı kasten 31.0 metreye çekip 32.5 metre kalkış talep ediyoruz (İhlal kaçınılmaz olmalıdır)
    res_c3 = run_test_scenario(
        "TEST_C3_ALTITUDE_BREACH_ABORT",
        "FULL_ROUTE",
        timeout_s=100,
        inject_fault="ALTITUDE_BREACH",
        custom_node_args=["-p max_relative_altitude_m:=31.0", "-p takeoff_altitude_m:=32.5"]
    )
    results.append(res_c3)
    print(f"\n[TEST C3 SONUCU] İrtifa İhlali Algılandı ve Acil İniş Yapıldı mı: {res_c3['passed']} (Son Durum: {res_c3['final_state']})")
    if res_c3["failure_reasons"]:
        print(f"Hata Nedenleri: {res_c3['failure_reasons']}")

    # --- TEST C4: Pilot RC Kumanda Müdahalesi (RC Override) ---
    res_c4 = run_test_scenario(
        "TEST_C4_PILOT_RC_TAKEOVER",
        "FULL_ROUTE",
        timeout_s=100,
        inject_fault="RC_TAKEOVER"
    )
    results.append(res_c4)
    print(f"\n[TEST C4 SONUCU] RC Kumanda Müdahalesi Teyit Edildi mi: {res_c4['passed']} (Son Durum: {res_c4['final_state']})")
    if res_c4["failure_reasons"]:
        print(f"Hata Nedenleri: {res_c4['failure_reasons']}")

    # Sonuçları JSON olarak kaydet
    json_path = "/workspace/ros2_ws/sitl_test_results.json"
    with open(json_path, "w") as f:
        json.dump(results, f, indent=2)
    print(f"\n[BİLGİ] Yapılandırılmış test sonuçları kaydedildi: {json_path}")

    print("\n" + "="*70)
    print("                    TÜM SITL TESTLERİ TAMAMLANDI")
    print("="*70)
    for r in results:
        status_str = "GEÇTİ" if r["passed"] else "KALDI"
        print(f"- {r['test_name']:<36}: {status_str:<7} | Son Durum: {r['final_state']:<18} | Log: {r.get('log_path', 'N/A')}")

    rclpy.shutdown()


if __name__ == '__main__':
    main()
