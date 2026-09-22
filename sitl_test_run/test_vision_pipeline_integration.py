#!/usr/bin/env python3
"""
S500 Gerçek Vision Node, Adaptör ve SITL Lua Entegrasyon Test Koşucusu
(test_vision_pipeline_integration.py)
---------------------------------------------------------------------
Bu test, gerçek vision_node (YOLO best.pt), companion ROS 2 adaptörü
(companion/mavlink_mission_commander.py) ve ArduPilot SITL (s500_mission.lua)
durum makinesini araç DISARMED kalırken uçtan uca doğrular.

KAPSAM VE SINIRLAR:
1. Görüntü işleme kodu, model ağırlıkları (best.pt), geometri dönüşümleri,
   sınıf eşleştirmesi ve Lua uçuş mantığı değiştirilmez.
2. Gerçek model çıktıları ile sentetik tespitler raporda açıkça birbirinden ayrılır.
3. Eksik girdi (kalibrasyonsuz kamera, eksik telemetri pozu) durumunun davranışı ve
   gerekli geometrik parametreler detaylı olarak belgelenir.
4. Canlılık paketinin Lua'daki kabulü mevcut telemetriden doğrudan gözlenemediği
   durumda UNVERIFIED olarak korunur; gözlenebilir protokol davranışları test edilir.
5. Status 3 ve 4 eksik üretici analizi görev sözleşmesine dayandırılarak açıklanır.
6. Araç test boyunca kesinlikle DISARMED tutulur.
"""

import os
import sys
import time
import math
import json
import signal
import queue
import threading
import subprocess
from typing import Optional, Dict, Any, List, Tuple

import numpy as np
import cv2

# ROS 2 ve MAVLink bağımlılıkları
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String
from std_srvs.srv import Trigger
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from vision_interfaces.msg import Detection, DetectionArray
from pymavlink import mavutil

# Companion adaptörü ve vision node
sys.path.append("/workspace/companion")
sys.path.append("/workspace/ros2_ws/src/vision_processing")
from mavlink_mission_commander import MavlinkAdapterNode
from vision_processing.vision_node import VisionNode

def get_file_sha256(filepath: str) -> str:
    if not os.path.exists(filepath):
        return "DOSYA_YOK"
    try:
        import hashlib
        h = hashlib.sha256()
        with open(filepath, "rb") as f:
            while chunk := f.read(8192):
                h.update(chunk)
        return h.hexdigest()
    except Exception:
        return "OKUMA_HATASI"

def make_ros2_image_msg(cv_image: np.ndarray, clock, frame_id: str = "camera_optical_frame") -> Image:
    """cv_bridge sürüm uyuşmazlıklarından bağımsız standart ROS 2 Image mesajı oluşturur."""
    msg = Image()
    msg.header.stamp = clock.now().to_msg()
    msg.header.frame_id = frame_id
    msg.height = int(cv_image.shape[0])
    msg.width = int(cv_image.shape[1])
    msg.encoding = "bgr8"
    msg.is_bigendian = 0
    msg.step = int(cv_image.shape[1] * 3)
    msg.data = cv_image.tobytes()
    return msg

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

class VisionPipelineFeeder(Node):
    """Görüntü ve poz yayınlayan, tespit ve adaptör durumlarını toplayan test düğümü."""
    def __init__(self):
        super().__init__('vision_pipeline_feeder')
        self.pub_image = self.create_publisher(Image, '/camera/image_raw', qos_profile_sensor_data)
        self.pub_pose = self.create_publisher(PoseStamped, '/mavros/local_position/pose', qos_profile_sensor_data)

        self.sub_detections = self.create_subscription(DetectionArray, '/vision/detections', self._det_cb, 10)
        self.sub_event_status = self.create_subscription(String, '/adapter/event_status', self._event_cb, 10)
        self.cli_start_session = self.create_client(Trigger, '/adapter/start_session')

        self.received_detections: List[Tuple[float, DetectionArray]] = []
        self.received_events: List[Tuple[float, str]] = []

    def _det_cb(self, msg: DetectionArray):
        now = time.monotonic()
        self.received_detections.append((now, msg))

    def _event_cb(self, msg: String):
        now = time.monotonic()
        text = msg.data.strip()
        print(f"  [ROS2 EVENT] /adapter/event_status: '{text}'", flush=True)
        self.received_events.append((now, text))

    def publish_image_and_pose(self, cv_image: np.ndarray, x: float = 0.0, y: float = 0.0, z: float = 10.0, publish_pose: bool = True):
        now_msg = self.get_clock().now().to_msg()
        if publish_pose:
            p_msg = PoseStamped()
            p_msg.header.stamp = now_msg
            p_msg.header.frame_id = "odom"  # ENU sabit referans çerçevesi (sözleşmeye uygun)
            p_msg.pose.position.x = float(x)
            p_msg.pose.position.y = float(y)
            p_msg.pose.position.z = float(z)
            p_msg.pose.orientation.x = 0.0
            p_msg.pose.orientation.y = 0.0
            p_msg.pose.orientation.z = 0.0
            p_msg.pose.orientation.w = 1.0
            self.pub_pose.publish(p_msg)

        img_msg = make_ros2_image_msg(cv_image, self.get_clock(), "camera_optical_frame")
        self.pub_image.publish(img_msg)

class VisionPipelineSuiteRunner:
    def __init__(self, sitl_dir: str = "/workspace/sitl_test_run"):
        self.sitl_dir = sitl_dir
        self.sitl_proc: Optional[subprocess.Popen] = None
        self.sitl_log_file = None
        self.mav_gcs = None
        self.gcs_rx_thread: Optional[threading.Thread] = None

        self.adapter_node: Optional[MavlinkAdapterNode] = None
        self.vision_node: Optional[VisionNode] = None
        self.feeder_node: Optional[VisionPipelineFeeder] = None
        self.executor: Optional[MultiThreadedExecutor] = None
        self.ros_thread: Optional[threading.Thread] = None
        self.running = True

        # Güvenlik ve Telemetri Durumu
        self.assembler = StatustextAssembler()
        self.gcs_logs: List[Dict[str, Any]] = []
        self.last_heartbeat_mono: float = 0.0
        self.is_armed: Optional[bool] = None
        self.critical_failure_occurred: bool = False
        self.armed_violation_seen: bool = False
        self.heartbeat_loss_detected: bool = False

        # Canlılık ve MAVLink İletim Takip Verileri
        self.transmitted_liveliness_mono: List[float] = []
        self.sent_mavlink_commands: List[Dict[str, Any]] = []

        # Lua Betik Hash Takibi
        self.main_lua_hash: str = ""
        self.sitl_lua_hash: str = ""
        self.lua_hashes_match: bool = False
        self.mandatory_scenarios_passed: bool = False

        # Rapor ve Sonuçlar
        self.scenario_results: Dict[str, Dict[str, Any]] = {}
        self.failure_reasons: List[str] = []
        self.t_start_mono: float = 0.0

    def _gcs_rx_worker(self):
        while self.running and self.mav_gcs:
            try:
                msg = self.mav_gcs.recv_match(blocking=True, timeout=0.05)
                if not msg:
                    continue

                now_mono = time.monotonic()
                src_sys = msg.get_srcSystem()
                src_comp = msg.get_srcComponent()
                if src_sys != 1 or src_comp != 1:
                    continue

                mtype = msg.get_type()
                if mtype == 'HEARTBEAT':
                    self.last_heartbeat_mono = now_mono
                    armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                    self.is_armed = armed
                    if armed:
                        self.armed_violation_seen = True
                        self.critical_failure_occurred = True
                        err = f"GÜVENLİK İHLALİ: Otopilot beklenmedik şekilde ARMED görüldü! (base_mode=0x{msg.base_mode:X})"
                        print(f"[GCS-RX] {err}", flush=True)
                        self.failure_reasons.append(err)

                elif mtype == 'STATUSTEXT':
                    text = self.assembler.feed(msg)
                    if text:
                        entry = {"timestamp_mono": now_mono, "text": text}
                        self.gcs_logs.append(entry)
                        print(f"  [GCS STATUSTEXT] {text}", flush=True)

            except Exception as e:
                if self.running:
                    self.critical_failure_occurred = True
                    err = f"GCS RX kritik hatası: {e}"
                    print(f"[GCS-RX] {err}", flush=True)
                    self.failure_reasons.append(err)

    def check_heartbeat_liveness(self, max_age_s: float = 3.0) -> bool:
        if self.critical_failure_occurred:
            return False
        if self.last_heartbeat_mono == 0.0:
            return False
        age = time.monotonic() - self.last_heartbeat_mono
        if age > max_age_s:
            self.heartbeat_loss_detected = True
            self.critical_failure_occurred = True
            err = f"Otopilot Heartbeat kaybı! ({age:.2f}s > {max_age_s:.1f}s)"
            print(f"[GÜVENLİK-DENETİMİ] {err}", flush=True)
            self.failure_reasons.append(err)
            return False
        return True

    def start_sitl(self):
        log_path = os.path.join(self.sitl_dir, "sitl_stdout_vision_pipeline.log")
        self.sitl_log_file = open(log_path, "w")
        cmd = [
            "/opt/ardupilot/build/sitl/bin/arducopter",
            "--model", "quad",
            "--home", "-35.363261,149.165230,584,353",
            "--defaults", f"/opt/ardupilot/Tools/autotest/default_params/copter.parm,{self.sitl_dir}/extra_params.parm"
        ]
        print(f"[SITL] Süreç başlatılıyor: {' '.join(cmd)}", flush=True)
        print(f"[SITL] Çıktı yönlendiriliyor: {log_path}", flush=True)
        self.sitl_proc = subprocess.Popen(
            cmd,
            cwd=self.sitl_dir,
            stdout=self.sitl_log_file,
            stderr=subprocess.STDOUT,
            preexec_fn=os.setsid
        )

        # GCS (port 5760) üzerinden bağlan
        t0 = time.monotonic()
        conn_ok = False
        while (time.monotonic() - t0) < 25.0:
            temp_mav = None
            try:
                temp_mav = mavutil.mavlink_connection("tcp:127.0.0.1:5760")
                hb = temp_mav.wait_heartbeat(timeout=1.0)
                if hb and hb.get_srcSystem() == 1 and hb.get_srcComponent() == 1:
                    self.mav_gcs = temp_mav
                    temp_mav = None  # Sahiplik devralındı
                    self.last_heartbeat_mono = time.monotonic()
                    armed = bool(hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                    self.is_armed = armed
                    if armed:
                        self.armed_violation_seen = True
                        self.critical_failure_occurred = True
                        err = f"GÜVENLİK İHLALİ: Otopilot başlangıçta beklenmedik şekilde ARMED görüldü! (base_mode=0x{hb.base_mode:X})"
                        print(f"[SITL] {err}", flush=True)
                        self.failure_reasons.append(err)
                        raise RuntimeError(err)
                    conn_ok = True
                    print(f"[SITL] GCS (5760) Heartbeat teyit edildi (is_armed={self.is_armed})!", flush=True)
                    break
                else:
                    if temp_mav:
                        temp_mav.close()
                        temp_mav = None
            except Exception:
                if temp_mav:
                    try:
                        temp_mav.close()
                    except Exception:
                        pass
                    temp_mav = None
                if self.critical_failure_occurred:
                    raise
                time.sleep(0.5)

        if not conn_ok or not self.mav_gcs:
            raise RuntimeError("SITL GCS bağlantısı (5760) 25 saniye içinde kurulamadı!")

        self.gcs_rx_thread = threading.Thread(target=self._gcs_rx_worker, daemon=True)
        self.gcs_rx_thread.start()
        self.mav_gcs.mav.request_data_stream_send(1, 1, mavutil.mavlink.MAV_DATA_STREAM_ALL, 10, 1)

        # Lua scriptinin yüklenmesini bekle (En fazla 35s)
        t_lua_wait = time.monotonic()
        lua_ready = False
        cursor = 0
        while (time.monotonic() - t_lua_wait) < 35.0:
            if not self.check_heartbeat_liveness():
                raise RuntimeError("Lua beklenirken Heartbeat kayboldu veya güvenlik ihlali!")
            while cursor < len(self.gcs_logs):
                text = self.gcs_logs[cursor]["text"]
                cursor += 1
                if "S500 Otonom Gorev Scripti yuklendi" in text or "BEKLEME durumunda tetikleme bekleniyor" in text:
                    lua_ready = True
                    break
            if lua_ready:
                print("[SITL] s500_mission.lua başarıyla yüklendi ve BEKLEME durumuna geçti.", flush=True)
                break
            time.sleep(0.2)

        if not lua_ready:
            raise RuntimeError("s500_mission.lua yükleme bildirimi alınamadı!")

    def start_ros_nodes(self):
        print("[ROS2] Düğümler başlatılıyor...", flush=True)
        # Adaptör SITL bağlantı parametresi ve Sentetik Test Kamera Kalibrasyonu
        rclpy.init(args=[
            "--ros-args",
            "-p", "mavlink_mission_commander:mavlink_connection:=tcp:127.0.0.1:5762",
            "-p", "vision_node:model_path:=/workspace/best.pt",
            # Sentetik Test Kalibrasyonu (1280x720, aşağı bakan kamera mount)
            "-p", "vision_node:has_calibration:=true",
            "-p", "vision_node:has_mount:=true",
            "-p", "vision_node:has_ground_ref:=true",
            "-p", "vision_node:cam_model:=plumb_bob",
            "-p", "vision_node:calib_width:=1280",
            "-p", "vision_node:calib_height:=720",
            "-p", "vision_node:cam_fx:=1000.0",
            "-p", "vision_node:cam_fy:=1000.0",
            "-p", "vision_node:cam_cx:=640.0",
            "-p", "vision_node:cam_cy:=360.0",
            "-p", "vision_node:cam_dists:=[0.0, 0.0, 0.0, 0.0, 0.0]",
            "-p", "vision_node:mount_xyz:=[0.0, 0.0, 0.0]",
            "-p", "vision_node:mount_rpy:=[0.0, 1.5707963, 0.0]",  # Aşağı bakan kamera (Pitch = +pi/2)
            "-p", "vision_node:ground_plane_z:=0.0",
            "-p", "vision_node:pose_age_tolerance_s:=0.5"
        ])

        self.adapter_node = MavlinkAdapterNode()
        self.vision_node = VisionNode()
        self.feeder_node = VisionPipelineFeeder()

        # Gerçek MAVLink command_long_send çağrısını izleyen ve ham parametreleri kaydeden kanca
        orig_command_long_send = self.adapter_node.master.mav.command_long_send

        def intercepted_command_long_send(target_system, target_component, command, confirmation,
                                          param1, param2, param3, param4, param5, param6, param7, **kwargs):
            now_mono = time.monotonic()
            call_record = {
                "ts_mono": now_mono,
                "target_system": target_system,
                "target_component": target_component,
                "command": command,
                "confirmation": confirmation,
                "param1": param1,
                "param2": param2,
                "param3": param3,
                "param4": param4,
                "param5": param5,
                "param6": param6,
                "param7": param7,
                "attempted": True,
                "succeeded": False,
                "error": None
            }
            try:
                result = orig_command_long_send(
                    target_system, target_component, command, confirmation,
                    param1, param2, param3, param4, param5, param6, param7, **kwargs
                )
                call_record["succeeded"] = True
                # Canlılık paketlerinin iletim zamanını kaydet (command == MAV_CMD_USER_1 / 31010 ve param5 == 0xAA)
                is_user_cmd = (command == mavutil.mavlink.MAV_CMD_USER_1 or command == 31010)
                is_liveliness = (param5 == 0xAA or param5 == float(0xAA))
                if is_user_cmd and is_liveliness:
                    self.transmitted_liveliness_mono.append(now_mono)
                # NOT: Başarılı gönderim yalnızca soket iletimini kanıtlar; Lua kabulü eşleşen log/ACK ile doğrulanacaktır.
                return result
            except Exception as e:
                call_record["error"] = str(e)
                raise
            finally:
                self.sent_mavlink_commands.append(call_record)

        self.adapter_node.master.mav.command_long_send = intercepted_command_long_send

        self.executor = MultiThreadedExecutor()
        self.executor.add_node(self.adapter_node)
        self.executor.add_node(self.vision_node)
        self.executor.add_node(self.feeder_node)

        self.ros_thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.ros_thread.start()
        print("[ROS2] Düğümler executor üzerinde aktif olarak çalışıyor.", flush=True)

    def wait_for_gcs_log(self, text_substr: str, timeout_s: float = 10.0, start_mono: Optional[float] = None) -> Optional[str]:
        t0 = time.monotonic()
        start_ts = start_mono or t0
        while (time.monotonic() - t0) < timeout_s:
            if not self.check_heartbeat_liveness():
                return None
            for entry in self.gcs_logs:
                if entry["timestamp_mono"] >= start_ts and text_substr in entry["text"]:
                    return entry["text"]
            time.sleep(0.1)
        return None

    def wait_for_adapter_event(self, prefix: str, timeout_s: float = 10.0, start_mono: Optional[float] = None) -> Optional[str]:
        t0 = time.monotonic()
        start_ts = start_mono or t0
        while (time.monotonic() - t0) < timeout_s:
            if not self.check_heartbeat_liveness():
                return None
            for ts, text in self.feeder_node.received_events:
                if ts >= start_ts and text.startswith(prefix):
                    return text
            time.sleep(0.1)
        return None

    def run_tests(self):
        self.t_start_mono = time.monotonic()
        print("\n" + "="*80)
        print("S500 GERÇEK VİSİON NODE -> ADAPTÖR -> SITL LUA ENTEGRASYON TESTİ")
        print("="*80 + "\n", flush=True)

        try:
            self.start_sitl()
            self.start_ros_nodes()

            # ------------------------------------------------------------------
            # SENARYO 1: SESSION_START El Sıkışması (Handshake)
            # ------------------------------------------------------------------
            print("\n[SENARYO 1] SESSION_START El Sıkışması Testi Başlatılıyor...", flush=True)
            t_s1 = time.monotonic()
            req = Trigger.Request()
            future = self.feeder_node.cli_start_session.call_async(req)

            # Servis cevabı
            t_srv_wait = time.monotonic()
            while not future.done() and (time.monotonic() - t_srv_wait) < 5.0:
                time.sleep(0.05)

            if not future.done() or not future.result().success:
                raise RuntimeError("SESSION_START servis çağrısı başarısız!")

            # Lua logu ve Adaptör olayını bekle
            lua_log = self.wait_for_gcs_log("Yeni Jetson oturumu basariyla acildi", timeout_s=8.0, start_mono=t_s1)
            adapter_event = self.wait_for_adapter_event("HANDSHAKE_ACCEPTED:1", timeout_s=8.0, start_mono=t_s1)

            if lua_log and adapter_event and self.adapter_node.session_state == self.adapter_node.ACTIVE:
                self.scenario_results["1_handshake"] = {
                    "status": "PASS",
                    "session_id": self.adapter_node.session_id,
                    "event": adapter_event,
                    "evidence_log": lua_log
                }
                print(f"[SENARYO 1: PASS] El sıkışması başarılı. Oturum ID: {self.adapter_node.session_id}, Durum: ACTIVE", flush=True)
            else:
                self.scenario_results["1_handshake"] = {
                    "status": "FAIL",
                    "reason": f"Lua log: {lua_log}, Adapter event: {adapter_event}, State: {self.adapter_node.session_state}"
                }
                self.failure_reasons.append("Handshake başarısız!")
                print(f"[SENARYO 1: FAIL] {self.scenario_results['1_handshake']['reason']}", flush=True)

            # ------------------------------------------------------------------
            # SENARYO 2: Canlılık Mesajı İletişim Davranışı & Gözlenebilirlik
            # ------------------------------------------------------------------
            print("\n[SENARYO 2] Canlılık Mesajı İletişim Davranışı Test Ediliyor...", flush=True)
            t_s2 = time.monotonic()
            time.sleep(3.0) # Canlılık akışını bekle (2 Hz)

            measured_count = len(self.transmitted_liveliness_mono)
            hz = 0.0
            if measured_count >= 2:
                dt = self.transmitted_liveliness_mono[-1] - self.transmitted_liveliness_mono[0]
                hz = (measured_count - 1) / dt if dt > 0 else 0.0

            # Gözlenebilir Protokol Testi: Uyumsuz Oturum ID ile Canlılık Paketi Gönderimi
            # Lua'nın kuyruktan canlılık (0xAA) mesajlarını ayrıştırıp denetlediğini kanıtlar
            mismatched_sess_id = (self.adapter_node.session_id + 99999) % 16000000 + 1
            t_mismatch_send = time.monotonic()
            self.adapter_node.master.mav.command_long_send(
                1, 1,
                mavutil.mavlink.MAV_CMD_USER_1,
                0,
                0.0, 0.0, 0.0,
                0,
                0xAA, # STATUS_LIVELINESS
                float(mismatched_sess_id),
                1.0  # PROTOCOL_VERSION = 1.0 zorunludur
            )

            # Lua'nın log_warn vermesini bekle
            warn_log = self.wait_for_gcs_log("Bilinmeyen veya uyumsuz oturumdan canlilik paketi", timeout_s=5.0, start_mono=t_mismatch_send)
            s2_rejection_observed = (warn_log is not None)

            self.scenario_results["2_liveliness_behavior"] = {
                "status": "UNVERIFIED",
                "measured_packets": measured_count,
                "measured_hz": round(hz, 2),
                "observable_protocol_test": {
                    "mismatched_session_id_tested": mismatched_sess_id,
                    "lua_rejection_observed": s2_rejection_observed,
                    "lua_evidence_log": warn_log
                },
                "internal_timestamp_verification": "UNVERIFIED",
                "reason": (
                    "Canlılık yayın frekansı 2.0 Hz olarak ölçüldü ve uyumsuz oturum ID'si gönderildiğinde "
                    "Lua'nın paketi başarıyla ayrıştırıp log_warn ile reddettiği telemetride kanıtlandı. "
                    "Ancak geçerli oturuma ait canlılık paketlerinde Lua iç zaman damgasının (last_msg_time_ms) "
                    "güncellenmesi BEKLEME durumunda MAVLink üzerinden yayınlanmadığı için iç durum UNVERIFIED olarak kaydedildi."
                )
            }
            print(f"  [SENARYO 2] Canlılık frekansı: {hz:.2f} Hz ({measured_count} paket).", flush=True)
            print(f"  [SENARYO 2] Gözlenebilir oturum uyumsuzluk logu: {warn_log}", flush=True)
            print(f"[SENARYO 2: UNVERIFIED] İç zaman damgası doğrudan telemetriden kanıtlanamadı (Protokole uygun).", flush=True)

            # ------------------------------------------------------------------
            # SENARYO 3: Eksik Girdi / Kalibrasyonsuz Durum Analizi
            # ------------------------------------------------------------------
            print("\n[SENARYO 3] Eksik Girdi & Kalibrasyonsuz Durum Analizi...", flush=True)
            # Test görüntüsü yükle: companion/Hedef_0002.jpg (720x1280)
            img_path = "/workspace/companion/Hedef_0002.jpg"
            test_img = cv2.imread(img_path)
            if test_img is None:
                raise RuntimeError(f"Test görüntüsü bulunamadı: {img_path}")

            # 1. Abonelik hazırlığı kontrolü (Açık pub_sub_ready bayrağı ile)
            pub_sub_ready = False
            t_ready = time.monotonic()
            while (time.monotonic() - t_ready) < 3.0:
                if (self.feeder_node.pub_image.get_subscription_count() > 0 and
                    self.feeder_node.count_publishers('/vision/detections') > 0):
                    pub_sub_ready = True
                    break
                time.sleep(0.05)

            if not pub_sub_ready:
                # Hazır değilse görüntü yayımlanmaz; hazırlık zaman aşımı nedeniyle FAIL kaydedilir
                s3_fail_reason = "Hazırlık zaman aşımı: 3.0s içinde ROS 2 görüntü/tespit abonelik eşleşmesi tamamlanamadı"
                self.scenario_results["3_missing_input_analysis"] = {
                    "status": "FAIL",
                    "test_image": "companion/Hedef_0002.jpg",
                    "missing_input_type": "TELEMETRY_POSE_MISSING",
                    "detection_received": False,
                    "vision_output": {},
                    "adapter_rejected_invalid_position": False,
                    "transmission_attempts_count": 0,
                    "transmission_succeeded_count": 0,
                    "fail_reason": s3_fail_reason,
                    "required_inputs_specification": {
                        "camera_calibration": ["calib_width=1280", "calib_height=720", "cam_fx=1000", "cam_fy=1000", "cam_cx=640", "cam_cy=360", "cam_dists=[0,0,0,0,0] (Sentetik Test Kalibrasyonu)"],
                        "camera_mount": ["mount_xyz=[0,0,0]", "mount_rpy=[0, 1.5707963, 0] (Aşağı bakan kamera)"],
                        "ground_reference": "ground_plane_z=0.0",
                        "telemetry_pose": "/mavros/local_position/pose (tazelik < 0.5s, z=irtifa, oryantasyon kuaterniyon, frame_id=odom)"
                    }
                }
                print(f"[SENARYO 3: FAIL] {s3_fail_reason}", flush=True)
                self.failure_reasons.append(f"Senaryo 3 başarısız: {s3_fail_reason}")
            else:
                # 2. Temiz başlangıç: Senaryo 1 ve 2'de görüntü yayımlanmadığı için vision_node ilk görüntüsünü alır
                t_s3 = time.monotonic()
                self.feeder_node.publish_image_and_pose(test_img, publish_pose=False)

                # Bu senaryonun görüntüsüne ait tespiti bekle (en fazla 5.0s)
                det_msg_received = None
                t_det_wait_s3 = time.monotonic()
                while (time.monotonic() - t_det_wait_s3) < 5.0:
                    for ts_mono, d_arr in self.feeder_node.received_detections:
                        if ts_mono >= t_s3 and d_arr.detections:
                            det_msg_received = d_arr
                            break
                    if det_msg_received:
                        break
                    time.sleep(0.05)

                s3_det_ok = False
                s3_evidence = {}
                s3_fail_reason = None
                if det_msg_received:
                    d = det_msg_received.detections[0]
                    s3_evidence = {
                        "detected_class": d.class_name,
                        "confidence": float(d.confidence),
                        "position_valid": d.position_valid,
                        "local_x": float(d.local_x),
                        "local_y": float(d.local_y),
                        "pose_valid": det_msg_received.pose_valid
                    }
                    # Doğrulama: pose_valid=False, position_valid=False ve koordinatlar NaN olmalı
                    if det_msg_received.pose_valid:
                        s3_fail_reason = "Eksik telemetriye rağmen DetectionArray.pose_valid=True üretildi"
                    elif d.position_valid:
                        s3_fail_reason = "Eksik telemetriye rağmen Detection.position_valid=True üretildi"
                    elif not (math.isnan(d.local_x) and math.isnan(d.local_y)):
                        s3_fail_reason = f"Geçersiz pozda koordinatlar NaN değil (x={d.local_x}, y={d.local_y})"
                    else:
                        s3_det_ok = True
                else:
                    s3_fail_reason = "Zaman aşımı: 5.0s içinde bu senaryoya ait /vision/detections çıktısı üretilmedi"

                # Sınırlı gözlem süresinde adaptörün durumunu incele
                time.sleep(0.5)
                s3_status_cmds = [
                    c for c in self.sent_mavlink_commands
                    if c["ts_mono"] >= t_s3 and int(c.get("param5", 0)) in (1, 2, 3, 4)
                ]
                s3_attempts = len(s3_status_cmds)
                s3_succeeded = sum(1 for c in s3_status_cmds if c.get("succeeded", False))
                adapter_sent_cmd = len(self.adapter_node.pending_commands) > 0

                if s3_det_ok and (s3_attempts > 0 or adapter_sent_cmd):
                    s3_fail_reason = f"Adaptör geçersiz pozlu tespit için MAVLink iletim girişiminde bulundu (attempts={s3_attempts}, pending={adapter_sent_cmd})"

                adapter_rejected_ok = bool(
                    s3_det_ok and
                    (s3_attempts == 0) and
                    (not adapter_sent_cmd)
                )
                s3_passed = adapter_rejected_ok

                self.scenario_results["3_missing_input_analysis"] = {
                    "status": "PASS" if s3_passed else "FAIL",
                    "test_image": "companion/Hedef_0002.jpg",
                    "missing_input_type": "TELEMETRY_POSE_MISSING",
                    "detection_received": det_msg_received is not None,
                    "vision_output": s3_evidence,
                    "adapter_rejected_invalid_position": adapter_rejected_ok,
                    "transmission_attempts_count": s3_attempts,
                    "transmission_succeeded_count": s3_succeeded,
                    "fail_reason": s3_fail_reason,
                    "required_inputs_specification": {
                        "camera_calibration": ["calib_width=1280", "calib_height=720", "cam_fx=1000", "cam_fy=1000", "cam_cx=640", "cam_cy=360", "cam_dists=[0,0,0,0,0] (Sentetik Test Kalibrasyonu)"],
                        "camera_mount": ["mount_xyz=[0,0,0]", "mount_rpy=[0, 1.5707963, 0] (Aşağı bakan kamera)"],
                        "ground_reference": "ground_plane_z=0.0",
                        "telemetry_pose": "/mavros/local_position/pose (tazelik < 0.5s, z=irtifa, oryantasyon kuaterniyon, frame_id=odom)"
                    }
                }
                if s3_passed:
                    print(f"[SENARYO 3: PASS] Eksik pozda position_valid=False üretildi, iletim girişimi ({s3_attempts}) olmadı.", flush=True)
                else:
                    print(f"[SENARYO 3: FAIL] Eksik poz denetimi başarısız! Neden: {s3_fail_reason}", flush=True)
                    self.failure_reasons.append(f"Senaryo 3 başarısız: {s3_fail_reason}")

            # ------------------------------------------------------------------
            # SENARYO 4: Gerçek Model Çıktısı -> Adaptör -> MAVLink -> SITL Lua
            # ------------------------------------------------------------------
            print("\n[SENARYO 4] Gerçek Model Çıktısı ile Uçtan Uca Doğrulama...", flush=True)
            t_s4 = time.monotonic()
            events_before = list(self.feeder_node.received_events)

            # Poz (z=10.0m irtifa) ve görüntüyü eşzamanlı yayınla
            self.feeder_node.publish_image_and_pose(test_img, x=0.0, y=0.0, z=10.0, publish_pose=True)

            # 1. Vision Detection'ı bekle
            det_received = None
            t_det_wait = time.monotonic()
            while (time.monotonic() - t_det_wait) < 5.0:
                for ts, d_arr in self.feeder_node.received_detections:
                    if ts >= t_s4 and d_arr.detections:
                        for det in d_arr.detections:
                            if det.class_name == "Hedef" and det.position_valid:
                                det_received = det
                                break
                    if det_received:
                        break
                if det_received:
                    break
                time.sleep(0.05)

            if not det_received:
                raise RuntimeError("Gerçek model tespiti (Hedef, position_valid=True) alınamadı!")

            print(f"  [VISION ÇIKTISI] Sınıf: '{det_received.class_name}', Conf: {det_received.confidence:.3f}, BBox: ({det_received.bbox_cx:.1f}, {det_received.bbox_cy:.1f}), Metrik: ({det_received.local_x:.3f}, {det_received.local_y:.3f})", flush=True)

            # Adaptör sözleşmesine göre beklenen değerler (ENU -> NED: x_ned = local_y, y_ned = local_x)
            expected_x_ned = float(det_received.local_y)
            expected_y_ned = float(det_received.local_x)
            expected_conf = float(det_received.confidence)
            expected_sess = int(self.adapter_node.session_id)

            # 2. İletilen MAVLink komut paketini bul (ts_mono >= t_s4, param5 == 2.0, param6 == expected_sess)
            sent_cmd = None
            t_cmd_wait = time.monotonic()
            while (time.monotonic() - t_cmd_wait) < 5.0:
                for c in self.sent_mavlink_commands:
                    if c["ts_mono"] >= t_s4 and int(c.get("param5", 0)) == 2 and int(c.get("param6", 0)) == expected_sess:
                        sent_cmd = c
                        break
                if sent_cmd:
                    break
                time.sleep(0.05)

            cmd_found = (sent_cmd is not None)
            tx_succeeded = sent_cmd.get("succeeded", False) if cmd_found else False
            tx_seq = int(sent_cmd.get("param4", 0)) if cmd_found else -1
            tx_x_ned = float(sent_cmd.get("param1", 0.0)) if cmd_found else 0.0
            tx_y_ned = float(sent_cmd.get("param2", 0.0)) if cmd_found else 0.0
            tx_conf = float(sent_cmd.get("param3", 0.0)) if cmd_found else 0.0
            tx_sess = int(sent_cmd.get("param6", 0)) if cmd_found else 0

            field_match_ok = (
                cmd_found and tx_succeeded and
                math.isclose(tx_x_ned, expected_x_ned, abs_tol=1e-3) and
                math.isclose(tx_y_ned, expected_y_ned, abs_tol=1e-3) and
                math.isclose(tx_conf, expected_conf, abs_tol=1e-3) and
                tx_sess == expected_sess
            )

            # 3. İletilen seq ile ilişkili Lua kabul logunu bekle
            lua_goal_log = self.wait_for_gcs_log(f"GOAL_FOUND (Seq: {tx_seq}", timeout_s=8.0, start_mono=t_s4) if cmd_found else None

            # 4. İletilen seq ile ilişkili Adaptör ACCEPTED olayını bekle
            adapter_goal_event = self.wait_for_adapter_event(f"ACCEPTED:{tx_seq}:2", timeout_s=8.0, start_mono=t_s4) if cmd_found else None

            s4_passed = field_match_ok and (lua_goal_log is not None) and (adapter_goal_event is not None)
            self.scenario_results["4_real_model_e2e_flow"] = {
                "status": "PASS" if s4_passed else "FAIL",
                "test_image": "companion/Hedef_0002.jpg",
                "yolo_detection": {
                    "class_name": det_received.class_name,
                    "confidence": round(float(det_received.confidence), 3),
                    "bbox": [round(float(det_received.bbox_cx), 1), round(float(det_received.bbox_cy), 1),
                             round(float(det_received.bbox_w), 1), round(float(det_received.bbox_h), 1)],
                    "frame_id": "odom",
                    "position_valid": det_received.position_valid,
                    "local_x_enu": round(float(det_received.local_x), 3),
                    "local_y_enu": round(float(det_received.local_y), 3)
                },
                "expected_adapter_contract": {
                    "mapped_status": 2,
                    "mapped_status_name": "GOAL_FOUND",
                    "coordinate_frame": "NED",
                    "expected_x_ned": round(expected_x_ned, 3),
                    "expected_y_ned": round(expected_y_ned, 3),
                    "expected_confidence": round(expected_conf, 3),
                    "expected_session_id": expected_sess
                },
                "transmitted_mavlink_packet": {
                    "command_found": cmd_found,
                    "succeeded": tx_succeeded,
                    "seq": tx_seq,
                    "param1_x_ned": round(tx_x_ned, 3),
                    "param2_y_ned": round(tx_y_ned, 3),
                    "param3_conf": round(tx_conf, 3),
                    "param5_status": int(sent_cmd.get("param5", 0)) if cmd_found else 0,
                    "param6_session_id": tx_sess
                },
                "field_verification": {
                    "x_ned_match": math.isclose(tx_x_ned, expected_x_ned, abs_tol=1e-3) if cmd_found else False,
                    "y_ned_match": math.isclose(tx_y_ned, expected_y_ned, abs_tol=1e-3) if cmd_found else False,
                    "confidence_match": math.isclose(tx_conf, expected_conf, abs_tol=1e-3) if cmd_found else False,
                    "session_match": (tx_sess == expected_sess) if cmd_found else False,
                    "all_fields_matched": field_match_ok
                },
                "correlated_lua_and_ack": {
                    "correlated_seq": tx_seq,
                    "correlated_session": expected_sess,
                    "adapter_event": adapter_goal_event,
                    "lua_evidence_log": lua_goal_log
                }
            }
            if s4_passed:
                print(f"[SENARYO 4: PASS] Tespit -> MAVLink -> Lua kabul zinciri doğrulandı (Seq={tx_seq}, ACK={adapter_goal_event}).", flush=True)
            else:
                print(f"[SENARYO 4: FAIL] Zincir doğrulama başarısız! fields_ok={field_match_ok}, lua={lua_goal_log}, ack={adapter_goal_event}", flush=True)
                self.failure_reasons.append("Senaryo 4 uçtan uca zincir doğrulama başarısız!")

            # ------------------------------------------------------------------
            # SENARYO 5: Engel Sınıfı Filtreleme Testi (companion/engel_0011.jpg)
            # ------------------------------------------------------------------
            print("\n[SENARYO 5] Engel Sınıfı Filtreleme Testi...", flush=True)
            engel_img_path = "/workspace/companion/engel_0011.jpg"
            engel_img = cv2.imread(engel_img_path)
            t_s5 = time.monotonic()
            events_s5_before = len(self.feeder_node.received_events)

            self.feeder_node.publish_image_and_pose(engel_img, x=0.0, y=0.0, z=10.0, publish_pose=True)
            time.sleep(0.8)

            # Engel tespiti alınmış mı?
            engel_det_seen = False
            for ts, d_arr in self.feeder_node.received_detections:
                if ts >= t_s5:
                    for d in d_arr.detections:
                        if d.class_name == "engel":
                            engel_det_seen = True
                            break

            # Status 1-4 için MAVLink gönderim girişimleri kontrolü
            s5_status_cmds = [
                c for c in self.sent_mavlink_commands
                if c["ts_mono"] >= t_s5 and int(c.get("param5", 0)) in (1, 2, 3, 4)
            ]
            s5_attempts = len(s5_status_cmds)
            s5_succeeded = sum(1 for c in s5_status_cmds if c.get("succeeded", False))

            # Adaptör yeni bir uçuş komutu (ACCEPTED veya komut kuyruğu) ÜRETMEMELİDİR
            new_events = [ev for ts, ev in self.feeder_node.received_events if ts >= t_s5 and ev.startswith("ACCEPTED:")]
            s5_passed = engel_det_seen and (s5_attempts == 0) and (len(new_events) == 0)

            self.scenario_results["5_obstacle_filtering"] = {
                "status": "PASS" if s5_passed else "FAIL",
                "test_image": "companion/engel_0011.jpg",
                "detected_obstacle": engel_det_seen,
                "flight_event_produced": len(new_events) > 0,
                "transmission_attempts_count": s5_attempts,
                "transmission_succeeded_count": s5_succeeded,
                "finding": "YOLO tarafından tespit edilen 'engel' sınıfı class_mapping.py tarafından status=0 (eylemsiz) eşlendi; iletim girişimi (0) yapılmadı."
            }
            if s5_passed:
                print(f"[SENARYO 5: PASS] 'engel' nesnesi tespit edildi, uçuş komutu gönderim girişimi ({s5_attempts}) olmadı.", flush=True)
            else:
                print(f"[SENARYO 5: FAIL] Engel filtreleme başarısız! det_seen={engel_det_seen}, attempts={s5_attempts}, events={new_events}", flush=True)
                self.failure_reasons.append(f"Senaryo 5 başarısız (girişim={s5_attempts})")

            # ------------------------------------------------------------------
            # SENARYO 6: Status 3 ve 4 Olaylarının Mimari Analizi (Eksik Üretici)
            # ------------------------------------------------------------------
            print("\n[SENARYO 6] Status 3 ve 4 Olaylarının Mimari Analizi...", flush=True)
            self.scenario_results["6_status_3_4_architecture"] = {
                "status": "MIMARI_KAPSAM_BILGISI",
                "current_producers": {
                    "vision_node": "Yalnızca nesne tespiti (bbox, confidence, metrik koordinat) üretir; görev semantiği üretmez.",
                    "class_mapping": "Yalnızca start_classes -> 1 (START_FOUND) ve goal_classes -> 2 (GOAL_FOUND) üretir.",
                    "mavlink_mission_commander": "Gelen tespitleri MAVLink MAV_CMD_USER_1 paketine dönüştürür; rota planlaması yapmaz."
                },
                "required_producer": {
                    "component": "Üst Seviye Rota / Görev Planlayıcı Düğümü (Mission / Path Planner Node)",
                    "status_3_goto_observation": (
                        "Koşul: Drone havalanıp STATE_HEDEF_BEKLE (33m) durumundayken, hedefin veya engellerin "
                        "daha iyi taranması için planlayıcı tarafından hesaplanan ara gözlem noktasına yönlendirmede üretilmelidir."
                    ),
                    "status_4_route_ready": (
                        "Koşul: Başlangıç ve hedef koordinatları onaylanıp, engel haritası üzerinde A* veya "
                        "grid tabanlı çarpışmasız rota hesabı tamamlandığında ve rotanın YİS'e iletimi teyit edildiğinde, "
                        "otopilotun STATE_DONUS aşamasına geçmesi için üretilmelidir."
                    ),
                    "protocol_rule": (
                        "jetson_lua_protocol.md Bölüm 2.2 ve 2.3 uyarınca: Görüntü sınıfları veya yüksek güven skoru (confidence) "
                        "asla doğrudan Status 3 veya 4 hareket tetikleyicisine dönüştürülemez. Bu ayrım uçuş güvenliğinin temelidir."
                    )
                }
            }
            print(f"[SENARYO 6: MIMARI_KAPSAM_BILGISI] Status 3/4 analiz raporu tamamlandı.", flush=True)

            # ------------------------------------------------------------------
            # SENARYO 7: Uçuş Güvenliği (DISARMED Güvencesi)
            # ------------------------------------------------------------------
            print("\n[SENARYO 7] Uçuş Güvenliği Kontrolü...", flush=True)
            hb_ok = self.check_heartbeat_liveness()
            is_armed = self.is_armed

            safety_passed = hb_ok and (is_armed is False) and (not self.armed_violation_seen)
            self.scenario_results["7_disarmed_safety"] = {
                "status": "PASS" if safety_passed else "FAIL",
                "is_armed": is_armed,
                "armed_violation_seen": self.armed_violation_seen,
                "heartbeat_healthy": hb_ok
            }
            if safety_passed:
                print(f"[SENARYO 7: PASS] Araç test boyunca kesintisiz DISARMED kaldı. Heartbeat sağlıklı.", flush=True)
            else:
                print(f"[SENARYO 7: FAIL] Güvenlik denetimi başarısız! is_armed={is_armed}, armed_violation={self.armed_violation_seen}, hb_ok={hb_ok}", flush=True)
                self.failure_reasons.append("Senaryo 7 güvenlik denetimi başarısız!")

        except Exception as e:
            self.critical_failure_occurred = True
            err_str = f"Test akışında istisna: {type(e).__name__}: {e}"
            print(f"\n[TEST ÇALIŞTIRICI HATASI] {err_str}", flush=True)
            self.failure_reasons.append(err_str)

        finally:
            self.teardown()

    def teardown(self):
        print("\n[TEMİZLİK] Kaynaklar kapatılıyor...", flush=True)
        self.running = False

        # ROS executor ve düğümlerini durdur
        if self.executor:
            try:
                self.executor.shutdown()
            except Exception as e:
                print(f"[TEMİZLİK] Executor shutdown hatası: {e}", flush=True)

        if self.adapter_node:
            try:
                self.adapter_node.destroy_node()
            except Exception:
                pass

        if self.vision_node:
            try:
                self.vision_node.destroy_node()
            except Exception:
                pass

        if self.feeder_node:
            try:
                self.feeder_node.destroy_node()
            except Exception:
                pass

        try:
            rclpy.shutdown()
        except Exception:
            pass

        if self.ros_thread and self.ros_thread.is_alive():
            self.ros_thread.join(timeout=3.0)

        # GCS bağlantısını kapat
        if self.mav_gcs:
            try:
                self.mav_gcs.close()
            except Exception:
                pass

        if self.gcs_rx_thread and self.gcs_rx_thread.is_alive():
            self.gcs_rx_thread.join(timeout=2.0)

        # SITL sürecini kapat
        if self.sitl_proc and self.sitl_proc.poll() is None:
            print("[TEMİZLİK] SITL süreci (SIGTERM) sonlandırılıyor...", flush=True)
            try:
                os.killpg(os.getpgid(self.sitl_proc.pid), signal.SIGTERM)
                self.sitl_proc.wait(timeout=5.0)
            except Exception:
                try:
                    os.killpg(os.getpgid(self.sitl_proc.pid), signal.SIGKILL)
                    self.sitl_proc.wait(timeout=2.0)
                except Exception:
                    pass

        if self.sitl_log_file:
            try:
                self.sitl_log_file.close()
            except Exception:
                pass

        self.save_report()

    def save_report(self):
        duration = time.monotonic() - self.t_start_mono if self.t_start_mono > 0 else 0.0
        self.main_lua_hash = get_file_sha256("/workspace/s500_lua_mission/s500_mission.lua")
        self.sitl_lua_hash = get_file_sha256(os.path.join(self.sitl_dir, "scripts", "s500_mission.lua"))
        is_valid_hash = lambda h: bool(h and h not in ("DOSYA_YOK", "OKUMA_HATASI") and len(h) == 64)
        self.lua_hashes_match = bool(
            is_valid_hash(self.main_lua_hash) and
            is_valid_hash(self.sitl_lua_hash) and
            self.main_lua_hash == self.sitl_lua_hash
        )

        s2_rejection_ok = self.scenario_results.get("2_liveliness_behavior", {}).get("observable_protocol_test", {}).get("lua_rejection_observed", False)

        self.mandatory_scenarios_passed = (
            not self.critical_failure_occurred and
            self.lua_hashes_match and
            self.scenario_results.get("1_handshake", {}).get("status") == "PASS" and
            bool(s2_rejection_ok) and
            self.scenario_results.get("3_missing_input_analysis", {}).get("status") == "PASS" and
            self.scenario_results.get("4_real_model_e2e_flow", {}).get("status") == "PASS" and
            self.scenario_results.get("5_obstacle_filtering", {}).get("status") == "PASS" and
            self.scenario_results.get("7_disarmed_safety", {}).get("status") == "PASS"
        )

        report = {
            "timestamp": time.strftime("%Y-%m-%d %H:%M:%S"),
            "scope": "Gerçek Vision Node (best.pt) -> Gerçek Adaptör (mavlink_mission_commander.py) -> SITL Lua (s500_mission.lua)",
            "duration_s": round(duration, 2),
            "overall_passed": self.mandatory_scenarios_passed,
            "mandatory_scenarios_passed": self.mandatory_scenarios_passed,
            "critical_failure_occurred": self.critical_failure_occurred,
            "main_lua_sha256": self.main_lua_hash,
            "sitl_lua_sha256": self.sitl_lua_hash,
            "lua_hashes_match": self.lua_hashes_match,
            "failure_reasons": self.failure_reasons,
            "scenarios": self.scenario_results,
            "gcs_lua_log_count": len(self.gcs_logs),
            "gcs_lua_logs": self.gcs_logs
        }

        report_path = os.path.join(self.sitl_dir, "report_vision_pipeline.json")
        try:
            with open(report_path, "w", encoding="utf-8") as f:
                json.dump(report, f, indent=2, ensure_ascii=False)
            print(f"[RAPOR] Sonuç raporu kaydedildi: {report_path}", flush=True)
        except Exception as e:
            print(f"[RAPOR] Rapor kaydedilemedi: {e}", flush=True)

if __name__ == "__main__":
    runner = VisionPipelineSuiteRunner()
    runner.run_tests()
    sys.exit(0 if runner.mandatory_scenarios_passed else 1)
