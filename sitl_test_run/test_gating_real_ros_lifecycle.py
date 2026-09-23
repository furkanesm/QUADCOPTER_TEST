#!/usr/bin/env python3
"""
test_gating_real_ros_lifecycle.py
---------------------------------
Sentetik FSM Mesajlarıyla Gerçek ROS 2/DDS Gating ve Yaşam Döngüsü Entegrasyon Testi (DISARMED)

TEST KAPSAMI VE MİMARİSİ:
1. SITL (ArduCopter) arka planda başlatılır ve test boyunca kesintisiz DISARMED kalır (kollar açılmaz, uçuş yapılmaz).
2. Otopilot Heartbeat sinyalleri bağımsız bir izleme iş parçacığı tarafından model yükleme ve tüm koşullu
   beklemeler dahil olmak üzere kesintisiz denetlenir; ARMED gözlemi veya 3.0s heartbeat kaybı testi kalıcı olarak sonlandırır.
   Son doğrulamada aynı soket eşzamanlı okunmaz; izleyicinin sayaç artışı beklenir.
3. ROS 2 düğümleri `vision_node:enable_gating:=true` parametresiyle MultiThreadedExecutor üzerinde gerçek DDS ile çalışır.
4. TestableVisionNode alt sınıfı üzerinden create_subscription bağlı yöntemi sağlam şekilde sarmalanır;
   kare geliş ve tamamlanma zaman damgaları ayrıştırılır, callback hataları ana teste iletilir ve thread-safe aktif inference takibi yapılır.
5. wait_for_inference_quiescence kilidi tutarken sleep yapmaz; kilit altında ilk kontrol, kilit bırakılarak bekleme,
   ve kilidi yeniden alarak teyit sonrası güncel sayacı döndürür.
6. Kapatma (DONUS, INIS, TAMAMLANDI) sonrasında yeni karelerin vision_node kamera geri çağrısına ulaşıp tamamlandığı,
   ancak model inference sayacının artmadığı (inference çağrılarının durduğu) ve yeni tespit yayını olmadığı kanıtlanır.
7. Açılma (HEDEF_BEKLE) fazlarında tek seferlik sabit sınır kullanılır, seçilen aynı mesajın detections alanı kontrol edilir
   ve kilit altında sayaç artışı doğrulanır.
"""

import os
import sys
import time
import cv2
import numpy as np
import subprocess
import threading
from typing import Optional, List, Tuple
from pymavlink import mavutil

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, String
from geometry_msgs.msg import PoseStamped
from vision_interfaces.msg import DetectionArray
from cv_bridge import CvBridge

# Companion modülleri
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "../companion")))
from mavlink_mission_commander import MavlinkAdapterNode

# Vision node
from vision_processing.vision_node import VisionNode


class TestableVisionNode(VisionNode):
    """
    Üretim kodunu değiştirmeden, abonelik bağlı yöntemini (bound method)
    sağlam şekilde geçersiz kılan (override) ve thread-safe sayaç tutan test düğümü.
    """
    def __init__(self):
        self.frame_received_count = 0
        self.frame_completed_count = 0
        self.last_completed_stamp_sec = 0
        self.last_completed_stamp_nanosec = 0
        self.callback_error: Optional[str] = None

        self.inference_lock = threading.Lock()
        self.active_inferences = 0
        self.inference_call_count = 0

        super().__init__()

        # Gerçek model çağrısını koruyarak aktif işlem ve toplam çağrı sayacı
        orig_predict = self.model.predict

        def counted_predict(*args, **kwargs):
            with self.inference_lock:
                self.active_inferences += 1
                self.inference_call_count += 1
            try:
                return orig_predict(*args, **kwargs)
            finally:
                with self.inference_lock:
                    self.active_inferences -= 1

        self.model.predict = counted_predict

    def image_callback(self, msg: Image):
        self.frame_received_count += 1
        try:
            super().image_callback(msg)
            # Tamamlanma damgası YALNIZCA super() başarıyla döndükten sonra güncellenir
            self.frame_completed_count += 1
            self.last_completed_stamp_sec = msg.header.stamp.sec
            self.last_completed_stamp_nanosec = msg.header.stamp.nanosec
        except Exception as e:
            self.callback_error = f"image_callback hatası: {e}"
            raise


class GatingLifecycleFeeder(Node):
    """Görüntü ve poz besleyen, /vision/enable ve tespitleri toplayan test düğümü."""
    def __init__(self):
        super().__init__('gating_lifecycle_feeder')
        self.bridge = CvBridge()
        self.pub_image = self.create_publisher(Image, '/camera/image_raw', qos_profile_sensor_data)
        self.pub_pose = self.create_publisher(PoseStamped, '/mavros/local_position/pose', qos_profile_sensor_data)

        self.sub_detections = self.create_subscription(DetectionArray, '/vision/detections', self._det_cb, 10)
        self.sub_enable = self.create_subscription(Bool, '/vision/enable', self._enable_cb, 10)
        self.sub_events = self.create_subscription(String, '/adapter/event_status', self._event_cb, 10)

        self.received_detections: List[Tuple[float, DetectionArray]] = []
        self.received_enable_msgs: List[Tuple[float, bool]] = []
        self.received_events: List[Tuple[float, str]] = []

    def _det_cb(self, msg: DetectionArray):
        self.received_detections.append((time.monotonic(), msg))

    def _enable_cb(self, msg: Bool):
        now = time.monotonic()
        print(f"  [ROS2 SUB] /vision/enable mesajı alındı: {msg.data} (ts={now:.3f})", flush=True)
        self.received_enable_msgs.append((now, msg.data))

    def _event_cb(self, msg: String):
        now = time.monotonic()
        text = msg.data.strip()
        print(f"  [ROS2 EVENT] /adapter/event_status: '{text}' (ts={now:.3f})", flush=True)
        self.received_events.append((now, text))

    def publish_frame(self, cv_img: np.ndarray, z: float = 33.0, stamp_msg=None) -> Tuple[int, int]:
        now_msg = stamp_msg or self.get_clock().now().to_msg()
        p_msg = PoseStamped()
        p_msg.header.stamp = now_msg
        p_msg.header.frame_id = 'odom'
        p_msg.pose.position.x = 0.0
        p_msg.pose.position.y = 0.0
        p_msg.pose.position.z = z
        p_msg.pose.orientation.x = 0.0
        p_msg.pose.orientation.y = 0.0
        p_msg.pose.orientation.z = 0.0
        p_msg.pose.orientation.w = 1.0
        self.pub_pose.publish(p_msg)

        img_msg = Image()
        img_msg.header.stamp = now_msg
        img_msg.header.frame_id = 'camera_link'
        img_msg.height = int(cv_img.shape[0])
        img_msg.width = int(cv_img.shape[1])
        img_msg.encoding = 'bgr8'
        img_msg.is_bigendian = 0
        img_msg.step = int(cv_img.shape[1] * 3)
        img_msg.data = cv_img.tobytes()
        self.pub_image.publish(img_msg)
        return (now_msg.sec, now_msg.nanosec)

    def wait_for_enable_state(self, expected_state: bool, min_ts: float, timeout_s: float = 3.0) -> bool:
        """Belirtilen zaman damgasından (min_ts) sonra gelen /vision/enable durumunu koşullu bekler."""
        t0 = time.monotonic()
        while (time.monotonic() - t0) < timeout_s:
            for ts, val in self.received_enable_msgs:
                if ts >= min_ts and val == expected_state:
                    return True
            time.sleep(0.02)
        return False


class GatingLifecycleTester:
    def __init__(self):
        self.sitl_dir = os.path.dirname(os.path.abspath(__file__))
        self.sitl_proc = None
        self.sitl_log_file = None
        self.mav_gcs = None

        self.last_heartbeat_mono = 0.0
        self.heartbeat_count = 0
        self.is_armed = False
        self.critical_error: Optional[str] = None
        self.running = True

        self.hb_thread: Optional[threading.Thread] = None
        self.executor: Optional[MultiThreadedExecutor] = None
        self.ros_thread: Optional[threading.Thread] = None

        self.adapter_node: Optional[MavlinkAdapterNode] = None
        self.vision_node: Optional[TestableVisionNode] = None
        self.feeder_node: Optional[GatingLifecycleFeeder] = None

    def start_sitl(self):
        log_name = f"sitl_stdout_gating_{int(time.time())}.log"
        log_path = os.path.join(self.sitl_dir, log_name)
        self.sitl_log_file = open(log_path, "a")
        cmd = [
            "/opt/ardupilot/build/sitl/bin/arducopter",
            "--model", "quad",
            "--home", "-35.363261,149.165230,584,353",
            "--defaults", f"/opt/ardupilot/Tools/autotest/default_params/copter.parm,{self.sitl_dir}/extra_params.parm"
        ]
        print(f"[SITL] Süreç başlatılıyor (DISARMED): {' '.join(cmd)}", flush=True)
        print(f"[SITL] Çıktı yönlendiriliyor: {log_path}", flush=True)
        self.sitl_proc = subprocess.Popen(
            cmd,
            cwd=self.sitl_dir,
            stdout=self.sitl_log_file,
            stderr=subprocess.STDOUT,
            preexec_fn=os.setsid
        )

        # İlk GCS bağlantısı (5760)
        t0 = time.monotonic()
        temp_mav = None
        while (time.monotonic() - t0) < 25.0:
            try:
                temp_mav = mavutil.mavlink_connection("tcp:127.0.0.1:5760")
                hb = temp_mav.wait_heartbeat(timeout=1.0)
                if hb and hb.get_srcSystem() == 1 and hb.get_srcComponent() == 1:
                    armed = bool(hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                    if armed:
                        raise RuntimeError(f"GÜVENLİK İHLALİ: Otopilot başlangıçta beklenmedik şekilde ARMED görüldü! (base_mode=0x{hb.base_mode:X})")
                    self.mav_gcs = temp_mav
                    self.last_heartbeat_mono = time.monotonic()
                    self.heartbeat_count = 1
                    self.is_armed = False
                    print("[SITL] İlk Heartbeat teyit edildi (is_armed=False).", flush=True)
                    break
                else:
                    if temp_mav:
                        temp_mav.close()
                        temp_mav = None
            except Exception as e:
                if "GÜVENLİK İHLALİ" in str(e):
                    raise
                if temp_mav:
                    temp_mav.close()
                    temp_mav = None
                time.sleep(0.5)

        if not self.mav_gcs:
            raise RuntimeError("SITL başlatılamadı veya 25s içinde GCS Heartbeat alınamadı.")

        # Kesintisiz tek Heartbeat izleme iş parçacığını başlat
        self.hb_thread = threading.Thread(target=self._heartbeat_monitor_loop, daemon=True)
        self.hb_thread.start()

    def _heartbeat_monitor_loop(self):
        """Test boyunca tek iş parçacığı üzerinden Heartbeat okur ve tazeliği/DISARMED durumunu izler."""
        while self.running and self.critical_error is None:
            try:
                msg = self.mav_gcs.recv_match(type='HEARTBEAT', blocking=True, timeout=0.5)
                if msg and msg.get_srcSystem() == 1 and msg.get_srcComponent() == 1:
                    now = time.monotonic()
                    self.last_heartbeat_mono = now
                    self.heartbeat_count += 1
                    armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                    if armed:
                        self.critical_error = f"GÜVENLİK İHLALİ: Araç test sırasında ARMED durumuna geçti! (base_mode=0x{msg.base_mode:X})"
                        self.is_armed = True
                        print(f"\n[GÜVENLİK ALARMI] {self.critical_error}", flush=True)
                        break
                    self.is_armed = False

                if self.last_heartbeat_mono > 0:
                    age = time.monotonic() - self.last_heartbeat_mono
                    if age > 3.0:
                        self.critical_error = f"Otopilot Heartbeat zaman aşımı! ({age:.2f}s > 3.0s)"
                        print(f"\n[GÜVENLİK ALARMI] {self.critical_error}", flush=True)
                        break
            except Exception as e:
                if self.running:
                    self.critical_error = f"Heartbeat izleme iş parçacığı hatası: {e}"
                    break

    def assert_healthy(self):
        """İzleme iş parçacığındaki ve vision callback'indeki herhangi bir kritik hatayı kontrol eder."""
        if self.critical_error is not None:
            raise RuntimeError(self.critical_error)
        if self.vision_node and self.vision_node.callback_error is not None:
            raise RuntimeError(self.vision_node.callback_error)

    def start_ros(self):
        print("[ROS2] Düğümler enable_gating:=true parametresiyle başlatılıyor...", flush=True)
        rclpy.init(args=[
            "--ros-args",
            "-p", "mavlink_mission_commander:mavlink_connection:=tcp:127.0.0.1:5762",
            "-p", "vision_node:model_path:=/workspace/best.pt",
            "-p", "vision_node:enable_gating:=true",
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
            "-p", "vision_node:mount_rpy:=[0.0, 1.5707963, 0.0]",
            "-p", "vision_node:ground_plane_z:=0.0",
            "-p", "vision_node:pose_age_tolerance_s:=0.5"
        ])

        self.adapter_node = MavlinkAdapterNode()
        self.vision_node = TestableVisionNode()
        self.feeder_node = GatingLifecycleFeeder()

        self.executor = MultiThreadedExecutor()
        self.executor.add_node(self.adapter_node)
        self.executor.add_node(self.vision_node)
        self.executor.add_node(self.feeder_node)

        self.ros_thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.ros_thread.start()
        print("[ROS2] MultiThreadedExecutor üzerinde düğümler aktif olarak çalışıyor.", flush=True)

    def wait_for_inference_quiescence(self, timeout_s: float = 3.0) -> int:
        """Devam eden aktif inference işlemi kalmayana kadar bekler; kilidi tutarken sleep yapmaz."""
        t0 = time.monotonic()
        while (time.monotonic() - t0) < timeout_s:
            self.assert_healthy()
            with self.vision_node.inference_lock:
                first_active = self.vision_node.active_inferences

            if first_active == 0:
                time.sleep(0.05)
                with self.vision_node.inference_lock:
                    if self.vision_node.active_inferences == 0:
                        return self.vision_node.inference_call_count

            time.sleep(0.02)

        with self.vision_node.inference_lock:
            active = self.vision_node.active_inferences
        raise TimeoutError(f"İnference durulma zaman aşımı ({timeout_s}s)! Aktif işlem sayısı: {active}")

    def verify_closed_phase_behavior(self, phase_name: str, test_img: np.ndarray):
        """Kapanma (DONUS, INIS, TAMAMLANDI) sonrası kare teslimi ve inference sıfır artış doğrulayıcısı."""
        assert self.vision_node.vision_enabled is False, f"{phase_name} fazında vision_enabled False olmalı!"
        quiescent_inf = self.wait_for_inference_quiescence(timeout_s=3.0)
        baseline_frames = self.vision_node.frame_completed_count
        baseline_dets = len(self.feeder_node.received_detections)

        # Kapanma sonrası belirgin zaman damgasıyla 10 yeni kare gönder
        last_stamp = (0, 0)
        for _ in range(10):
            last_stamp = self.feeder_node.publish_frame(test_img)
            time.sleep(0.04)

        # Gönderilen son karenin callback'te BAŞARIYLA TAMAMLANDIĞINI süre sınırıyla bekle
        t_wait_frame = time.monotonic()
        frame_delivered = False
        while (time.monotonic() - t_wait_frame) < 3.0:
            self.assert_healthy()
            if (self.vision_node.last_completed_stamp_sec == last_stamp[0] and
                self.vision_node.last_completed_stamp_nanosec == last_stamp[1]):
                frame_delivered = True
                break
            time.sleep(0.02)

        assert frame_delivered, f"{phase_name} fazında son karenin callback tamamlanması zaman aşımına uğradı!"

        # Tespit mesajı teslimatı için sınırlı gözlem süresi (bounded observation window)
        time.sleep(0.3)
        self.assert_healthy()

        frames_delta = self.vision_node.frame_completed_count - baseline_frames
        inf_delta = self.vision_node.inference_call_count - quiescent_inf
        det_delta = len(self.feeder_node.received_detections) - baseline_dets

        assert frames_delta >= 10, f"{phase_name} fazında en az 10 kare tamamlanmalıydı (Tamamlanan: {frames_delta})"
        assert inf_delta == 0, f"{phase_name} fazında yeni karelere rağmen model.predict çağrıldı! (Çağrı artışı: {inf_delta})"
        assert det_delta == 0, f"{phase_name} fazında yeni karelere rağmen DetectionArray yayımlandı! (Yayımlanan: {det_delta})"
        print(f"✓ {phase_name} BAŞARILI: {frames_delta} yeni karenin callback'te tamamlandığı teyit edildi; inference çağrıları durdu (Sayaç artışı: 0, Tespit artışı: 0).")

    def verify_open_phase_behavior(self, phase_name: str, test_img: np.ndarray):
        """Açılma (HEDEF_BEKLE) sonrası sabit sınır ve asenkron tespit doğrulaması."""
        open_start_mono = time.monotonic()
        with self.vision_node.inference_lock:
            inf_before = self.vision_node.inference_call_count

        matching_det = None
        t_det_wait = time.monotonic()
        while (time.monotonic() - t_det_wait) < 4.0:
            self.feeder_node.publish_frame(test_img)
            self.assert_healthy()

            # Sabit open_start_mono sonrası gelen ve detections listesi boş olmayan ilk mesajı bul
            for rec_mono, det_msg in self.feeder_node.received_detections:
                if rec_mono >= open_start_mono and len(det_msg.detections) > 0:
                    matching_det = det_msg
                    break

            with self.vision_node.inference_lock:
                current_inf = self.vision_node.inference_call_count

            if matching_det is not None and current_inf > inf_before:
                break
            time.sleep(0.08)

        # Doğrulamalar: Aynı matching_det nesnesi ve kilit altındaki güncel sayaç
        assert matching_det is not None, f"{phase_name}: Açılma sonrası tespit mesajı alınamadı!"
        assert len(matching_det.detections) > 0, f"{phase_name}: Seçilen DetectionArray mesajının detections alanı boş!"
        det_class = matching_det.detections[0].class_name
        assert det_class == "Hedef", f"{phase_name}: Beklenen 'Hedef' yerine '{det_class}' tespit edildi!"

        with self.vision_node.inference_lock:
            final_inf = self.vision_node.inference_call_count
        assert final_inf > inf_before, f"{phase_name}: Açılma sonrası inference çağrı sayısı artmadı!"

        print(f"✓ {phase_name} BAŞARILI: Açılma sonrasında inference sayacı arttı ve boş olmayan hedef tespiti alındı (Sınıf: {det_class}, Conf: {matching_det.detections[0].confidence:.2f}).")

    def run_lifecycle_test(self):
        img_path = "/workspace/companion/Hedef_0002.jpg"
        assert os.path.exists(img_path), f"Test görüntüsü bulunamadı: {img_path}"
        test_img = cv2.imread(img_path)

        # -----------------------------------------------------------------
        # FAZ 1: Warm Standby (enable_gating=True Başlangıç Durumu)
        # -----------------------------------------------------------------
        print("\n--- FAZ 1: Warm Standby Başlangıcı (Gating Devrede) ---", flush=True)
        self.assert_healthy()
        time.sleep(0.5)
        assert self.vision_node.enable_gating is True, "enable_gating parametresi True olmalı!"
        assert self.vision_node.vision_enabled is False, "Başlangıçta vision_enabled False olmalı!"

        self.verify_closed_phase_behavior("FAZ 1 (Warm Standby)", test_img)

        # -----------------------------------------------------------------
        # FAZ 2: HEDEF_BEKLE Geçişi -> Açılma ve Tespit Başlangıcı
        # -----------------------------------------------------------------
        print("\n--- FAZ 2: HEDEF_BEKLE Tetiklemesi ile Açılma Doğrulaması ---", flush=True)
        self.adapter_node.session_state = self.adapter_node.ACTIVE
        t_phase2 = time.monotonic()

        log_bekle = type("MockMsg", (), {
            "get_srcSystem": lambda self: 1,
            "get_srcComponent": lambda self: 1,
            "severity": 6, "id": 0, "chunk_seq": 0,
            "text": "[S500 LUA] Durum Gecisi: ILERI_HAREKET -> HEDEF_BEKLE"
        })()
        self.adapter_node._handle_statustext_msg(log_bekle)

        ok = self.feeder_node.wait_for_enable_state(True, min_ts=t_phase2, timeout_s=3.0)
        assert ok, "Zaman aşımı: HEDEF_BEKLE sonrası /vision/enable: True mesajı alınamadı!"

        t_vn_wait = time.monotonic()
        while (time.monotonic() - t_vn_wait) < 2.0:
            self.assert_healthy()
            if self.vision_node.vision_enabled:
                break
            time.sleep(0.02)
        assert self.vision_node.vision_enabled is True, "vision_node.vision_enabled True olmadı!"

        self.verify_open_phase_behavior("FAZ 2 (HEDEF_BEKLE Açılma)", test_img)

        # -----------------------------------------------------------------
        # FAZ 3: DONUS Durumunda Kapanma
        # -----------------------------------------------------------------
        print("\n--- FAZ 3: DONUS Durumunda Kapanma Doğrulaması ---", flush=True)
        t_phase3 = time.monotonic()
        log_donus = type("MockMsg", (), {
            "get_srcSystem": lambda self: 1,
            "get_srcComponent": lambda self: 1,
            "severity": 6, "id": 0, "chunk_seq": 0,
            "text": "[S500 LUA] Durum Gecisi: HEDEFE_GIT -> DONUS"
        })()
        self.adapter_node._handle_statustext_msg(log_donus)

        ok = self.feeder_node.wait_for_enable_state(False, min_ts=t_phase3, timeout_s=3.0)
        assert ok, "Zaman aşımı: DONUS sonrası /vision/enable: False mesajı alınamadı!"

        t_vn_wait = time.monotonic()
        while (time.monotonic() - t_vn_wait) < 2.0:
            self.assert_healthy()
            if not self.vision_node.vision_enabled:
                break
            time.sleep(0.02)
        assert self.vision_node.vision_enabled is False, "vision_node.vision_enabled False olmadı!"

        self.verify_closed_phase_behavior("FAZ 3 (DONUS)", test_img)

        # -----------------------------------------------------------------
        # FAZ 4: Yeniden Açılma -> INIS ve TAMAMLANDI Kapanmaları
        # -----------------------------------------------------------------
        print("\n--- FAZ 4: Yeniden Açılma, INIS ve TAMAMLANDI Doğrulaması ---", flush=True)

        # 4a: Yeniden Açılma (HEDEF_BEKLE) ve Tespit Üretimi Teyidi
        t_p4_re1 = time.monotonic()
        self.adapter_node._handle_statustext_msg(log_bekle)
        ok = self.feeder_node.wait_for_enable_state(True, min_ts=t_p4_re1, timeout_s=3.0)
        assert ok, "Zaman aşımı: Yeniden açılmada /vision/enable: True alınamadı!"

        t_vn_wait = time.monotonic()
        while (time.monotonic() - t_vn_wait) < 2.0:
            self.assert_healthy()
            if self.vision_node.vision_enabled:
                break
            time.sleep(0.02)
        assert self.vision_node.vision_enabled is True, "Yeniden açılmada vision_enabled True olmadı!"

        self.verify_open_phase_behavior("FAZ 4a (Yeniden Açılma)", test_img)

        # 4b: INIS Geçişi ve Kapanma
        t_p4_inis = time.monotonic()
        log_inis = type("MockMsg", (), {
            "get_srcSystem": lambda self: 1,
            "get_srcComponent": lambda self: 1,
            "severity": 6, "id": 0, "chunk_seq": 0,
            "text": "[S500 LUA] Durum Gecisi: DONUS -> INIS"
        })()
        self.adapter_node._handle_statustext_msg(log_inis)
        ok = self.feeder_node.wait_for_enable_state(False, min_ts=t_p4_inis, timeout_s=3.0)
        assert ok, "Zaman aşımı: INIS sonrası /vision/enable: False alınamadı!"

        t_vn_wait = time.monotonic()
        while (time.monotonic() - t_vn_wait) < 2.0:
            self.assert_healthy()
            if not self.vision_node.vision_enabled:
                break
            time.sleep(0.02)
        assert self.vision_node.vision_enabled is False, "vision_node.vision_enabled False olmadı!"

        self.verify_closed_phase_behavior("FAZ 4b (INIS)", test_img)

        # 4c: İkinci Yeniden Açılma ve TAMAMLANDI Kapanması
        t_p4_re2 = time.monotonic()
        self.adapter_node._handle_statustext_msg(log_bekle)
        ok = self.feeder_node.wait_for_enable_state(True, min_ts=t_p4_re2, timeout_s=3.0)
        assert ok, "Zaman aşımı: İkinci yeniden açılmada /vision/enable: True alınamadı!"

        t_vn_wait = time.monotonic()
        while (time.monotonic() - t_vn_wait) < 2.0:
            self.assert_healthy()
            if self.vision_node.vision_enabled:
                break
            time.sleep(0.02)
        assert self.vision_node.vision_enabled is True, "İkinci yeniden açılmada vision_enabled True olmadı!"

        self.verify_open_phase_behavior("FAZ 4c (İkinci Yeniden Açılma)", test_img)

        t_p4_tamam = time.monotonic()
        log_tamam = type("MockMsg", (), {
            "get_srcSystem": lambda self: 1,
            "get_srcComponent": lambda self: 1,
            "severity": 6, "id": 0, "chunk_seq": 0,
            "text": "[S500 LUA] Durum Gecisi: INIS -> TAMAMLANDI"
        })()
        self.adapter_node._handle_statustext_msg(log_tamam)
        ok = self.feeder_node.wait_for_enable_state(False, min_ts=t_p4_tamam, timeout_s=3.0)
        assert ok, "Zaman aşımı: TAMAMLANDI sonrası /vision/enable: False alınamadı!"

        t_vn_wait = time.monotonic()
        while (time.monotonic() - t_vn_wait) < 2.0:
            self.assert_healthy()
            if not self.vision_node.vision_enabled:
                break
            time.sleep(0.02)
        assert self.vision_node.vision_enabled is False, "vision_node.vision_enabled False olmadı!"

        self.verify_closed_phase_behavior("FAZ 4c (TAMAMLANDI)", test_img)

        # -----------------------------------------------------------------
        # FAZ 5: Son Güvenlik ve Taze Heartbeat Doğrulaması
        # -----------------------------------------------------------------
        print("\n--- FAZ 5: Kesintisiz DISARMED ve Taze Heartbeat Doğrulaması ---", flush=True)
        baseline_hb_count = self.heartbeat_count
        t_end = time.monotonic()
        got_fresh_hb = False
        while (time.monotonic() - t_end) < 3.0:
            self.assert_healthy()
            if self.heartbeat_count > baseline_hb_count and (time.monotonic() - self.last_heartbeat_mono) <= 1.0:
                got_fresh_hb = True
                break
            time.sleep(0.05)

        self.assert_healthy()
        assert got_fresh_hb, f"Zaman aşımı: Son aşamada izleyiciden taze heartbeat alınamadı! (Mevcut: {self.heartbeat_count}, Başlangıç: {baseline_hb_count})"
        assert self.is_armed is False, "HATA: Test sonunda araç DISARMED değil!"
        print(f"✓ FAZ 5 BAŞARILI: Test süresi boyunca alınan {self.heartbeat_count} adet MAVLink Heartbeat mesajında base_mode SAFETY_ARMED biti 0 olarak ölçülmüştür; araç DISARMED durumda kalmıştır.")

    def cleanup(self):
        print("\n[TEMİZLİK] Kaynaklar kapatılıyor...", flush=True)
        self.running = False
        if self.hb_thread and self.hb_thread.is_alive():
            self.hb_thread.join(timeout=2.0)
        if self.mav_gcs:
            try:
                self.mav_gcs.close()
            except Exception:
                pass
        if self.executor:
            try:
                self.executor.shutdown()
            except Exception:
                pass
        if self.ros_thread and self.ros_thread.is_alive():
            self.ros_thread.join(timeout=3.0)
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
        if rclpy.ok():
            try:
                rclpy.shutdown()
            except Exception:
                pass
        if self.sitl_proc:
            try:
                os.killpg(os.getpgid(self.sitl_proc.pid), subprocess.signal.SIGTERM)
                self.sitl_proc.wait(timeout=5.0)
            except Exception:
                pass
        if self.sitl_log_file:
            try:
                self.sitl_log_file.close()
            except Exception:
                pass
        print("[TEMİZLİK] Tamamlandı.", flush=True)


def main():
    print("=" * 80)
    print("AŞAMA 3: GERÇEK ROS DDS GATING & YAŞAM DÖNGÜSÜ ENTEGRASYON TESTİ (DISARMED)")
    print("=" * 80)

    tester = GatingLifecycleTester()
    try:
        tester.start_sitl()
        tester.start_ros()
        tester.run_lifecycle_test()
        print("\n" + "=" * 80)
        print(">>> TÜM FAZLAR (1..5) EKSİKSİZ BAŞARIYLA GEÇTİ (DISARMED, Sıfır Regresyon) <<<")
        print("=" * 80)
        sys.exit(0)
    except Exception as e:
        print(f"\n[TEST BAŞARISIZ]: {e}", flush=True)
        import traceback
        traceback.print_exc()
        sys.exit(1)
    finally:
        tester.cleanup()


if __name__ == '__main__':
    main()
