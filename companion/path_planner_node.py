#!/usr/bin/env python3
"""
S500 Rota Planlayıcı Düğümü (PathPlannerNode)
---------------------------------------------
Bu düğüm:
1. /vision/detections (DetectionArray) konusunu dinler.
   - position_valid=True olan 'start', 'hedef' ve 'engel' tespitlerini kaydeder.
   - Start konumunu ilk geçerli tespitte kilitler (jitter & drift koruması).
   - position_valid=False veya geçersiz (NaN/Inf) verileri kesinlikle filtreler.
2. Dinamik 2D ızgara (Grid) üzerinde GridPlanner (A* + cv2 dilate enflasyon) çalıştırır.
   - S500 quadcopter boyutlarına göre çarpışmasız (collision-free) rota üretir.
3. Rota Çıktıları:
   - YKI (Yer Kontrol İstasyonu) görselleştirmesi için: /planner/path (nav_msgs/Path) yayınlar.
   - Dönüş rotası için: /planner/return_path (nav_msgs/Path) yayınlar.
   - Durum bildirimleri için: /planner/status (std_msgs/String) yayınlar.
4. Status 4 için:
   - Çarpışmasız rota hesaplandığında ve otopilot HEDEF_BEKLE durumundayken,
     aracın o anki taze konumundan kilitli kalkış referansına dönüş rotası yayınlanır
     ve mavlink_mission_commander üzerindeki /mission/route_ready (std_srvs/Trigger)
     servisi çağrılarak Lua FSM'nin DONUS aşamasına doğrudan geçişi tetiklenir.
"""

import os
import sys
import math
import time
from typing import Optional, List, Tuple, Dict, Any
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import String
from std_srvs.srv import Trigger
from vision_interfaces.msg import Detection, DetectionArray

# companion dizinini path'e ekle
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from grid_path_planner import GridPlanner


class PathPlannerNode(Node):
    def __init__(self):
        super().__init__('path_planner_node')

        # ----------------------------------------------------------------------
        # Parametre Tanımları
        # ----------------------------------------------------------------------
        # FAZ B GÜNCELLEMESİ: Drone 33 m uzağa uçtuğu için 20x20'lik Grid (10m ofset) yetersizdi. 
        # Yeni büyüklük: 33 m gidiş + görüş açısı toleransı (~31m) + pay ~ 70m uç.
        # Toplam arena 140x140m (NED x,y = [-70, +70]). (ROS parametreleriyle ezilebilir)
        self.declare_parameter('arena_width', 140.0)        # Metre (X ekseni genişliği: 140m)
        self.declare_parameter('arena_height', 140.0)       # Metre (Y ekseni genişliği: 140m)
        self.declare_parameter('grid_resolution', 0.5)     # Metre / hücre
        self.declare_parameter('vehicle_width', 0.6)       # S500 gövde genişliği (m)
        self.declare_parameter('safety_margin', 0.4)       # Güvenlik marjı (m)
        self.declare_parameter('origin_offset_x', 70.0)    # NED X -> Grid sütun ofseti (m)
        self.declare_parameter('origin_offset_y', 70.0)    # NED Y -> Grid satır ofseti (m)
        self.declare_parameter('strict_boundary_walls', False)  # Varsayılan: Açık hava uçuş sahası (kenarlar sahte duvar sayılmaz)
        self.declare_parameter('auto_trigger_route_ready', True)
        self.declare_parameter('start_classes', ['start'])
        self.declare_parameter('goal_classes', ['hedef', 'goal'])
        self.declare_parameter('obstacle_classes', ['engel', 'obstacle'])
        self.declare_parameter('target_cruise_alt', 33.0)
        self.declare_parameter('vehicle_pos_stale_timeout', 3.0)

        self.arena_w = float(self.get_parameter('arena_width').value)
        self.arena_h = float(self.get_parameter('arena_height').value)
        self.res = float(self.get_parameter('grid_resolution').value)
        self.veh_w = float(self.get_parameter('vehicle_width').value)
        self.safety_m = float(self.get_parameter('safety_margin').value)
        self.offset_x = float(self.get_parameter('origin_offset_x').value)
        self.offset_y = float(self.get_parameter('origin_offset_y').value)
        self.strict_boundary_walls = bool(self.get_parameter('strict_boundary_walls').value)
        self.auto_route_ready = bool(self.get_parameter('auto_trigger_route_ready').value)
        self.target_cruise_alt = float(self.get_parameter('target_cruise_alt').value)
        self.vehicle_pos_stale_timeout = float(self.get_parameter('vehicle_pos_stale_timeout').value)

        self.start_classes = [s.strip().lower() for s in self.get_parameter('start_classes').value]
        self.goal_classes = [s.strip().lower() for s in self.get_parameter('goal_classes').value]
        self.obstacle_classes = [s.strip().lower() for s in self.get_parameter('obstacle_classes').value]

        # ----------------------------------------------------------------------
        # Durum Değişkenleri
        # ----------------------------------------------------------------------
        self.start_pos_ned: Optional[Tuple[float, float]] = None
        self.start_locked: bool = False
        self.goal_pos_ned: Optional[Tuple[float, float]] = None
        self.obstacles_ned: List[Tuple[float, float]] = []

        self.current_session_id: Optional[int] = None
        self.takeoff_return_pos_ned: Optional[Tuple[float, float]] = None
        self.takeoff_return_pos_ned_3d: Optional[Tuple[float, float, float]] = None
        self.vehicle_pos_ned: Optional[Tuple[float, float]] = None
        self.vehicle_pos_ned_3d: Optional[Tuple[float, float, float]] = None
        self.last_vehicle_pos_time: Optional[Time] = None
        self.last_vehicle_pos_mono: Optional[float] = None

        self.last_path_ned: List[Tuple[float, float]] = []
        self.return_path_ned: List[Tuple[float, float]] = []
        self.lua_fsm_state: str = "UNKNOWN"
        self.route_ready_called: bool = False
        self.return_path_locked: bool = False

        # GridPlanner örneği
        self.planner = GridPlanner(
            cell_size=self.res,
            vehicle_width=self.veh_w,
            safety_margin=self.safety_m,
            strict_boundary_walls=self.strict_boundary_walls
        )

        # ----------------------------------------------------------------------
        # ROS 2 Abonelikleri ve Yayıncıları
        # ----------------------------------------------------------------------
        # 1. Görüntü Tespitleri
        self.sub_detections = self.create_subscription(
            DetectionArray,
            '/vision/detections',
            self.detections_callback,
            qos_profile_sensor_data
        )

        # 2. Adaptör Olay / FSM Durumu
        self.sub_adapter_events = self.create_subscription(
            String,
            '/adapter/event_status',
            self.adapter_event_callback,
            10
        )

        # 3. Planlanan Rota Çıktısı (nav_msgs/Path) -> /planner/path (Gidiş / YKI)
        self.pub_path = self.create_publisher(
            Path,
            '/planner/path',
            10
        )

        # 4b. Planlanan Dönüş Rotası Çıktısı (nav_msgs/Path) -> /planner/return_path (Dönüş)
        self.pub_return_path = self.create_publisher(
            Path,
            '/planner/return_path',
            10
        )

        # 5. Planlayıcı Durum Bildirimi
        self.pub_status = self.create_publisher(
            String,
            '/planner/status',
            10
        )

        # 6. Status 4 Servis İstemcisi (/mission/route_ready)
        self.cli_route_ready = self.create_client(
            Trigger,
            '/mission/route_ready'
        )

        # 7. Manuel Tetikleme Servisi (/planner/plan_route)
        self.srv_plan = self.create_service(
            Trigger,
            '/planner/plan_route',
            self.plan_route_srv_cb
        )

        # 8. Yerde Kilitlenen Referans Noktası Aboneliği
        self.sub_takeoff_return = self.create_subscription(
            PoseStamped,
            '/mission/takeoff_return_point',
            self.takeoff_return_cb,
            10
        )

        # 9. Güncel Araç Konumu Aboneliği
        self.sub_vehicle_pose = self.create_subscription(
            PoseStamped,
            '/mission/vehicle_pose',
            self.vehicle_pose_cb,
            10
        )

        self.get_logger().info(
            f"[PLANNER] PathPlannerNode baslatildi (Arena: {self.arena_w}x{self.arena_h}m, "
            f"Res: {self.res}m, VehW: {self.veh_w}m, Safety: {self.safety_m}m)"
        )

    def reset_state(self):
        """Planlayıcı durumunu temizler (yeni oturum veya testler için)."""
        self.start_pos_ned = None
        self.start_locked = False
        self.goal_pos_ned = None
        self.obstacles_ned = []
        self.last_path_ned = []
        self.return_path_ned = []
        self.lua_fsm_state = "UNKNOWN"
        self.route_ready_called = False
        self.return_path_locked = False
        self.return_plan_attempts = 0
        self.last_attempt_mono = 0.0
        self.takeoff_return_pos_ned = None
        self.takeoff_return_pos_ned_3d = None
        self.vehicle_pos_ned = None
        self.vehicle_pos_ned_3d = None
        self.last_vehicle_pos_time = None
        self.last_vehicle_pos_mono = None
        self.diagnostic_logged = False
        self.warned_out_of_bounds_pts = set()
        
        # Teşhis ve uyarı kilitleri
        self.diagnostic_logged = False
        self.warned_out_of_bounds_pts = set()

    def takeoff_return_cb(self, msg: PoseStamped):
        """Yerde kilitlenen fiziksel kalkış/dönüş referans noktasını kaydeder."""
        # 1. Aktif oturum kontrolü
        if self.current_session_id is None or self.current_session_id <= 0:
            self.get_logger().warn("[PLANNER] Reddedildi: Aktif oturum yokken referans alinamazi.")
            return

        # 2. Frame ve oturum eki doğrulaması
        raw_frame = (msg.header.frame_id or "").strip()
        if ":session_" not in raw_frame:
            self.get_logger().warn(f"[PLANNER] Reddedildi: Frame oturum eki icermiyor: '{raw_frame}'")
            return

        parts = raw_frame.split(":session_")
        if len(parts) != 2:
            self.get_logger().warn(f"[PLANNER] Reddedildi: Bozuk frame yapisi: '{raw_frame}'")
            return

        base_frame = parts[0].strip().lower()
        if base_frame != "ekf_origin_ned":
            self.get_logger().warn(f"[PLANNER] Reddedildi: Gecersiz temel frame: '{base_frame}', beklenen: ekf_origin_ned")
            return

        try:
            msg_session_id = int(parts[1].strip())
        except Exception:
            self.get_logger().warn(f"[PLANNER] Reddedildi: Bozuk oturum kimligi: '{parts[1]}'")
            return

        if msg_session_id != self.current_session_id:
            self.get_logger().warn(f"[PLANNER] Reddedildi: Oturum uyusmazligi: {msg_session_id} != {self.current_session_id}")
            return

        # 3. Sonlu koordinat kontrolü
        px = float(msg.pose.position.x)
        py = float(msg.pose.position.y)
        pz = float(msg.pose.position.z)
        if not (math.isfinite(px) and math.isfinite(py) and math.isfinite(pz)):
            self.get_logger().warn(f"[PLANNER] Reddedildi: Sonlu olmayan koordinatlar: [{px}, {py}, {pz}]")
            return

        # 4. Kilitli referansın korunması (aynı koordinat zararsız, farklı koordinat ret)
        if self.takeoff_return_pos_ned_3d is not None:
            cur_x, cur_y, cur_z = self.takeoff_return_pos_ned_3d
            if (math.isclose(px, cur_x, abs_tol=1e-3) and
                math.isclose(py, cur_y, abs_tol=1e-3) and
                math.isclose(pz, cur_z, abs_tol=1e-3)):
                self.get_logger().debug(f"[PLANNER] Ayni referans noktasi tekrar alindi (Session {msg_session_id}), zararsiz gecildi.")
                return
            else:
                self.get_logger().warn(
                    f"[PLANNER] Reddedildi: Kilitli referans degistirilemez: "
                    f"Yeni [{px:.2f}, {py:.2f}, {pz:.2f}] != Kilitli [{cur_x:.2f}, {cur_y:.2f}, {cur_z:.2f}]"
                )
                return

        # Bütün doğrulamalar tamamlandı: SADECE fiziksel dönüş referansı durumunu güncelle
        self.takeoff_return_pos_ned = (px, py)
        self.takeoff_return_pos_ned_3d = (px, py, pz)
        self.get_logger().info(f"[PLANNER] Takeoff return reference locked: NED 3D [{px:.2f}, {py:.2f}, {pz:.2f}] (Session: {msg_session_id})")
        self.pub_status.publish(String(data=f"TAKEOFF_REF_LOCKED:{px:.2f}:{py:.2f}:{pz:.2f}"))

        if self.lua_fsm_state == "HEDEF_BEKLE" and self._is_vehicle_pos_fresh():
            self.check_and_unlock_return_path()

    def _is_vehicle_pos_fresh(self) -> bool:
        """Kayıtlı araç konumunun sonlu ve zaman aşımına uğramamış (taze) olduğunu doğrular."""
        if self.vehicle_pos_ned is None or self.last_vehicle_pos_time is None:
            return False
        now_ros = self.get_clock().now()
        age_s = (now_ros - self.last_vehicle_pos_time).nanoseconds / 1e9
        return 0.0 <= age_s <= self.vehicle_pos_stale_timeout

    def vehicle_pose_cb(self, msg: PoseStamped):
        """
        Güncel araç konumunu (/mission/vehicle_pose) kaydeder.
        Aktif oturum, ekf_origin_ned frame'i, mesajdaki session kimliği ve
        konum güncelliği doğrulanmadan hiçbir durum güncellenmez.
        """
        # 1. Aktif oturum kontrolü
        if self.current_session_id is None or self.current_session_id <= 0:
            self.get_logger().warn("[PLANNER] Reddedildi (/mission/vehicle_pose): Aktif oturum yok.")
            return

        # 2. Frame ve oturum eki doğrulaması
        raw_frame = (msg.header.frame_id or "").strip()
        if ":session_" not in raw_frame:
            self.get_logger().warn(f"[PLANNER] Reddedildi (/mission/vehicle_pose): Frame oturum eki icermiyor: '{raw_frame}'")
            return

        parts = raw_frame.split(":session_")
        if len(parts) != 2:
            self.get_logger().warn(f"[PLANNER] Reddedildi (/mission/vehicle_pose): Bozuk frame yapisi: '{raw_frame}'")
            return

        base_frame = parts[0].strip().lower()
        if base_frame != "ekf_origin_ned":
            self.get_logger().warn(f"[PLANNER] Reddedildi (/mission/vehicle_pose): Gecersiz temel frame: '{base_frame}', beklenen: ekf_origin_ned")
            return

        try:
            msg_session_id = int(parts[1].strip())
        except Exception:
            self.get_logger().warn(f"[PLANNER] Reddedildi (/mission/vehicle_pose): Bozuk oturum kimligi: '{parts[1]}'")
            return

        if msg_session_id != self.current_session_id:
            self.get_logger().warn(f"[PLANNER] Reddedildi (/mission/vehicle_pose): Oturum uyusmazligi: {msg_session_id} != {self.current_session_id}")
            return

        # 3. Sonlu koordinat kontrolü
        px = float(msg.pose.position.x)
        py = float(msg.pose.position.y)
        pz = float(msg.pose.position.z)
        if not (math.isfinite(px) and math.isfinite(py) and math.isfinite(pz)):
            self.get_logger().warn(f"[PLANNER] Reddedildi (/mission/vehicle_pose): Sonlu olmayan koordinatlar: [{px}, {py}, {pz}]")
            return

        # 4. Konum güncelliği (staleness check)
        now_ros = self.get_clock().now()
        if msg.header.stamp.sec == 0 and msg.header.stamp.nanosec == 0:
            self.get_logger().warn("[PLANNER] Reddedildi (/mission/vehicle_pose): Bos zaman damgasi.")
            return

        msg_time = Time.from_msg(msg.header.stamp, clock_type=now_ros.clock_type)
        dt_source = (now_ros - msg_time).nanoseconds / 1e9
        if dt_source < 0.0 or dt_source > self.vehicle_pos_stale_timeout:
            self.get_logger().warn(f"[PLANNER] Reddedildi (/mission/vehicle_pose): Gecikmis/eskis veri ({dt_source:.2f}s > {self.vehicle_pos_stale_timeout}s)")
            return

        # Bütün doğrulamalar tamamlandı: Durum değişkenlerini ve kabul zamanını güncelle
        self.vehicle_pos_ned = (px, py)
        self.vehicle_pos_ned_3d = (px, py, pz)
        self.last_vehicle_pos_time = msg_time
        self.last_vehicle_pos_mono = time.monotonic()

        # Doğrulanmış konumla dönüş rotası açılma denetimi:
        # HEDEF_BEKLE durumunda taze konum ve kilitli kalkış referansı varsa dönüş rotasını kontrol et/aç
        if self.lua_fsm_state == "HEDEF_BEKLE" and self.takeoff_return_pos_ned is not None:
            self.check_and_unlock_return_path()

    # --------------------------------------------------------------------------
    # Koordinat Dönüşüm Yardımcıları (NED <-> Grid)
    # --------------------------------------------------------------------------
    def ned_to_grid(self, x_ned: float, y_ned: float) -> Optional[Tuple[int, int]]:
        """
        Metrik NED koordinatını ızgara (gx, gy) hücre indislerine dönüştürür.
        gx: Sütun (X ekseni)
        gy: Satır (Y ekseni)
        """
        gx = int(math.floor((x_ned + self.offset_x) / self.res))
        gy = int(math.floor((y_ned + self.offset_y) / self.res))

        grid_w = int(math.floor(self.arena_w / self.res))
        grid_h = int(math.floor(self.arena_h / self.res))

        if 0 <= gx < grid_w and 0 <= gy < grid_h:
            return (gx, gy)
            
        pt_key = (round(x_ned, 1), round(y_ned, 1))
        if pt_key not in self.warned_out_of_bounds_pts:
            self.get_logger().warn(f"[PLANNER] NED coordinati sınır disinda kaldi: {x_ned:.2f}, {y_ned:.2f} (Hesaplanilan hucre: {gx}, {gy}, Max hucre: {grid_w-1}, {grid_h-1})")
            self.warned_out_of_bounds_pts.add(pt_key)
        
        return None

    def grid_to_ned(self, gx: int, gy: int) -> Tuple[float, float]:
        """Izgara hücresi merkezini metrik NED koordinatına dönüştürür."""
        x_ned = (gx + 0.5) * self.res - self.offset_x
        y_ned = (gy + 0.5) * self.res - self.offset_y
        return (round(x_ned, 3), round(y_ned, 3))

    # --------------------------------------------------------------------------
    # Girdi Callback'leri
    # --------------------------------------------------------------------------
    def adapter_event_callback(self, msg: String):
        data = msg.data.strip()
        if data.startswith("OBSERVED_STATE:"):
            self.lua_fsm_state = data.split("OBSERVED_STATE:")[1].strip()
        elif data.startswith("CORRELATED_TRANSITION:"):
            parts = data.split(":")
            if len(parts) >= 2:
                self.lua_fsm_state = parts[1].strip()
        elif data.startswith("SESSION_ACTIVE:"):
            try:
                new_session_id = int(data.split("SESSION_ACTIVE:")[1].strip())
            except Exception:
                return

            if self.current_session_id != new_session_id:
                self.reset_state()
                self.current_session_id = new_session_id
                self.get_logger().info(f"[PLANNER] Yeni oturum aktiflestirildi: {new_session_id}")
            else:
                self.get_logger().debug(f"[PLANNER] Ayni oturum tekrar alindi ({new_session_id}), kilitler korunuyor.")
        elif data.startswith("SESSION_STOPPED") or data.startswith("FATAL_SESSION_LIMIT"):
            self.reset_state()
            self.current_session_id = None

        # Kesin FSM tetikleme ve rota açma kuralı:
        # HEDEF_BEKLE durumuna geçildiğinde taze konum ve kilitli kalkış referansı varsa dönüş rotasını kontrol et/aç
        if self.lua_fsm_state == "HEDEF_BEKLE" and self.takeoff_return_pos_ned is not None:
            self.check_and_unlock_return_path()

    def detections_callback(self, msg: DetectionArray):
        if not msg.detections:
            return

        new_info = False
        for d in msg.detections:
            # 1. Metrik Doğruluk ve Güvenlik Filtresi (position_valid kontrolü)
            if not d.position_valid:
                continue

            local_x = float(d.local_x)
            local_y = float(d.local_y)
            if not (math.isfinite(local_x) and math.isfinite(local_y)):
                continue

            # ENU -> NED Eksen Dönüşümü (Sözleşmeye uygun)
            # local_x = East -> y_ned
            # local_y = North -> x_ned
            x_ned = local_y
            y_ned = local_x
            c_name = (d.class_name or "").strip().lower()

            # Start tespiti: İlk geçerli tespitte KİLİTLENİR (Jitter & drift koruması)
            if c_name in self.start_classes:
                if not self.start_locked:
                    self.start_pos_ned = (x_ned, y_ned)
                    self.start_locked = True
                    new_info = True
                    self.get_logger().info(f"[PLANNER] Start konumu KİLİTLENDİ: NED [{x_ned:.2f}, {y_ned:.2f}]")
                    self.pub_status.publish(String(data=f"START_LOCKED:{x_ned:.2f}:{y_ned:.2f}"))

            # Hedef tespiti
            elif c_name in self.goal_classes:
                if self.goal_pos_ned is None or math.hypot(self.goal_pos_ned[0]-x_ned, self.goal_pos_ned[1]-y_ned) > 0.5:
                    self.goal_pos_ned = (x_ned, y_ned)
                    new_info = True
                    self.get_logger().info(f"[PLANNER] Hedef kaydedildi: NED [{x_ned:.2f}, {y_ned:.2f}]")
                    self.pub_status.publish(String(data=f"GOAL_RECORDED:{x_ned:.2f}:{y_ned:.2f}"))

            # Engel tespiti
            elif c_name in self.obstacle_classes:
                is_duplicate = False
                for ox, oy in self.obstacles_ned:
                    if math.hypot(ox - x_ned, oy - y_ned) < 0.5:
                        is_duplicate = True
                        break
                if not is_duplicate:
                    self.obstacles_ned.append((x_ned, y_ned))
                    new_info = True
                    self.get_logger().info(f"[PLANNER] Yeni engel eklendi: NED [{x_ned:.2f}, {y_ned:.2f}]")
                    self.pub_status.publish(String(data=f"OBSTACLE_ADDED:{x_ned:.2f}:{y_ned:.2f}"))
                    
                    if self.return_path_locked and len(self.return_path_ned) >= 2:
                        cut = False
                        thresh = self.veh_w + self.safety_m
                        for px, py in self.return_path_ned:
                            if math.hypot(px - x_ned, py - y_ned) < thresh:
                                cut = True
                                break
                        if cut:
                            self.get_logger().warn(f"[PLANNER] Yeni engel ({x_ned:.2f}, {y_ned:.2f}) dönüş rotasını kesti! Kilit kaldırılıyor.")
                            self.invalidate_return_path("OBSTACLE_CUTS_RETURN_PATH", reset_attempts=True)

        # Yeni anlamlı konum bilgisi alındıysa ve hem start hem goal varsa rota planla
        if new_info and self.start_pos_ned is not None and self.goal_pos_ned is not None:
            self.execute_planning()

    # --------------------------------------------------------------------------
    # Segment Doğrulama ve Dönüş Rotası Yönetimi
    # --------------------------------------------------------------------------
    def _validate_path_segments(self, metric_path: List[Tuple[float, float]], work_grid: np.ndarray) -> bool:
        """Tüm segmentleri (ilk ve son noktalar dahil) ızgara engelleri ve sınırlarına karşı kontrol eder."""
        if len(metric_path) < 2:
            return False

        h, w = work_grid.shape
        step_dist = max(0.05, self.res * 0.5)

        for i in range(len(metric_path) - 1):
            p1 = metric_path[i]
            p2 = metric_path[i + 1]
            seg_len = math.hypot(p2[0] - p1[0], p2[1] - p1[1])
            num_steps = max(1, int(math.ceil(seg_len / step_dist)))

            for s in range(num_steps + 1):
                alpha = s / float(num_steps)
                sx = p1[0] + alpha * (p2[0] - p1[0])
                sy = p1[1] + alpha * (p2[1] - p1[1])
                grid_pt = self.ned_to_grid(sx, sy)
                if grid_pt is None:
                    return False
                gx, gy = grid_pt
                if not (0 <= gx < w and 0 <= gy < h):
                    return False
                if work_grid[gy, gx] == 1:
                    return False
        return True

    def invalidate_return_path(self, reason: str, reset_attempts: bool = False):
        """Dönüş rotasını geçersiz kılar, adaptördeki rotayı temizler ve Status 4'ü engeller."""
        self.return_path_ned = []
        self.route_ready_called = False
        self.return_path_locked = False
        if reset_attempts:
            self.return_plan_attempts = 0

        # Adaptördeki return_path'i temizlemek için aktif oturum ekiyle boş Path yayınla
        empty_path = Path()
        empty_path.header.stamp = self.get_clock().now().to_msg()
        if self.current_session_id is not None:
            empty_path.header.frame_id = f"ekf_origin_ned:session_{self.current_session_id}"
        else:
            empty_path.header.frame_id = "ekf_origin_ned"
        empty_path.poses = []
        self.pub_return_path.publish(empty_path)

        err = f"RETURN_PLANNING_FAILED:{reason}"
        self.get_logger().warn(f"[PLANNER] {err}")
        self.pub_status.publish(String(data=err))

    def plan_return_path(self, ret_start: Tuple[float, float]) -> Tuple[bool, str]:
        """
        Dönüş başlangıcından (ret_start) yerde kilitlenen fiziksel kalkış noktasına (takeoff_return_pos_ned)
        bağımsız A* ile ayrı dönüş rotası hesaplar.
        Tüm segmentleri (ilk ve son bağlantı dahil) engel ve sınır denetiminden geçirir.
        Tek noktalı yolları en az 2 waypoint sözleşmesine uyarlar.
        """
        t_start = time.monotonic()
        if self.takeoff_return_pos_ned is None or self.takeoff_return_pos_ned_3d is None:
            err = "MISSING_TAKEOFF_RETURN_REF"
            self.invalidate_return_path(err)
            return False, err

        if self.current_session_id is None or self.current_session_id <= 0:
            err = "NO_ACTIVE_SESSION"
            self.invalidate_return_path(err)
            return False, err

        grid_w = int(math.floor(self.arena_w / self.res))
        grid_h = int(math.floor(self.arena_h / self.res))
        grid = np.zeros((grid_h, grid_w), dtype=np.uint8)

        for ox, oy in self.obstacles_ned:
            pt = self.ned_to_grid(ox, oy)
            if pt is not None:
                grid[pt[1], pt[0]] = GridPlanner.OBSTACLE

        start_grid = self.ned_to_grid(*ret_start)
        goal_grid = self.ned_to_grid(*self.takeoff_return_pos_ned)

        if start_grid is None:
            err = "RETURN_START_OUT_OF_BOUNDS"
            self.invalidate_return_path(err)
            duration_ms = (time.monotonic() - t_start) * 1000.0
            self.get_logger().info(f"plan süresi: {duration_ms:.2f} ms")
            return False, err

        if goal_grid is None:
            err = "RETURN_GOAL_OUT_OF_BOUNDS"
            self.invalidate_return_path(err)
            duration_ms = (time.monotonic() - t_start) * 1000.0
            self.get_logger().info(f"plan süresi: {duration_ms:.2f} ms")
            return False, err

        resp, path_grid_ret, work_grid_ret = self.planner.plan_path(grid, start_grid, goal_grid)
        status = resp.get("status", "UNKNOWN")

        if status != "SUCCESS" or not path_grid_ret:
            err = f"NO_RETURN_PATH:{status}"
            self.invalidate_return_path(err)
            duration_ms = (time.monotonic() - t_start) * 1000.0
            self.get_logger().info(f"plan süresi: {duration_ms:.2f} ms")
            return False, err

        # Tek noktalı yolun iki uç atamasıyla bozulmasını önle (en az 2 waypoint sözleşmesi)
        if len(path_grid_ret) == 1:
            metric_ret = [ret_start, self.takeoff_return_pos_ned]
        else:
            metric_ret = [self.grid_to_ned(gx, gy) for gx, gy in path_grid_ret]
            metric_ret[0] = ret_start
            metric_ret[-1] = self.takeoff_return_pos_ned

        # Tüm segmentlerin (ilk ve son bağlantılar dahil) engel ve sınır denetimi
        if not self._validate_path_segments(metric_ret, work_grid_ret):
            err = "RETURN_SEGMENT_COLLISION"
            self.invalidate_return_path(err)
            return False, err

        self._publish_return_path(metric_ret)
        duration_ms = (time.monotonic() - t_start) * 1000.0
        self.get_logger().info(f"plan süresi: {duration_ms:.2f} ms")
        return True, f"RETURN_PLANNING_SUCCESS:waypoints_{len(metric_ret)}"

    def _publish_return_path(self, metric_ret: List[Tuple[float, float]]):
        """Doğrulanmış dönüş rotasını nav_msgs/Path olarak /planner/return_path konusuna yayınlar."""
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = f"ekf_origin_ned:session_{self.current_session_id}"

        # Seyir z değeri sözleşmesi: ref_z - target_cruise_alt (ref_z - 33.0)
        ref_z = self.takeoff_return_pos_ned_3d[2]
        cruise_z = float(ref_z - self.target_cruise_alt)

        for px, py in metric_ret:
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = float(px)
            pose.pose.position.y = float(py)
            pose.pose.position.z = cruise_z
            path_msg.poses.append(pose)

        self.pub_return_path.publish(path_msg)
        self.return_path_ned = metric_ret
        msg_success = f"RETURN_PLANNING_SUCCESS:waypoints_{len(metric_ret)}"
        self.get_logger().info(
            f"[PLANNER] Çarpışmasız dönüş rotası kullanıma açıldı ve yayınlandı: "
            f"{len(metric_ret)} nokta, cruise_z={cruise_z:.2f}m (Session: {self.current_session_id})"
        )
        self.pub_status.publish(String(data=msg_success))

    def check_and_unlock_return_path(self):
        """
        HEDEF_BEKLE durumundayken dönüş rotasını doğrular/planlar ve Status 4 (ROUTE_READY) tetikler.
        """
        # 1. KESİN DURUM KONTROLÜ - Yalnızca HEDEF_BEKLE durumunda açılabilir!
        if self.lua_fsm_state != "HEDEF_BEKLE":
            return

        # 2. Aktif oturum doğrulaması
        if self.current_session_id is None or self.current_session_id <= 0:
            self.get_logger().warn("[PLANNER] Dönüş rotası açılamıyor: Aktif oturum mevcut değil.")
            return

        # 3. Yerde kilitlenen fiziksel kalkış referansı kontrolü
        if self.takeoff_return_pos_ned is None or self.takeoff_return_pos_ned_3d is None:
            self.get_logger().warn("[PLANNER] Dönüş rotası açılamıyor: Kilitli fiziksel kalkış referansı henüz yok.")
            return

        # 4. Araç konumu ve güncelliği (staleness kontrolü: <3.0s)
        if not self._is_vehicle_pos_fresh():
            self.get_logger().warn("[PLANNER] Dönüş rotası açılamıyor: Araç konumu eksik veya güncel değil (stale)!")
            return

        if self.goal_pos_ned is None:
            return

        vx, vy = self.vehicle_pos_ned
        
        # Sadece TEK BİR KEZ teşhis logu al:
        if not self.diagnostic_logged:
            self.diagnostic_logged = True
            min_x_bd = -self.offset_x
            max_x_bd = self.arena_w - self.offset_x
            min_y_bd = -self.offset_y
            max_y_bd = self.arena_h - self.offset_y
            
            obs_x = [ox for ox, oy in self.obstacles_ned]
            obs_y = [oy for ox, oy in self.obstacles_ned]
            min_ox, max_ox = round(min(obs_x), 2) if obs_x else 0.0, round(max(obs_x), 2) if obs_x else 0.0
            min_oy, max_oy = round(min(obs_y), 2) if obs_y else 0.0, round(max(obs_y), 2) if obs_y else 0.0
            
            veh_valid = True if self.ned_to_grid(vx, vy) else False
            tkf_valid = True if self.ned_to_grid(*self.takeoff_return_pos_ned) else False
            
            self.get_logger().info(
                f"\n--- [PLANNER TEŞHİS LOGU (İLK DENEME)] ---\n"
                f"Araç Konumu (NED): ({vx:.2f}, {vy:.2f}) -> Grid'de mi: {veh_valid}\n"
                f"Kalkış Hedefi (NED): ({self.takeoff_return_pos_ned[0]:.2f}, {self.takeoff_return_pos_ned[1]:.2f}) -> Grid'de mi: {tkf_valid}\n"
                f"Start/Goal (NED): {self.start_pos_ned} / {self.goal_pos_ned}\n"
                f"Engel Sayısı: {len(self.obstacles_ned)}, Engel Min/Max X: [{min_ox}, {max_ox}], Y: [{min_oy}, {max_oy}]\n"
                f"Grid Sınırları: X Eksen [{min_x_bd:.2f}, {max_x_bd:.2f}], Y Eksen [{min_y_bd:.2f}, {max_y_bd:.2f}]\n"
                f"--------------------------------------------\n"
            )

        # 5. HAZIR ROTA KONTROLÜ: Önceden yayınlanmış hazır dönüş rotası varsa doğrudan tetikle
        if self.return_path_locked and len(self.return_path_ned) >= 2:
            if self.auto_route_ready and not self.route_ready_called:
                self.trigger_route_ready_call()
            return
            
        if self.return_plan_attempts >= 3:
            return
            
        now = time.monotonic()
        if now - self.last_attempt_mono < 1.0:
            return

        # Hazır rota yoksa: Doğrulanmış taze araç konumundan fiziksel kalkış referansına planla ve yayınla
        self.return_plan_attempts += 1
        self.last_attempt_mono = now
        ok, msg = self.plan_return_path((vx, vy))
        
        if ok:
            self.return_path_locked = True
            if self.auto_route_ready and not self.route_ready_called and len(self.return_path_ned) >= 2:
                self.trigger_route_ready_call()
        else:
            self.get_logger().warn(f"[PLANNER] Dönüş rotası planlanamadı: {msg} (Deneme: {self.return_plan_attempts}/3)")
            if self.return_plan_attempts >= 3:
                self.return_path_locked = True
                self.pub_status.publish(String(data="RETURN_PLANNING_FAILED_FINAL"))

    # --------------------------------------------------------------------------
    # Rota Planlama Çekirdeği
    # --------------------------------------------------------------------------
    def execute_planning(self) -> Tuple[bool, str]:
        if self.start_pos_ned is None:
            err = "PLANNING_FAILED:MISSING_START"
            self.pub_status.publish(String(data=err))
            return False, err

        if self.goal_pos_ned is None:
            err = "PLANNING_FAILED:MISSING_GOAL"
            self.pub_status.publish(String(data=err))
            return False, err

        grid_w = int(math.floor(self.arena_w / self.res))
        grid_h = int(math.floor(self.arena_h / self.res))
        grid = np.zeros((grid_h, grid_w), dtype=np.uint8)

        # Engelleri ızgaraya yerleştir
        for ox, oy in self.obstacles_ned:
            pt = self.ned_to_grid(ox, oy)
            if pt is not None:
                grid[pt[1], pt[0]] = GridPlanner.OBSTACLE

        start_grid = self.ned_to_grid(*self.start_pos_ned)
        goal_grid = self.ned_to_grid(*self.goal_pos_ned)

        if start_grid is None:
            err = "PLANNING_FAILED:START_OUT_OF_BOUNDS"
            self.get_logger().warn(f"[PLANNER] {err}")
            self.pub_status.publish(String(data=err))
            return False, err

        if goal_grid is None:
            err = "PLANNING_FAILED:GOAL_OUT_OF_BOUNDS"
            self.get_logger().warn(f"[PLANNER] {err}")
            self.pub_status.publish(String(data=err))
            return False, err

        # A* Arama çalıştır
        resp, path_grid, work_grid = self.planner.plan_path(grid, start_grid, goal_grid)
        status = resp.get("status", "UNKNOWN")

        if status != "SUCCESS" or not path_grid:
            err = f"PLANNING_FAILED:{status}"
            self.get_logger().warn(f"[PLANNER] Rota bulunamadi: {err}")
            self.pub_status.publish(String(data=err))
            return False, err

        # Izgara yolunu metrik NED koordinatlarına dönüştür
        if len(path_grid) == 1:
            metric_path = [self.start_pos_ned, self.goal_pos_ned]
        else:
            metric_path = [self.grid_to_ned(gx, gy) for gx, gy in path_grid]
            metric_path[0] = self.start_pos_ned
            metric_path[-1] = self.goal_pos_ned

        # Gidiş rotası segment kontrolü
        if not self._validate_path_segments(metric_path, work_grid):
            err = "PLANNING_FAILED:SEGMENT_COLLISION"
            self.get_logger().warn(f"[PLANNER] Gidiş segmenti engelle kesişti: {err}")
            self.pub_status.publish(String(data=err))
            return False, err

        self.last_path_ned = metric_path

        # 1. nav_msgs/Path yayınla
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = f"ekf_origin_ned:session_{self.current_session_id}" if self.current_session_id is not None else "ekf_origin_ned"
        for px, py in metric_path:
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = float(px)
            pose.pose.position.y = float(py)
            pose.pose.position.z = 0.0
            path_msg.poses.append(pose)
        self.pub_path.publish(path_msg)

        msg_success = f"PLANNING_SUCCESS:waypoints_{len(metric_path)}"
        self.get_logger().info(f"[PLANNER] Çarpışmasız rota başarıyla hesaplandı: {len(metric_path)} nokta.")
        self.pub_status.publish(String(data=msg_success))

        # Gidiş rotası /planner/path ile YKI'ye yayınlandı (otopilot yerine operatör ekranına gider).
        # HEDEF_BEKLE durumundaysa ve kalkış referansı kilitliyse dönüş rotasını aç ve Status 4 tetikle:
        if self.takeoff_return_pos_ned is not None and self.takeoff_return_pos_ned_3d is not None:
            if self.lua_fsm_state == "HEDEF_BEKLE" and self._is_vehicle_pos_fresh():
                self.check_and_unlock_return_path()
        else:
            self.get_logger().info("[PLANNER] takeoff_return referansı henüz kilitlenmedi, referans gelince dönüş planlanacak.")

        return True, msg_success

    def trigger_route_ready_call(self):
        """Status 4 için /mission/route_ready Trigger servisini çağırır."""
        # 1. Hazır rota varken konum eskirse Status 4 gönderilmemeli!
        if not self._is_vehicle_pos_fresh():
            self.get_logger().warn("[PLANNER] Status 4 engellendi: Araç konumu güncel değil (stale)!")
            self.pub_status.publish(String(data="ROUTE_READY_BLOCKED_STALE_POSITION"))
            return

        # 2. En az 2 waypoint içeren geçerli bir dönüş rotası olmalı
        if len(self.return_path_ned) < 2:
            self.get_logger().warn("[PLANNER] Status 4 engellendi: Geçerli dönüş rotası yok!")
            return

        if not self.cli_route_ready.service_is_ready():
            self.get_logger().warn("[PLANNER] /mission/route_ready servisi henüz hazır değil, çağrı bekletiliyor.")
            return

        self.route_ready_called = True
        req = Trigger.Request()
        self.get_logger().info("[PLANNER] Status 4: /mission/route_ready servisi çağrılıyor...")
        self.pub_status.publish(String(data="ROUTE_READY_REQUESTED"))

        future = self.cli_route_ready.call_async(req)
        future.add_done_callback(self._route_ready_response_cb)

    def _route_ready_response_cb(self, future):
        try:
            res = future.result()
            if res.success:
                self.get_logger().info(f"[PLANNER] ROUTE_READY kabul edildi: {res.message}")
                self.pub_status.publish(String(data=f"ROUTE_READY_ACCEPTED:{res.message}"))
            else:
                self.get_logger().warn(f"[PLANNER] ROUTE_READY reddedildi: {res.message}")
                self.pub_status.publish(String(data=f"ROUTE_READY_REJECTED:{res.message}"))
                self.route_ready_called = False  # Tekrar denemeye izin ver
        except Exception as e:
            self.get_logger().error(f"[PLANNER] ROUTE_READY çağrısında istisna: {e}")
            self.route_ready_called = False

    def plan_route_srv_cb(self, request, response):
        ok, msg = self.execute_planning()
        response.success = ok
        response.message = msg
        return response


def main(args=None):
    rclpy.init(args=args)
    node = PathPlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
