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
3. Rota ve Gözlem Noktası Çıktıları:
   - Status 3 için: /mission/goto_observation (geometry_msgs/PoseStamped, frame: ekf_origin_ned) yayınlar.
   - Hesaplanan tam yol için: /planner/path (nav_msgs/Path) yayınlar.
   - Durum bildirimleri için: /planner/status (std_msgs/String) yayınlar.
4. Status 4 için:
   - Çarpışmasız rota hesaplandığında ve otopilot uygun durumda olduğunda
     (gözlem sonrası HEDEF_KONUMUNDA_BEKLE veya doğrudan dönüş için HEDEF_BEKLE),
     mavlink_mission_commander üzerindeki /mission/route_ready (std_srvs/Trigger)
     servisini çağırarak Lua FSM'nin DONUS aşamasına geçişini tetikler.
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
        self.declare_parameter('arena_width', 20.0)        # Metre (X ekseni genişliği)
        self.declare_parameter('arena_height', 20.0)       # Metre (Y ekseni genişliği)
        self.declare_parameter('grid_resolution', 0.5)     # Metre / hücre
        self.declare_parameter('vehicle_width', 0.6)       # S500 gövde genişliği (m)
        self.declare_parameter('safety_margin', 0.4)       # Güvenlik marjı (m)
        self.declare_parameter('origin_offset_x', 10.0)    # NED X -> Grid sütun ofseti (m)
        self.declare_parameter('origin_offset_y', 10.0)    # NED Y -> Grid satır ofseti (m)
        self.declare_parameter('strict_boundary_walls', False)  # Varsayılan: Açık hava uçuş sahası (kenarlar sahte duvar sayılmaz)
        self.declare_parameter('auto_trigger_route_ready', True)
        self.declare_parameter('start_classes', ['start'])
        self.declare_parameter('goal_classes', ['hedef', 'goal'])
        self.declare_parameter('obstacle_classes', ['engel', 'obstacle'])

        self.arena_w = float(self.get_parameter('arena_width').value)
        self.arena_h = float(self.get_parameter('arena_height').value)
        self.res = float(self.get_parameter('grid_resolution').value)
        self.veh_w = float(self.get_parameter('vehicle_width').value)
        self.safety_m = float(self.get_parameter('safety_margin').value)
        self.offset_x = float(self.get_parameter('origin_offset_x').value)
        self.offset_y = float(self.get_parameter('origin_offset_y').value)
        self.strict_boundary_walls = bool(self.get_parameter('strict_boundary_walls').value)
        self.auto_route_ready = bool(self.get_parameter('auto_trigger_route_ready').value)

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

        self.last_path_ned: List[Tuple[float, float]] = []
        self.lua_fsm_state: str = "UNKNOWN"
        self.observation_dispatched: bool = False
        self.route_ready_called: bool = False

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

        # 3. Status 3 Çıktısı: Gözlem Noktası (PoseStamped)
        self.pub_goto_obs = self.create_publisher(
            PoseStamped,
            '/mission/goto_observation',
            10
        )

        # 4. Planlanan Rota Çıktısı (nav_msgs/Path)
        self.pub_path = self.create_publisher(
            Path,
            '/planner/path',
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
        self.lua_fsm_state = "UNKNOWN"
        self.observation_dispatched = False
        self.route_ready_called = False

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

        # Kesin FSM tetikleme kuralı:
        # Gözlem noktasına gönderilmişse (observation_dispatched), varış şartı (HEDEF_KONUMUNDA_BEKLE) aranır.
        # Doğrudan dönüş senaryosunda ise HEDEF_BEKLE durumunda tetiklenir.
        # HEDEFE_GIT durumunda asla tetiklenmez (araç henüz yoldadır).
        if self.auto_route_ready and not self.route_ready_called and len(self.last_path_ned) > 0:
            if self.observation_dispatched:
                if self.lua_fsm_state == "HEDEF_KONUMUNDA_BEKLE":
                    self.trigger_route_ready_call()
            else:
                if self.lua_fsm_state == "HEDEF_BEKLE":
                    self.trigger_route_ready_call()

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

        # Yeni anlamlı konum bilgisi alındıysa ve hem start hem goal varsa rota planla
        if new_info and self.start_pos_ned is not None and self.goal_pos_ned is not None:
            self.execute_planning()

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
        metric_path: List[Tuple[float, float]] = []
        for gx, gy in path_grid:
            metric_path.append(self.grid_to_ned(gx, gy))

        # İlk ve son noktayı kesin tespit koordinatlarıyla hizala
        metric_path[0] = self.start_pos_ned
        metric_path[-1] = self.goal_pos_ned
        self.last_path_ned = metric_path

        # 1. nav_msgs/Path yayınla
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = "ekf_origin_ned"
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

        # 2. Status 3: Gözlem Noktası Gönderimi (Hedef veya hedefe yakın ara nokta)
        obs_x, obs_y = self.goal_pos_ned
        self.dispatch_goto_observation(obs_x, obs_y)

        # 3. Eğer doğrudan dönüş senaryosundaysak ve otopilot HEDEF_BEKLE durumundaysa Status 4 servisini çağır
        if self.auto_route_ready and not self.route_ready_called:
            if not self.observation_dispatched and self.lua_fsm_state == "HEDEF_BEKLE":
                self.trigger_route_ready_call()
            elif self.observation_dispatched and self.lua_fsm_state == "HEDEF_KONUMUNDA_BEKLE":
                self.trigger_route_ready_call()

        return True, msg_success

    def dispatch_goto_observation(self, x_ned: float, y_ned: float):
        """Status 3 için PoseStamped gözlem koordinatını yayınlar."""
        obs_msg = PoseStamped()
        obs_msg.header.stamp = self.get_clock().now().to_msg()
        obs_msg.header.frame_id = "ekf_origin_ned"
        obs_msg.pose.position.x = float(x_ned)
        obs_msg.pose.position.y = float(y_ned)
        obs_msg.pose.position.z = 0.0

        self.pub_goto_obs.publish(obs_msg)
        self.observation_dispatched = True
        self.get_logger().info(f"[PLANNER] Status 3 Gözlem Noktası yayınlandı: NED [{x_ned:.2f}, {y_ned:.2f}]")
        self.pub_status.publish(String(data=f"OBSERVATION_DISPATCHED:{x_ned:.2f}:{y_ned:.2f}"))

    def trigger_route_ready_call(self):
        """Status 4 için /mission/route_ready Trigger servisini çağırır."""
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
