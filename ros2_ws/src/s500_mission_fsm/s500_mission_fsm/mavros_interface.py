"""
MAVROS Interface Module for S500 Quadcopter Mission FSM.
Handles all ROS 2 MAVROS publishers, subscribers, services, and coordinate frames.
All local coordinates follow ROS convention: ENU (X: East, Y: North, Z: Up).

Safety Rules Enforced:
1. No automatic takeoff pose latching in callbacks. Explicit ground capture only.
2. No assuming disarmed == landed. Missing/stale telemetry is strictly UNKNOWN.
3. Independent freshness tracking for every telemetry topic. Never return 0.0 for missing velocity.
"""

import time
import math
from enum import Enum
from typing import Optional, Tuple, List

from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import PoseStamped, TwistStamped
from nav_msgs.msg import Odometry
from mavros_msgs.msg import State, ExtendedState, Waypoint
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL, MessageInterval, StreamRate, ParamPull, WaypointPush
from rcl_interfaces.srv import SetParameters, GetParameters
from rcl_interfaces.msg import Parameter as RosParameter, ParameterValue, ParameterType


class LandedStatus(Enum):
    """Fiziksel iniş durumu telemetri göstergesi."""
    UNKNOWN = "UNKNOWN"       # Veri yok, eski (> timeout) veya tanımsız (UNDEFINED)
    ON_GROUND = "ON_GROUND"   # Güncel ve teyitli yerde (LANDED_STATE_ON_GROUND)
    IN_AIR = "IN_AIR"         # Havada uçuş halinde (LANDED_STATE_IN_AIR)
    TAKEOFF = "TAKEOFF"       # Kalkış tırmanış fazında (LANDED_STATE_TAKEOFF)
    LANDING = "LANDING"       # İniş alçalış fazında (LANDED_STATE_LANDING)


class MavrosInterface:
    def __init__(self, node: Node, autopilot_type: str = "ardupilot"):
        self.node = node
        self.autopilot_type = str(autopilot_type).lower()
        if self.autopilot_type not in ["ardupilot", "px4"]:
            raise ValueError(
                f"Geçersiz autopilot_type: '{self.autopilot_type}'. "
                "'ardupilot' veya 'px4' olmalıdır."
            )

        # Telemetry storage
        self._state: Optional[State] = None
        self._extended_state: Optional[ExtendedState] = None
        self._current_pose: Optional[PoseStamped] = None
        self._current_velocity: Optional[TwistStamped] = None

        # Bağımsız zaman damgaları (Freshness Watchdog)
        self._last_state_time: float = 0.0
        self._last_pose_time: float = 0.0
        self._last_vel_time: float = 0.0
        self._last_ext_state_time: float = 0.0

        # Dondurulmuş Kalkış/Home Konumu (ENU). Asla otomatik latch edilmez!
        self._takeoff_pose_frozen: Optional[Tuple[float, float, float]] = None

        # Son alınan konum örnekleri (Kararlılık / Varyans testi için)
        self._pose_history: List[Tuple[float, float, float, float]] = []  # (t, x, y, z)

        # QoS Profilleri
        qos_state = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        qos_sensor = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )

        # Subscribers (Açık ve kesin otopilot seçimi: Fallback YOK)
        self._sub_state = self.node.create_subscription(
            State, '/mavros/state', self._state_cb, qos_state
        )
        self._sub_ext_state = self.node.create_subscription(
            ExtendedState, '/mavros/extended_state', self._ext_state_cb, qos_sensor
        )

        if self.autopilot_type == "ardupilot":
            self.node.get_logger().info(
                "[MAVROS] Otopilot: ARDUPILOT. /mavros/global_position/local (Odometry ENU) dinleniyor."
            )
            self._sub_odom = self.node.create_subscription(
                Odometry, '/mavros/global_position/local', self._odom_cb, qos_sensor
            )
            self._sub_pose = None
            self._sub_vel = None
        else:
            self.node.get_logger().info(
                "[MAVROS] Otopilot: PX4. /mavros/local_position/pose ve velocity_local dinleniyor."
            )
            self._sub_pose = self.node.create_subscription(
                PoseStamped, '/mavros/local_position/pose', self._pose_cb, qos_sensor
            )
            self._sub_vel = self.node.create_subscription(
                TwistStamped, '/mavros/local_position/velocity_local', self._vel_cb, qos_sensor
            )
            self._sub_odom = None

        # Publishers
        self._pub_setpoint_pos = self.node.create_publisher(
            PoseStamped, '/mavros/setpoint_position/local', 10
        )

        # Service Clients
        self._cli_arming = self.node.create_client(CommandBool, '/mavros/cmd/arming')
        self._cli_set_mode = self.node.create_client(SetMode, '/mavros/set_mode')
        self._cli_takeoff = self.node.create_client(CommandTOL, '/mavros/cmd/takeoff')
        self._cli_land = self.node.create_client(CommandTOL, '/mavros/cmd/land')
        self._cli_set_msg_interval = self.node.create_client(MessageInterval, '/mavros/set_message_interval')
        self._cli_set_stream_rate = self.node.create_client(StreamRate, '/mavros/set_stream_rate')
        self._cli_push_mission = self.node.create_client(WaypointPush, '/mavros/mission/push')

    # ==========================================
    # Callbacks (Sadece veri saklar, karar vermez)
    # ==========================================
    def _state_cb(self, msg: State):
        self._state = msg
        self._last_state_time = time.time()

    def _ext_state_cb(self, msg: ExtendedState):
        self._extended_state = msg
        self._last_ext_state_time = time.time()

    def _odom_cb(self, msg: Odometry):
        """
        ArduPilot EKF yerel kartezyen ENU (East-North-Up) telemetrisi.
        /mavros/global_position/local konusundan gelen Odometry verisini
        PoseStamped ve TwistStamped olarak çözümler.
        İrtifa artışı pozitif (Up), alçalış negatif (Down) ENU kuralı uygulanır.
        """
        now = time.time()
        p = msg.pose.pose.position
        if (math.isfinite(p.x) and math.isfinite(p.y) and math.isfinite(p.z) and
                abs(p.x) < 1000.0 and abs(p.y) < 1000.0 and abs(p.z) < 1000.0):
            pose_msg = PoseStamped()
            pose_msg.header = msg.header
            pose_msg.pose = msg.pose.pose
            self._current_pose = pose_msg
            self._last_pose_time = now
            self._pose_history.append((now, p.x, p.y, p.z))
            if len(self._pose_history) > 100:
                self._pose_history = self._pose_history[-50:]

        v = msg.twist.twist.linear
        if math.isfinite(v.x) and math.isfinite(v.y) and math.isfinite(v.z):
            vel_msg = TwistStamped()
            vel_msg.header = msg.header
            # MAVLink GLOBAL_POSITION_INT vz ekseni (NED: Down+) ROS ENU (Up+) dönüşümü:
            # Tırmanışta vz pozitif Up olmalıdır.
            vel_msg.twist.linear.x = v.x
            vel_msg.twist.linear.y = v.y
            vel_msg.twist.linear.z = -v.z
            self._current_velocity = vel_msg
            self._last_vel_time = now

    def _pose_cb(self, msg: PoseStamped):
        now = time.time()
        p = msg.pose.position
        if (math.isfinite(p.x) and math.isfinite(p.y) and math.isfinite(p.z) and
                abs(p.x) < 1000.0 and abs(p.y) < 1000.0 and abs(p.z) < 1000.0):
            self._current_pose = msg
            self._last_pose_time = now
            self._pose_history.append((now, p.x, p.y, p.z))
            if len(self._pose_history) > 100:
                self._pose_history = self._pose_history[-50:]

    def _vel_cb(self, msg: TwistStamped):
        v = msg.twist.linear
        if math.isfinite(v.x) and math.isfinite(v.y) and math.isfinite(v.z):
            self._current_velocity = msg
            self._last_vel_time = time.time()

    # ==========================================
    # Güncellik (Freshness) Kontrolleri
    # ==========================================
    def is_state_fresh(self, max_age_s: float = 3.0) -> bool:
        return self._last_state_time > 0.0 and (time.time() - self._last_state_time) < max_age_s

    def is_pose_fresh(self, max_age_s: float = 1.0) -> bool:
        return self._last_pose_time > 0.0 and (time.time() - self._last_pose_time) < max_age_s

    def is_vel_fresh(self, max_age_s: float = 1.0) -> bool:
        return self._last_vel_time > 0.0 and (time.time() - self._last_vel_time) < max_age_s

    def is_ext_state_fresh(self, max_age_s: float = 1.5) -> bool:
        return self._last_ext_state_time > 0.0 and (time.time() - self._last_ext_state_time) < max_age_s

    def is_telemetry_healthy(self, timeout_s: float = 3.0) -> bool:
        """Tüm temel telemetri akışlarının sağlıklı ve güncel olduğunu doğrular."""
        return (
            self.is_state_fresh(timeout_s) and
            self.is_pose_fresh(timeout_s) and
            self.is_connected()
        )

    # ==========================================
    # Durum Bilgileri
    # ==========================================
    def is_connected(self) -> bool:
        return self.is_state_fresh() and self._state is not None and self._state.connected

    def is_armed(self) -> bool:
        return self.is_state_fresh() and self._state is not None and self._state.armed

    def get_flight_mode(self) -> str:
        if not self.is_state_fresh() or self._state is None:
            return "UNKNOWN"
        return self._state.mode

    # ==========================================
    # İniş Durumu (Düzeltme 2: Açık ve Güncel Teyit)
    # ==========================================
    def get_landed_status(self, max_age_s: float = 1.5) -> LandedStatus:
        """
        İniş durumunu telemetriden çözümler.
        Veri yoksa veya eskiyse kesinlikle UNKNOWN döner.
        """
        if not self.is_ext_state_fresh(max_age_s) or self._extended_state is None:
            return LandedStatus.UNKNOWN

        code = self._extended_state.landed_state
        if code == ExtendedState.LANDED_STATE_ON_GROUND:
            return LandedStatus.ON_GROUND
        elif code == ExtendedState.LANDED_STATE_IN_AIR:
            return LandedStatus.IN_AIR
        elif code == ExtendedState.LANDED_STATE_TAKEOFF:
            return LandedStatus.TAKEOFF
        elif code == ExtendedState.LANDED_STATE_LANDING:
            return LandedStatus.LANDING
        return LandedStatus.UNKNOWN

    def is_confirmed_landed(self, max_age_s: float = 1.5) -> bool:
        """
        İnişin tamamlandığını SADECE güncel ON_GROUND telemetrisiyle doğrular.
        Disarmed olmak veya verinin gelmemesi asla 'landed' sayılmaz!
        """
        return self.get_landed_status(max_age_s=max_age_s) == LandedStatus.ON_GROUND

    # ==========================================
    # Hız ve Konum (Düzeltme 3: Eksik Veride None Dönüşü)
    # ==========================================
    def get_current_xyz(self, max_age_s: float = 1.0) -> Optional[Tuple[float, float, float]]:
        """
        Güncel ENU konumunu döner.
        Veri eskiyse, yoksa veya NaN/Inf içeriyorsa None döner.
        """
        if not self.is_pose_fresh(max_age_s) or self._current_pose is None:
            return None
        p = self._current_pose.pose.position
        if not (math.isfinite(p.x) and math.isfinite(p.y) and math.isfinite(p.z)):
            return None
        return (p.x, p.y, p.z)

    def get_current_vertical_speed(self, max_age_s: float = 1.0) -> Optional[float]:
        """
        Dikey hızı (vz, m/s) döner.
        DİKKAT: Veri yoksa veya eskiyse 0.0 DÖNMEZ; None döner!
        Görev mantığı None değerini 'araç durdu' sanamaz.
        """
        if not self.is_vel_fresh(max_age_s) or self._current_velocity is None:
            return None
        vz = self._current_velocity.twist.linear.z
        if not math.isfinite(vz):
            return None
        return vz

    def get_current_horizontal_speed(self, max_age_s: float = 1.0) -> Optional[float]:
        """Yatay yer hızını (m/s) döner. Veri yoksa/eskiyse None döner."""
        if not self.is_vel_fresh(max_age_s) or self._current_velocity is None:
            return None
        vx = self._current_velocity.twist.linear.x
        vy = self._current_velocity.twist.linear.y
        if not (math.isfinite(vx) and math.isfinite(vy)):
            return None
        return math.hypot(vx, vy)

    # ==========================================
    # Kalkış Konumu Kilitleme (Düzeltme 1: Açık Fonksiyon)
    # ==========================================
    def is_takeoff_pose_locked(self) -> bool:
        return self._takeoff_pose_frozen is not None

    def get_takeoff_pose(self) -> Optional[Tuple[float, float, float]]:
        return self._takeoff_pose_frozen

    def capture_takeoff_pose(
        self,
        sample_count: int = 5,
        max_variance_m: float = 0.3,
        max_age_s: float = 1.0,
        check_landed: bool = True
    ) -> Tuple[bool, str]:
        """
        Kalkış konumunu açıkça, yalnızca yerdeyken ve kararlıysa kilitler.
        Bir kez kilitlendikten sonra görev boyunca veya bağlantı kopmasında değişmez.

        Returns:
            (success: bool, message: str)
        """
        if self._takeoff_pose_frozen is not None:
            return False, "Kalkış konumu zaten kilitlenmiş, üzerine yazılamaz."

        if not self.is_pose_fresh(max_age_s):
            return False, f"Konum verisi güncel değil (yaş > {max_age_s}s)."

        if check_landed and not self.is_confirmed_landed(max_age_s=2.0):
            status = self.get_landed_status()
            return False, f"Araç yerde doğrulanmadı. LandedStatus={status.value}."

        now = time.time()
        valid_samples = [
            (x, y, z) for (t, x, y, z) in self._pose_history
            if (now - t) <= max_age_s * 2.0
        ]

        if len(valid_samples) < sample_count:
            return False, f"Yetersiz konum örneği: {len(valid_samples)}/{sample_count}."

        recent = valid_samples[-sample_count:]
        xs = [p[0] for p in recent]
        ys = [p[1] for p in recent]
        zs = [p[2] for p in recent]

        if any(abs(c) >= 1000.0 for c in xs + ys + zs):
            return False, "Kalkış koordinatları makul aralıkta değil (|x,y,z| >= 1000m)."

        x_span = max(xs) - min(xs)
        y_span = max(ys) - min(ys)
        z_span = max(zs) - min(zs)

        if x_span > max_variance_m or y_span > max_variance_m or z_span > max_variance_m:
            return False, (
                f"Konum kararsız (sapma sınır aşıldı: dx={x_span:.2f}m, "
                f"dy={y_span:.2f}m, dz={z_span:.2f}m > {max_variance_m}m)."
            )

        avg_x = sum(xs) / len(xs)
        avg_y = sum(ys) / len(ys)
        avg_z = sum(zs) / len(zs)

        self._takeoff_pose_frozen = (avg_x, avg_y, avg_z)
        self.node.get_logger().info(
            f"[MAVROS] Kalkış konumu başarıyla kilitlendi (ENU): "
            f"X={avg_x:.3f}, Y={avg_y:.3f}, Z={avg_z:.3f} ({len(recent)} örnek ortalaması)"
        )
        return True, "Kalkış konumu başarıyla kilitlendi."

    # ==========================================
    # Servis ve Komut İstemcileri
    # ==========================================
    def request_stream_rate(self, stream_id: int = 0, message_rate: int = 10, on_off: bool = True):
        """
        ArduPilot MAVLink akış hızını (StreamRate) ayarlar.
        ArduPilot (hem SITL hem de donanım telem portları) için gereklidir.
        """
        if not self._cli_set_stream_rate.wait_for_service(timeout_sec=1.0):
            return None
        req = StreamRate.Request()
        req.stream_id = int(stream_id)
        req.message_rate = int(message_rate)
        req.on_off = bool(on_off)
        return self._cli_set_stream_rate.call_async(req)

    def request_extended_state_stream(self, rate_hz: float = 4.0):
        """ArduPilot'tan message 245 (EXTENDED_SYS_STATE) akışını talep eder."""
        if not self._cli_set_msg_interval.wait_for_service(timeout_sec=1.0):
            return None
        req = MessageInterval.Request()
        req.message_id = 245
        req.message_rate = float(rate_hz)
        return self._cli_set_msg_interval.call_async(req)

    def send_set_mode_async(self, custom_mode: str):
        if not self._cli_set_mode.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().error("[MAVROS] /mavros/set_mode servisi bulunamadı!")
            return None
        req = SetMode.Request()
        req.custom_mode = custom_mode
        return self._cli_set_mode.call_async(req)

    def send_arm_async(self, arm: bool):
        if not self._cli_arming.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().error("[MAVROS] /mavros/cmd/arming servisi bulunamadı!")
            return None
        req = CommandBool.Request()
        req.value = arm
        return self._cli_arming.call_async(req)

    def send_takeoff_async(self, altitude_m: float):
        if not self._cli_takeoff.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().error("[MAVROS] /mavros/cmd/takeoff servisi bulunamadı!")
            return None
        req = CommandTOL.Request()
        req.altitude = float(altitude_m)
        return self._cli_takeoff.call_async(req)

    def send_land_async(self):
        if not self._cli_land.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().error("[MAVROS] /mavros/cmd/land servisi bulunamadı!")
            return None
        req = CommandTOL.Request()
        return self._cli_land.call_async(req)

    def publish_position_target(self, x: float, y: float, z: float, yaw_rad: float = 0.0):
        msg = PoseStamped()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.position.z = float(z)

        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = math.sin(yaw_rad / 2.0)
        msg.pose.orientation.w = math.cos(yaw_rad / 2.0)

        self._pub_setpoint_pos.publish(msg)

    def set_ardupilot_speed_param(self, speed_mps: float, timeout_sec: float = 5.0) -> bool:
        """
        ArduPilot navigasyon seyir hızını (WP_SPD) ayarlar ve teyit eder.
        Parametrelerin MAVROS üzerinde bildirilmiş olması için gerekirse önce pull yapar,
        ardından SetParameters ile WP_SPD değerini ayarlar.
        GetParameters ile okunan değer speed_mps ile doğrulanmazsa veya teyit yapılamazsa
        kesinlikle False döner.
        """
        import rclpy
        helper = Node("_speed_param_setter_helper")
        try:
            cli_pull = helper.create_client(ParamPull, "/mavros/param/pull")
            cli_set = helper.create_client(SetParameters, "/mavros/param/set_parameters")
            cli_get = helper.create_client(GetParameters, "/mavros/param/get_parameters")

            # MAVROS'un otopilot parametrelerini almasını tetikle
            if cli_pull.wait_for_service(timeout_sec=2.0):
                f_p = cli_pull.call_async(ParamPull.Request(force_pull=True))
                rclpy.spin_until_future_complete(helper, f_p, timeout_sec=4.0)

            if not cli_set.wait_for_service(timeout_sec=timeout_sec):
                self.node.get_logger().error("[MAVROS] /mavros/param/set_parameters servisi bulunamadı!")
                return False

            p = RosParameter(name="WP_SPD", value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=float(speed_mps)))
            f_s = cli_set.call_async(SetParameters.Request(parameters=[p]))
            rclpy.spin_until_future_complete(helper, f_s, timeout_sec=timeout_sec)
            res_s = f_s.result()
            if not res_s or not res_s.results or not res_s.results[0].successful:
                reason = res_s.results[0].reason if (res_s and res_s.results) else "Zaman aşımı / boş yanıt"
                self.node.get_logger().error(f"[MAVROS] WP_SPD SetParameters başarısız: {reason}")
                return False

            # GetParameters ile teyit et: Bulunamazsa veya okunan değer eşleşmezse False dön
            if not cli_get.wait_for_service(timeout_sec=3.0):
                self.node.get_logger().warn("[MAVROS] WP_SPD GetParameters servisi bulunamadı, ayar teyit edilemedi!")
                return False

            f_g = cli_get.call_async(GetParameters.Request(names=["WP_SPD"]))
            rclpy.spin_until_future_complete(helper, f_g, timeout_sec=3.0)
            res_g = f_g.result()
            if not res_g or not res_g.values:
                self.node.get_logger().warn("[MAVROS] WP_SPD GetParameters boş veya geçersiz döndü, teyit başarısız!")
                return False

            val = res_g.values[0].double_value
            matched = (abs(val - float(speed_mps)) < 0.05)
            if matched:
                self.node.get_logger().info(f"[MAVROS] WP_SPD parametresi başarıyla teyit edildi: {val:.2f} m/s")
                return True
            else:
                self.node.get_logger().error(
                    f"[MAVROS] WP_SPD teyit uyuşmazlığı! Beklenen: {speed_mps:.2f} m/s, Okunan: {val:.2f} m/s"
                )
                return False
            helper.destroy_node()

    def push_mission(self, waypoints: List[Waypoint], timeout_sec: float = 5.0) -> bool:
        """
        Pushes a list of Waypoints to ArduPilot via MAVROS /mavros/mission/push service.
        Uses a helper node to synchronously wait for the service call to complete.
        """
        import rclpy
        helper = Node("_mission_push_helper")
        try:
            cli_push = helper.create_client(WaypointPush, "/mavros/mission/push")
            
            if not cli_push.wait_for_service(timeout_sec=timeout_sec):
                self.node.get_logger().error("[MAVROS] /mavros/mission/push servisi bulunamadı!")
                return False
                
            req = WaypointPush.Request()
            req.start_index = 0
            req.waypoints = waypoints
            
            f_p = cli_push.call_async(req)
            rclpy.spin_until_future_complete(helper, f_p, timeout_sec=timeout_sec)
            res = f_p.result()
            
            if not res:
                self.node.get_logger().error("[MAVROS] push_mission çağrısı sonuç dönmedi veya zaman aşımına uğradı.")
                return False
                
            if res.success:
                self.node.get_logger().info(f"[MAVROS] {res.wp_transfered} waypoint başarıyla yüklendi.")
                return True
            else:
                self.node.get_logger().error("[MAVROS] push_mission başarısız oldu!")
                return False
        finally:
            helper.destroy_node()

