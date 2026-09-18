"""
S500 Quadcopter ROS 2 Humble Mission State Machine Node.
Architecture:
- Flight Control: ArduCopter on Cube Orange (SITL / Real)
- Mission Logic: ROS 2 Humble node on Jetson
- States: BEKLEME -> HAZIRLIK -> GUIDED_VE_ARM -> KALKIS -> HAVADA_BEKLEME
          -> KISA_ROTA -> KALKIS_NOKTASINA_DONUS -> INIS -> TAMAMLANDI
- Failsafes: PILOT_MUDAHALESI, GOREV_IPTAL, IRTITA_IHLALI
"""

import time
import math
from enum import Enum
from typing import Optional, Tuple, List

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger

from s500_mission_fsm.mavros_interface import MavrosInterface, LandedStatus


class MissionState(Enum):
    BEKLEME = "BEKLEME"
    HAZIRLIK = "HAZIRLIK"
    GUIDED_VE_ARM = "GUIDED_VE_ARM"
    KALKIS = "KALKIS"
    HAVADA_BEKLEME = "HAVADA_BEKLEME"
    KISA_ROTA = "KISA_ROTA"
    KALKIS_NOKTASINA_DONUS = "KALKIS_NOKTASINA_DONUS"
    INIS = "INIS"
    TAMAMLANDI = "TAMAMLANDI"
    PILOT_MUDAHALESI = "PILOT_MUDAHALESI"
    GOREV_IPTAL = "GOREV_IPTAL"
    IRTITA_IHLALI = "IRTITA_IHLALI"


class S500MissionNode(Node):
    def __init__(self):
        super().__init__('s500_mission_fsm_node')

        # --- Parametreleri Yükle ---
        self.declare_parameter('min_cruise_altitude_m', 30.0)
        self.declare_parameter('max_relative_altitude_m', 35.0)
        self.declare_parameter('takeoff_altitude_m', 32.5)
        self.declare_parameter('cruise_altitude_m', 32.5)
        self.declare_parameter('altitude_tolerance_m', 0.8)
        self.declare_parameter('max_vertical_speed_for_arrival_ms', 0.25)
        self.declare_parameter('stability_duration_s', 2.0)

        self.declare_parameter('max_horizontal_speed_ms', 1.0)
        self.declare_parameter('waypoint_acceptance_radius_m', 0.8)
        self.declare_parameter('hover_duration_s', 5.0)
        self.declare_parameter('waypoint_hold_time_s', 1.0)

        self.declare_parameter('square_waypoints_x', [5.0, 5.0, 0.0, 0.0])
        self.declare_parameter('square_waypoints_y', [0.0, 5.0, 5.0, 0.0])

        self.declare_parameter('test_mode', 'FULL_ROUTE')  # 'HOVER_ONLY' veya 'FULL_ROUTE'
        self.declare_parameter('autopilot_type', 'ardupilot')  # 'ardupilot' veya 'px4'

        self.declare_parameter('telemetry_timeout_s', 3.0)
        self.declare_parameter('takeoff_timeout_s', 75.0)
        self.declare_parameter('landing_timeout_s', 80.0)
        self.declare_parameter('waypoint_timeout_s', 35.0)

        # Parametre değerlerini al
        self.min_cruise_alt = float(self.get_parameter('min_cruise_altitude_m').value)
        self.max_rel_alt = float(self.get_parameter('max_relative_altitude_m').value)
        self.takeoff_alt = float(self.get_parameter('takeoff_altitude_m').value)
        self.cruise_alt = float(self.get_parameter('cruise_altitude_m').value)
        self.alt_tol = float(self.get_parameter('altitude_tolerance_m').value)
        self.max_vz_arrival = float(self.get_parameter('max_vertical_speed_for_arrival_ms').value)
        self.stability_dur = float(self.get_parameter('stability_duration_s').value)

        self.max_h_speed = float(self.get_parameter('max_horizontal_speed_ms').value)
        self.wp_radius = float(self.get_parameter('waypoint_acceptance_radius_m').value)
        self.hover_dur = float(self.get_parameter('hover_duration_s').value)
        self.wp_hold = float(self.get_parameter('waypoint_hold_time_s').value)

        self.sq_wps_x = list(self.get_parameter('square_waypoints_x').value)
        self.sq_wps_y = list(self.get_parameter('square_waypoints_y').value)
        self.test_mode = str(self.get_parameter('test_mode').value)
        self.autopilot_type = str(self.get_parameter('autopilot_type').value).lower()

        self.telem_timeout = float(self.get_parameter('telemetry_timeout_s').value)
        self.takeoff_timeout = float(self.get_parameter('takeoff_timeout_s').value)
        self.landing_timeout = float(self.get_parameter('landing_timeout_s').value)
        self.wp_timeout = float(self.get_parameter('waypoint_timeout_s').value)

        # MAVROS arayüzü (Açık ve kesin otopilot seçimi)
        self.interface = MavrosInterface(self, autopilot_type=self.autopilot_type)

        # --- Durum Makinesi Değişkenleri ---
        self.state: MissionState = MissionState.BEKLEME
        self.state_enter_time: float = time.time()
        self.state_step_count: int = 0

        # İrtifa ve Performans Metrikleri (Raporlama için)
        self.min_cruise_rel_alt_observed: float = float('inf')
        self.max_cruise_rel_alt_observed: float = float('-inf')
        self.max_mission_rel_alt_observed: float = float('-inf')
        self.max_horizontal_speed_observed: float = 0.0
        self.return_distance_error_m: Optional[float] = None
        self.landing_completed_successfully: bool = False

        # Alt aşama sayaçları
        self.stability_start_time: Optional[float] = None
        self.takeoff_ack_received: bool = False
        self.current_wp_index: int = 0
        self.wp_arrival_time: Optional[float] = None

        # Güvenlik ve iptal nedeni
        self.abort_reason: str = ""

        # Servisler: Açık Görev Başlatma ve İptal
        self.srv_start = self.create_service(Trigger, '/mission/start', self._srv_start_cb)
        self.srv_abort = self.create_service(Trigger, '/mission/abort', self._srv_abort_cb)

        # Ana FSM Döngüsü (10 Hz)
        self.timer = self.create_timer(0.1, self._fsm_loop)

        self.get_logger().info(
            f"[S500 FSM] Başlatıldı. Durum: {self.state.value}. "
            f"İrtifa Kuralları: Hedef={self.cruise_alt}m, Band=[{self.min_cruise_alt}m, {self.max_rel_alt}m]. "
            f"Mod={self.test_mode}. Başlatmak için /mission/start servisini çağırın."
        )

    # ==========================================
    # Servis Tetikleyicileri
    # ==========================================
    def _srv_start_cb(self, request, response):
        if self.state == MissionState.BEKLEME:
            self._transition_to(MissionState.HAZIRLIK, "Kullanıcı /mission/start servisini çağırdı.")
            response.success = True
            response.message = "Görev başlatıldı. HAZIRLIK aşamasına geçildi."
        else:
            response.success = False
            response.message = f"Görev zaten aktif durumda: {self.state.value}."
        return response

    def _srv_abort_cb(self, request, response):
        self._abort_mission("Kullanıcı /mission/abort servisini çağırdı.")
        response.success = True
        response.message = "Görev kullanıcı tarafından iptal edildi."
        return response

    # ==========================================
    # Durum Geçiş Yardımcısı
    # ==========================================
    def _transition_to(self, new_state: MissionState, reason: str = ""):
        old_state = self.state
        self.state = new_state
        self.state_enter_time = time.time()
        self.state_step_count = 0
        self.stability_start_time = None
        self.wp_arrival_time = None
        self.get_logger().info(f"[FSM GEÇİŞ] {old_state.value} -> {new_state.value} | Gerekçe: {reason}")

    def _abort_mission(self, reason: str):
        self.abort_reason = reason
        self.get_logger().error(f"[GÖREV İPTAL] {reason}")
        self._transition_to(MissionState.GOREV_IPTAL, reason)

    # ==========================================
    # İrtifa ve Sınır Denetimleri
    # ==========================================
    def _check_altitude_bounds(self, current_rel_alt: float) -> bool:
        """
        Telemetri üzerinde irtifa kurallarını denetler:
        1. 35m Üst Sınır: Tüm görev boyunca (tüm durumlarda) kesindir.
        2. 30m Alt Sınır: Sadece seyir aşamalarında (HAVADA_BEKLEME, KISA_ROTA, KALKIS_NOKTASINA_DONUS) geçerlidir.
        """
        if current_rel_alt > self.max_mission_rel_alt_observed:
            self.max_mission_rel_alt_observed = current_rel_alt

        # 35m Tavan Denetimi (Her durumda geçerli)
        if current_rel_alt > self.max_rel_alt:
            self.abort_reason = (
                f"35m Tavan İhlali! Göreli irtifa={current_rel_alt:.2f}m > {self.max_rel_alt:.2f}m."
            )
            self.get_logger().error(f"[İRTİFA İHLALİ] {self.abort_reason}")
            self._transition_to(MissionState.IRTITA_IHLALI, self.abort_reason)
            return False

        # Seyir İçi İrtifa İstatistikleri ve 30m Taban Denetimi
        cruise_states = [
            MissionState.HAVADA_BEKLEME,
            MissionState.KISA_ROTA,
            MissionState.KALKIS_NOKTASINA_DONUS
        ]
        if self.state in cruise_states:
            if current_rel_alt < self.min_cruise_rel_alt_observed:
                self.min_cruise_rel_alt_observed = current_rel_alt
            if current_rel_alt > self.max_cruise_rel_alt_observed:
                self.max_cruise_rel_alt_observed = current_rel_alt

            # 30m Taban Denetimi
            if current_rel_alt < self.min_cruise_alt:
                self.abort_reason = (
                    f"30m Seyir Tabanı İhlali! Göreli irtifa={current_rel_alt:.2f}m < {self.min_cruise_alt:.2f}m."
                )
                self.get_logger().error(f"[İRTİFA İHLALİ] {self.abort_reason}")
                self._transition_to(MissionState.IRTITA_IHLALI, self.abort_reason)
                return False

        return True

    def _validate_target_altitude(self, target_rel_z: float) -> bool:
        """Hedef komutunun 30-35m bandı içinde olduğunu komut gönderilmeden önce doğrular."""
        if target_rel_z < self.min_cruise_alt or target_rel_z > self.max_rel_alt:
            self.get_logger().error(
                f"[KOMUT REDDİ] Geçersiz hedef irtifa talebi: {target_rel_z:.2f}m! "
                f"Sınırlar: [{self.min_cruise_alt}m, {self.max_rel_alt}m]."
            )
            return False
        return True

    # ==========================================
    # Ana FSM Döngüsü (10 Hz)
    # ==========================================
    def _fsm_loop(self):
        self.state_step_count += 1
        now = time.time()
        elapsed_in_state = now - self.state_enter_time

        # Hız metrik takibi
        h_spd = self.interface.get_current_horizontal_speed()
        if h_spd is not None and h_spd > self.max_horizontal_speed_observed:
            self.max_horizontal_speed_observed = h_spd

        # --- 1. Global Koruma: Telemetri Canlılığı (Uçuş Aşamaları) ---
        # HAZIRLIK aşaması kendi içinde bağlantının oturmasını bekler (30s timeout ile).
        if self.state not in [
            MissionState.BEKLEME,
            MissionState.HAZIRLIK,
            MissionState.TAMAMLANDI,
            MissionState.GOREV_IPTAL,
            MissionState.PILOT_MUDAHALESI
        ]:
            if not self.interface.is_telemetry_healthy(self.telem_timeout):
                self._abort_mission(f"Telemetri kesildi! Yaş > {self.telem_timeout}s.")
                return

        # --- 2. Global Koruma: Pilot Müdahalesi (Takeover) ---
        # Doğrulama 5: Kendi istediğimiz LAND modunu pilot müdahalesi sanma!
        # Pilot müdahalesi kontrolü yalnız otonom uçuş aşamalarında (kalkış ve sonrası) aktiftir.
        autonomous_states = [
            MissionState.KALKIS,
            MissionState.HAVADA_BEKLEME,
            MissionState.KISA_ROTA,
            MissionState.KALKIS_NOKTASINA_DONUS,
            MissionState.INIS
        ]
        if self.state in autonomous_states:
            current_mode = self.interface.get_flight_mode()
            if self.state == MissionState.INIS:
                # İnişte beklenen modlar GUIDED (geçiş anı) veya LAND (kendi komutumuz)
                if current_mode not in ["GUIDED", "LAND"]:
                    self.get_logger().warn(
                        f"[PILOT TAKE-OVER] İniş sırasında pilot modu değiştirdi: {current_mode}! "
                        "Otonom komutlar derhal kesildi."
                    )
                    self._transition_to(MissionState.PILOT_MUDAHALESI, f"Pilot iniş modunu {current_mode} yaptı.")
                    return
            else:
                if current_mode != "GUIDED":
                    self.get_logger().warn(
                        f"[PILOT TAKE-OVER] Uçuş modu değişti: {current_mode} != GUIDED! "
                        "Otonom komutlar derhal kesildi."
                    )
                    self._transition_to(MissionState.PILOT_MUDAHALESI, f"Pilot modu {current_mode} yaptı.")
                    return

        # --- 3. Konum ve Göreli İrtifa Hesaplama ---
        current_xyz = self.interface.get_current_xyz()
        takeoff_pose = self.interface.get_takeoff_pose()

        rel_alt: Optional[float] = None
        if current_xyz is not None and takeoff_pose is not None:
            rel_alt = current_xyz[2] - takeoff_pose[2]
            if not self._check_altitude_bounds(rel_alt):
                return

        # --- 4. Durum Yönetimi ---
        if self.state == MissionState.BEKLEME:
            pass

        elif self.state == MissionState.HAZIRLIK:
            self._handle_hazirlik(elapsed_in_state)

        elif self.state == MissionState.GUIDED_VE_ARM:
            self._handle_guided_ve_arm(elapsed_in_state)

        elif self.state == MissionState.KALKIS:
            self._handle_kalkis(elapsed_in_state, rel_alt, current_xyz)

        elif self.state == MissionState.HAVADA_BEKLEME:
            self._handle_havada_bekleme(elapsed_in_state, takeoff_pose)

        elif self.state == MissionState.KISA_ROTA:
            self._handle_kisa_rota(elapsed_in_state, current_xyz, takeoff_pose, rel_alt)

        elif self.state == MissionState.KALKIS_NOKTASINA_DONUS:
            self._handle_donus(elapsed_in_state, current_xyz, takeoff_pose, rel_alt)

        elif self.state == MissionState.INIS:
            self._handle_inis(elapsed_in_state)

        elif self.state == MissionState.TAMAMLANDI:
            if self.state_step_count == 1:
                self._log_mission_summary(success=True)

        elif self.state == MissionState.PILOT_MUDAHALESI:
            # Pilot devraldıktan sonra irtifa koruması dahil hiçbir görev mantığı komut göndermez
            pass

        elif self.state == MissionState.IRTITA_IHLALI:
            # İrtifa ihlali nedeniyle acil iniş başlatılır, fakat bu asla normal görev başarısı sayılamaz!
            if self.state_step_count == 1:
                self.get_logger().error("[ACİL DURUM] İrtifa bandı ihlali! Yatay hareket durduruluyor, LAND talep ediliyor.")
                self.interface.send_land_async()
            # İniş tamamlanınca da TAMAMLANDI yerine başarısız olarak raporlanır
            if self.interface.is_confirmed_landed() and not self.interface.is_armed():
                if self.state_step_count % 10 == 0:
                    self._log_mission_summary(success=False)

        elif self.state == MissionState.GOREV_IPTAL:
            if self.state_step_count == 1:
                self._log_mission_summary(success=False)

    # ==========================================
    # Durum İşleyicileri
    # ==========================================
    def _handle_hazirlik(self, elapsed: float):
        """HAZIRLIK: Telemetri, yer durumu ve kalkış konumu doğrulaması."""
        # ExtendedState ve otopilot veri akışı talep et
        if self.state_step_count % 10 == 1:
            self.interface.request_extended_state_stream(4.0)
            if self.autopilot_type == "ardupilot":
                self.interface.request_stream_rate(stream_id=0, message_rate=10, on_off=True)

        if elapsed > 30.0:
            self._abort_mission("HAZIRLIK aşaması zaman aşımına uğradı (30s).")
            return

        if not self.interface.is_telemetry_healthy():
            if self.state_step_count % 20 == 0:
                self.get_logger().info("[HAZIRLIK] Telemetri bağlantısı bekleniyor...")
            return

        if not self.interface.is_confirmed_landed():
            if self.state_step_count % 20 == 0:
                self.get_logger().info("[HAZIRLIK] Aracın yerde olduğu teyidi bekleniyor (ExtendedState)...")
            return

        # Kalkış konumunu açıkça kaydet
        if not self.interface.is_takeoff_pose_locked():
            success, reason = self.interface.capture_takeoff_pose(
                sample_count=5, max_variance_m=0.3, check_landed=True
            )
            if not success:
                if self.state_step_count % 20 == 0:
                    self.get_logger().warn(f"[HAZIRLIK] Kalkış konumu kilitlenemedi: {reason}")
                return

        self._transition_to(MissionState.GUIDED_VE_ARM, "Kalkış konumu başarıyla kilitlendi.")

    def _handle_guided_ve_arm(self, elapsed: float):
        """GUIDED_VE_ARM: Mod değiştirme ve motor arm etme."""
        if elapsed > 60.0:
            self._abort_mission("GUIDED_VE_ARM aşaması zaman aşımına uğradı (60s).")
            return

        mode_ok = (self.interface.get_flight_mode() == "GUIDED")
        armed_ok = self.interface.is_armed()

        if not mode_ok and (self.state_step_count % 20 == 1):
            self.get_logger().info("[GUIDED_VE_ARM] GUIDED modu talep ediliyor...")
            self.interface.send_set_mode_async("GUIDED")

        if mode_ok and not armed_ok and (self.state_step_count % 20 == 1):
            self.get_logger().info("[GUIDED_VE_ARM] Motor ARM talep ediliyor...")
            self.interface.send_arm_async(True)

        if mode_ok and armed_ok:
            self._transition_to(MissionState.KALKIS, "GUIDED modu ve ARM telemetriyle teyit edildi.")

    def _handle_kalkis(self, elapsed: float, rel_alt: Optional[float], current_xyz: Optional[Tuple[float, float, float]]):
        """KALKIŞ: 32.5 metreye tırmanış ve kararlılık teyidi."""
        if elapsed > self.takeoff_timeout:
            self._abort_mission(f"Kalkış zaman aşımına uğradı ({self.takeoff_timeout}s)!")
            return

        if self.state_step_count == 1:
            self.get_logger().info(f"[KALKIŞ] {self.takeoff_alt}m kalkış komutu gönderiliyor...")
            self.interface.send_takeoff_async(self.takeoff_alt)
            self.takeoff_ack_received = True

        if rel_alt is None:
            return

        vz = self.interface.get_current_vertical_speed()
        if vz is None:
            return

        alt_err = abs(rel_alt - self.takeoff_alt)
        speed_ok = (abs(vz) <= self.max_vz_arrival)

        if alt_err <= self.alt_tol and speed_ok:
            if self.stability_start_time is None:
                self.stability_start_time = time.time()
                self.get_logger().info(
                    f"[KALKIŞ] Hedef irtifa penceresine girildi (alt={rel_alt:.2f}m, vz={vz:.2f}m/s). "
                    f"{self.stability_dur}s kararlılık bekleniyor..."
                )
            elif (time.time() - self.stability_start_time) >= self.stability_dur:
                self.get_logger().info(
                    f"[KALKIŞ TAMAM] Hedef irtifada kararlı kalındı (alt={rel_alt:.2f}m). "
                    "HAVADA_BEKLEME aşamasına geçiliyor."
                )
                self._transition_to(MissionState.HAVADA_BEKLEME, "Kalkış başarıyla tamamlandı.")
        else:
            self.stability_start_time = None

    def _handle_havada_bekleme(self, elapsed: float, takeoff_pose: Tuple[float, float, float]):
        """HAVADA_BEKLEME: 32.5 metrede 5 saniye havada asılı kalma."""
        target_x = takeoff_pose[0]
        target_y = takeoff_pose[1]
        target_z = takeoff_pose[2] + self.cruise_alt

        if not self._validate_target_altitude(self.cruise_alt):
            self._abort_mission("Havada bekleme hedef irtifası sınır dışı!")
            return

        self.interface.publish_position_target(target_x, target_y, target_z)

        if elapsed >= self.hover_dur:
            if self.test_mode == "HOVER_ONLY":
                self.get_logger().info("[HAVADA BEKLEME] 5s tamamlandı (HOVER_ONLY). İNİŞ aşamasına geçiliyor.")
                self._transition_to(MissionState.INIS, "Test A (Hover Only) tamamlandı.")
            else:
                self.get_logger().info("[HAVADA BEKLEME] 5s tamamlandı. KISA_ROTA aşamasına geçiliyor.")
                self.current_wp_index = 0
                self._transition_to(MissionState.KISA_ROTA, "Test B (Kısa Rota) başlatılıyor.")

    def _handle_kisa_rota(
        self,
        elapsed: float,
        current_xyz: Optional[Tuple[float, float, float]],
        takeoff_pose: Tuple[float, float, float],
        rel_alt: Optional[float]
    ):
        """KISA_ROTA: (5,0), (5,5), (0,5), (0,0) kare rotası."""
        if self.current_wp_index >= len(self.sq_wps_x):
            self.get_logger().info("[KISA ROTA] Tüm kare waypoint'leri tamamlandı. KALKIŞ_NOKTASINA_DÖNÜŞ aşamasına geçiliyor.")
            self._transition_to(MissionState.KALKIS_NOKTASINA_DONUS, "Kare rota bitti.")
            return

        wp_rel_x = self.sq_wps_x[self.current_wp_index]
        wp_rel_y = self.sq_wps_y[self.current_wp_index]

        # Doğrulama 4: Koordinatlar kaydedilen kalkış konumuna göre hesaplanır
        target_x = takeoff_pose[0] + wp_rel_x
        target_y = takeoff_pose[1] + wp_rel_y
        target_z = takeoff_pose[2] + self.cruise_alt

        if not self._validate_target_altitude(self.cruise_alt):
            self._abort_mission("Rota hedef irtifası sınır dışı!")
            return

        self.interface.publish_position_target(target_x, target_y, target_z)

        if current_xyz is None or rel_alt is None:
            return

        dist_h = math.hypot(current_xyz[0] - target_x, current_xyz[1] - target_y)
        alt_err = abs(rel_alt - self.cruise_alt)

        if dist_h <= self.wp_radius and alt_err <= self.alt_tol:
            if self.wp_arrival_time is None:
                self.wp_arrival_time = time.time()
                self.get_logger().info(
                    f"[KISA ROTA] WP {self.current_wp_index + 1}/{len(self.sq_wps_x)} "
                    f"({wp_rel_x}, {wp_rel_y}) hedefine varıldı (dist={dist_h:.2f}m). "
                    f"{self.wp_hold}s bekleme..."
                )
            elif (time.time() - self.wp_arrival_time) >= self.wp_hold:
                self.get_logger().info(f"[KISA ROTA] WP {self.current_wp_index + 1} tamamlandı.")
                self.current_wp_index += 1
                self.wp_arrival_time = None
                self.state_enter_time = time.time()
        else:
            self.wp_arrival_time = None
            if elapsed > self.wp_timeout:
                self._abort_mission(f"WP {self.current_wp_index + 1} için zaman aşımı ({self.wp_timeout}s)!")

    def _handle_donus(
        self,
        elapsed: float,
        current_xyz: Optional[Tuple[float, float, float]],
        takeoff_pose: Tuple[float, float, float],
        rel_alt: Optional[float]
    ):
        """KALKIŞ_NOKTASINA_DÖNÜŞ: (takeoff_x, takeoff_y, 32.5m) noktasına varış ve kararlılık."""
        target_x = takeoff_pose[0]
        target_y = takeoff_pose[1]
        target_z = takeoff_pose[2] + self.cruise_alt

        self.interface.publish_position_target(target_x, target_y, target_z)

        if current_xyz is None or rel_alt is None:
            return

        dist_h = math.hypot(current_xyz[0] - target_x, current_xyz[1] - target_y)
        h_speed = self.interface.get_current_horizontal_speed()
        alt_err = abs(rel_alt - self.cruise_alt)

        if dist_h <= self.wp_radius and alt_err <= self.alt_tol and (h_speed is not None and h_speed <= 0.3):
            if self.stability_start_time is None:
                self.stability_start_time = time.time()
                self.get_logger().info(
                    f"[DÖNÜŞ] Kalkış noktasına varıldı (dist={dist_h:.2f}m, speed={h_speed:.2f}m/s). "
                    f"{self.stability_dur}s kararlılık bekleniyor..."
                )
            elif (time.time() - self.stability_start_time) >= self.stability_dur:
                self.return_distance_error_m = dist_h
                self.get_logger().info(
                    f"[DÖNÜŞ TAMAM] Kalkış noktasında kararlılık sağlandı (Hata={dist_h:.2f}m). "
                    "İNİŞ aşamasına geçiliyor."
                )
                self._transition_to(MissionState.INIS, "Dönüş başarıyla tamamlandı.")
        else:
            self.stability_start_time = None
            if elapsed > self.wp_timeout:
                self._abort_mission(f"Kalkış noktasına dönüş zaman aşımına uğradı ({self.wp_timeout}s)!")

    def _handle_inis(self, elapsed: float):
        """İNİŞ: LAND modu talebi, çift katmanlı teyit (ON_GROUND + DISARMED)."""
        if elapsed > self.landing_timeout:
            self._abort_mission(f"İniş zaman aşımına uğradı ({self.landing_timeout}s)!")
            return

        if self.state_step_count == 1:
            self.get_logger().info("[İNİŞ] LAND modu talep ediliyor...")
            self.interface.send_set_mode_async("LAND")

        is_on_ground = self.interface.is_confirmed_landed()
        is_disarmed = (not self.interface.is_armed())

        if is_on_ground and is_disarmed:
            self.landing_completed_successfully = True
            self.get_logger().info("[İNİŞ TAMAM] Yerde olma ve DISARM durumu birlikte doğrulandı.")
            self._transition_to(MissionState.TAMAMLANDI, "İniş ve motor duruşu başarıyla doğrulandı.")
        elif is_disarmed and not is_on_ground:
            if self.state_step_count % 20 == 0:
                self.get_logger().warn("[İNİŞ] Motorlar durdu ancak telemetri ON_GROUND henüz teyit etmedi; bekleniyor...")

    def _log_mission_summary(self, success: bool):
        min_c = f"{self.min_cruise_rel_alt_observed:.2f} m" if self.min_cruise_rel_alt_observed != float('inf') else "N/A"
        max_c = f"{self.max_cruise_rel_alt_observed:.2f} m" if self.max_cruise_rel_alt_observed != float('-inf') else "N/A"
        max_m = f"{self.max_mission_rel_alt_observed:.2f} m" if self.max_mission_rel_alt_observed != float('-inf') else "N/A"
        ret_err = f"{self.return_distance_error_m:.2f} m" if self.return_distance_error_m is not None else "N/A"

        self.get_logger().info("=" * 65)
        self.get_logger().info(f"             GÖREV RAPORU: {'BAŞARILI' if success else 'İPTAL / HATA'}")
        self.get_logger().info("=" * 65)
        self.get_logger().info(f"Son Durum                      : {self.state.value}")
        self.get_logger().info(f"Seyir En Düşük İrtifa          : {min_c} (Kural: >= 30.0 m)")
        self.get_logger().info(f"Seyir En Yüksek İrtifa         : {max_c} (Kural: <= 35.0 m)")
        self.get_logger().info(f"Tüm Görev Zirve İrtifa         : {max_m} (Kural: <= 35.0 m)")
        self.get_logger().info(f"Ölçülen En Yüksek Yatay Hız    : {self.max_horizontal_speed_observed:.2f} m/s")
        self.get_logger().info(f"Kalkış Noktasına Dönüş Hatası  : {ret_err}")
        self.get_logger().info(f"İniş ve Disarm Teyidi          : {'EVET' if self.landing_completed_successfully else 'HAYIR'}")
        if not success:
            self.get_logger().info(f"İptal / Hata Gerekçesi         : {self.abort_reason}")
        self.get_logger().info("=" * 65)


def main(args=None):
    rclpy.init(args=args)
    node = S500MissionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
