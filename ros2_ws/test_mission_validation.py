#!/usr/bin/env python3
"""
Hardware-Free Validation & YAML Parameter Loading Test Suite for S500 Mission FSM.

Part 1: Unit Tests (Mocked Interface & Controlled Clock)
- Lawnmower waypoint generation (20x20m / 4m spacing -> 6 lanes, 12 waypoints).
- Parameter typing and declaration.
- test_mode normalization and rejection.
- Service rejection on parameter errors, verifying NO commands are issued.
- Controlled clock verification: hold times, telemetry gap resets, timeout enforcement.

Part 2: Real YAML Integration Tests (--ros-args --params-file)
- Real ROS 2 parameter loader reading mission_params.yaml.
- Confirmation that custom_survey_waypoints_x/y are empty and DOUBLE_ARRAY.
- Confirmation that 6 lanes and 12 waypoints are generated from the YAML.
- Confirmation of no command issuance.
- Verification with temporary YAML copy that params-file actively overrides defaults (non-default check).
"""

import math
import os
import sys
import tempfile
import unittest
from unittest.mock import MagicMock, patch

import rclpy
from rclpy.context import Context
from rclpy.parameter import Parameter
from rcl_interfaces.msg import ParameterType
from std_srvs.srv import Trigger
import yaml

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), 'src/s500_mission_fsm')))
from s500_mission_fsm.mission_node import S500MissionNode, MissionState


class ControllableClock:
    """Sahte zaman kontrolörü (Testlerde zamanı kontrollü ve deterministik ilerletmek için)."""
    def __init__(self, start_time: float = 1000.0):
        self._current_time = float(start_time)

    def time(self) -> float:
        return self._current_time

    def advance(self, seconds: float):
        self._current_time += float(seconds)


class TestMissionFSMValidation(unittest.TestCase):
    """Birim Testleri: Mock MAVROS arayüzü ve sahte saat ile mantık doğrulamaları."""

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    # =========================================================================
    # 1. LAWNMOWER / BOUSTROPHEDON GENERATION TESTS
    # =========================================================================
    def test_lawnmower_default_20x20_spacing_4(self):
        """Varsayılan 20x20m alan ve 4m aralıkta 6 şerit ve 12 waypoint üretilmeli."""
        wps_x, wps_y = S500MissionNode.generate_lawnmower_waypoints(20.0, 20.0, 4.0)
        self.assertEqual(len(wps_x), 12, f"Beklenen 12 waypoint, üretilen: {len(wps_x)}")
        self.assertEqual(len(wps_y), 12)

        # 6 şerit: y koordinatları [0.0, 4.0, 8.0, 12.0, 16.0, 20.0]
        unique_y = sorted(list(set(wps_y)))
        self.assertEqual(unique_y, [0.0, 4.0, 8.0, 12.0, 16.0, 20.0])
        self.assertEqual(len(unique_y), 6)

        # İlk şerit (Batı -> Doğu): (0.0, 0.0) -> (20.0, 0.0)
        self.assertEqual((wps_x[0], wps_y[0]), (0.0, 0.0))
        self.assertEqual((wps_x[1], wps_y[1]), (20.0, 0.0))

        # İkinci şerit (Doğu -> Batı): (20.0, 4.0) -> (0.0, 4.0)
        self.assertEqual((wps_x[2], wps_y[2]), (20.0, 4.0))
        self.assertEqual((wps_x[3], wps_y[3]), (0.0, 4.0))

        # Son şerit (y=20.0): (20.0, 20.0) -> (0.0, 20.0)
        self.assertEqual((wps_x[10], wps_y[10]), (20.0, 20.0))
        self.assertEqual((wps_x[11], wps_y[11]), (0.0, 20.0))

    def test_lawnmower_invalid_parameters_raise(self):
        """Geçersiz alan genişliği, uzunluğu veya şerit aralığı ValueError fırlatmalı."""
        with self.assertRaises(ValueError):
            S500MissionNode.generate_lawnmower_waypoints(-20.0, 20.0, 4.0)
        with self.assertRaises(ValueError):
            S500MissionNode.generate_lawnmower_waypoints(20.0, 0.0, 4.0)
        with self.assertRaises(ValueError):
            S500MissionNode.generate_lawnmower_waypoints(20.0, 20.0, -4.0)
        with self.assertRaises(ValueError):
            S500MissionNode.generate_lawnmower_waypoints(float('nan'), 20.0, 4.0)
        with self.assertRaises(ValueError):
            S500MissionNode.generate_lawnmower_waypoints(20.0, float('inf'), 4.0)

    # =========================================================================
    # 2. SEPARATE NODE PARAMETER SCENARIOS & TYPING
    # =========================================================================
    @patch('s500_mission_fsm.mission_node.MavrosInterface')
    def test_param_scenario_no_override(self, mock_interface_cls):
        """Senaryo 1: Override verilmeden başlatma (Varsayılanlar: DOUBLE_ARRAY, 12 WP)."""
        node = S500MissionNode()
        try:
            p_x = node.get_parameter('custom_survey_waypoints_x')
            p_y = node.get_parameter('custom_survey_waypoints_y')
            self.assertEqual(p_x.type_, Parameter.Type.DOUBLE_ARRAY)
            self.assertEqual(p_y.type_, Parameter.Type.DOUBLE_ARRAY)
            self.assertEqual(p_x.value, [])
            self.assertEqual(p_y.value, [])
            self.assertIsNone(node.param_validation_error)
            self.assertEqual(len(node.survey_wps_x), 12)
            self.assertEqual(len(node.survey_wps_y), 12)
        finally:
            node.destroy_node()

    @patch('s500_mission_fsm.mission_node.MavrosInterface')
    def test_param_scenario_override_empty_double_arrays(self, mock_interface_cls):
        """Senaryo 2: parameter_overrides üzerinden boş DOUBLE_ARRAY verilerek başlatma (Birim testi)."""
        node = S500MissionNode(parameter_overrides=[
            Parameter('custom_survey_waypoints_x', Parameter.Type.DOUBLE_ARRAY, []),
            Parameter('custom_survey_waypoints_y', Parameter.Type.DOUBLE_ARRAY, []),
        ])
        try:
            p_x = node.get_parameter('custom_survey_waypoints_x')
            p_y = node.get_parameter('custom_survey_waypoints_y')
            self.assertEqual(p_x.type_, Parameter.Type.DOUBLE_ARRAY)
            self.assertEqual(p_y.type_, Parameter.Type.DOUBLE_ARRAY)
            self.assertEqual(p_x.value, [])
            self.assertEqual(p_y.value, [])
            self.assertIsNone(node.param_validation_error)
            self.assertEqual(len(node.survey_wps_x), 12)
            self.assertEqual(len(node.survey_wps_y), 12)
        finally:
            node.destroy_node()

    @patch('s500_mission_fsm.mission_node.MavrosInterface')
    def test_param_scenario_populated_float_arrays(self, mock_interface_cls):
        """Senaryo 3: Eşit uzunlukta dolu float dizileriyle özel rota oluşturulması."""
        custom_x = [1.5, 3.5, 5.0]
        custom_y = [2.0, 4.0, 6.0]
        node = S500MissionNode(parameter_overrides=[
            Parameter('custom_survey_waypoints_x', Parameter.Type.DOUBLE_ARRAY, custom_x),
            Parameter('custom_survey_waypoints_y', Parameter.Type.DOUBLE_ARRAY, custom_y),
        ])
        try:
            p_x = node.get_parameter('custom_survey_waypoints_x')
            p_y = node.get_parameter('custom_survey_waypoints_y')
            self.assertEqual(p_x.type_, Parameter.Type.DOUBLE_ARRAY)
            self.assertEqual(p_y.type_, Parameter.Type.DOUBLE_ARRAY)
            self.assertIsNone(node.param_validation_error)
            self.assertEqual(node.survey_wps_x, custom_x)
            self.assertEqual(node.survey_wps_y, custom_y)
        finally:
            node.destroy_node()

    @patch('s500_mission_fsm.mission_node.MavrosInterface')
    def test_param_scenario_single_array_populated_rejects(self, mock_interface_cls):
        """Senaryo 4: Tek dizinin dolu olması durumunda görev başlangıcı reddedilmeli ve komut gönderilmemeli."""
        mock_if = mock_interface_cls.return_value
        node = S500MissionNode(parameter_overrides=[
            Parameter('custom_survey_waypoints_x', Parameter.Type.DOUBLE_ARRAY, [1.0, 2.0]),
            Parameter('custom_survey_waypoints_y', Parameter.Type.DOUBLE_ARRAY, []),
        ])
        try:
            self.assertIsNotNone(node.param_validation_error)
            self.assertIn("Özel tarama koordinatları tutarsız", node.param_validation_error)
            self.assertEqual(len(node.survey_wps_x), 0)

            # /mission/start servis çağrısı
            req = Trigger.Request()
            resp = Trigger.Response()
            result = node._srv_start_cb(req, resp)

            self.assertFalse(result.success)
            self.assertEqual(node.state, MissionState.GOREV_IPTAL)

            # Çıkış komutlarının gönderilmediğini doğrula
            mock_if.send_set_mode_async.assert_not_called()
            mock_if.send_arm_async.assert_not_called()
            mock_if.send_takeoff_async.assert_not_called()
            mock_if.send_land_async.assert_not_called()
            mock_if.publish_position_target.assert_not_called()
        finally:
            node.destroy_node()

    @patch('s500_mission_fsm.mission_node.MavrosInterface')
    def test_param_scenario_length_mismatch_rejects(self, mock_interface_cls):
        """Senaryo 5: Uzunluk uyuşmazlığında görev başlangıcı reddedilmeli ve komut gönderilmemeli."""
        mock_if = mock_interface_cls.return_value
        node = S500MissionNode(parameter_overrides=[
            Parameter('custom_survey_waypoints_x', Parameter.Type.DOUBLE_ARRAY, [1.0, 2.0, 3.0]),
            Parameter('custom_survey_waypoints_y', Parameter.Type.DOUBLE_ARRAY, [1.0, 2.0]),
        ])
        try:
            self.assertIsNotNone(node.param_validation_error)
            self.assertIn("Özel tarama koordinatları tutarsız", node.param_validation_error)
            self.assertEqual(len(node.survey_wps_x), 0)

            req = Trigger.Request()
            resp = Trigger.Response()
            result = node._srv_start_cb(req, resp)

            self.assertFalse(result.success)
            self.assertEqual(node.state, MissionState.GOREV_IPTAL)

            mock_if.send_set_mode_async.assert_not_called()
            mock_if.send_arm_async.assert_not_called()
            mock_if.send_takeoff_async.assert_not_called()
            mock_if.send_land_async.assert_not_called()
            mock_if.publish_position_target.assert_not_called()
        finally:
            node.destroy_node()

    @patch('s500_mission_fsm.mission_node.MavrosInterface')
    def test_param_scenario_nan_inf_rejected(self, mock_interface_cls):
        """Senaryo 6: NaN / Inf içeren diziler reddedilmeli ve komut gönderilmemeli."""
        mock_if = mock_interface_cls.return_value
        node = S500MissionNode(parameter_overrides=[
            Parameter('custom_survey_waypoints_x', Parameter.Type.DOUBLE_ARRAY, [float('nan'), 2.0]),
            Parameter('custom_survey_waypoints_y', Parameter.Type.DOUBLE_ARRAY, [1.0, 2.0]),
        ])
        try:
            self.assertIsNotNone(node.param_validation_error)
            self.assertIn("NaN/Inf", node.param_validation_error)

            req = Trigger.Request()
            resp = Trigger.Response()
            result = node._srv_start_cb(req, resp)

            self.assertFalse(result.success)
            self.assertEqual(node.state, MissionState.GOREV_IPTAL)

            mock_if.send_set_mode_async.assert_not_called()
            mock_if.send_arm_async.assert_not_called()
            mock_if.send_takeoff_async.assert_not_called()
            mock_if.send_land_async.assert_not_called()
            mock_if.publish_position_target.assert_not_called()
        finally:
            node.destroy_node()

    @patch('s500_mission_fsm.mission_node.MavrosInterface')
    def test_param_scenario_bool_elements_rejected(self, mock_interface_cls):
        """Senaryo 7: [True]/[False] içeren boolean elemanları açıkça reddedilmeli ve komut gönderilmemeli."""
        mock_if = mock_interface_cls.return_value
        node = S500MissionNode(parameter_overrides=[
            Parameter('custom_survey_waypoints_x', Parameter.Type.BOOL_ARRAY, [True, False]),
            Parameter('custom_survey_waypoints_y', Parameter.Type.BOOL_ARRAY, [False, True]),
        ])
        try:
            self.assertIsNotNone(node.param_validation_error)
            self.assertIn("bool", node.param_validation_error)

            req = Trigger.Request()
            resp = Trigger.Response()
            result = node._srv_start_cb(req, resp)

            self.assertFalse(result.success)
            self.assertEqual(node.state, MissionState.GOREV_IPTAL)

            mock_if.send_set_mode_async.assert_not_called()
            mock_if.send_arm_async.assert_not_called()
            mock_if.send_takeoff_async.assert_not_called()
            mock_if.send_land_async.assert_not_called()
            mock_if.publish_position_target.assert_not_called()
        finally:
            node.destroy_node()

    # =========================================================================
    # 3. TEST_MODE NORMALIZATION & UNKNOWN MODE REJECTION
    # =========================================================================
    @patch('s500_mission_fsm.mission_node.MavrosInterface')
    def test_test_mode_normalization(self, mock_interface_cls):
        """Küçük harfli test_mode değeri (örn: 'hover_only') büyük harfe ('HOVER_ONLY') normalize edilmeli."""
        node = S500MissionNode(parameter_overrides=[
            Parameter('test_mode', Parameter.Type.STRING, 'hover_only')
        ])
        try:
            self.assertEqual(node.test_mode, 'HOVER_ONLY')
            self.assertIsNone(node.param_validation_error)
        finally:
            node.destroy_node()

    @patch('s500_mission_fsm.mission_node.MavrosInterface')
    def test_invalid_test_mode_rejects_mission_start_and_no_commands(self, mock_interface_cls):
        """Bilinmeyen test_mode ile başlatıldığında /mission/start reddedilmeli ve komut gönderilmemeli."""
        mock_if = mock_interface_cls.return_value
        node = S500MissionNode(parameter_overrides=[
            Parameter('test_mode', Parameter.Type.STRING, 'INVALID_TEST_MODE')
        ])
        try:
            self.assertEqual(node.test_mode, 'INVALID_TEST_MODE')
            self.assertIsNotNone(node.param_validation_error)
            self.assertIn("Geçersiz test_mode", node.param_validation_error)

            req = Trigger.Request()
            resp = Trigger.Response()
            result = node._srv_start_cb(req, resp)

            self.assertFalse(result.success)
            self.assertEqual(node.state, MissionState.GOREV_IPTAL)

            mock_if.send_set_mode_async.assert_not_called()
            mock_if.send_arm_async.assert_not_called()
            mock_if.send_takeoff_async.assert_not_called()
            mock_if.send_land_async.assert_not_called()
            mock_if.publish_position_target.assert_not_called()
        finally:
            node.destroy_node()

    # =========================================================================
    # 4. CONTROLLABLE CLOCK: HOLD TIMES, TELEMETRY DROPS & TIMEOUTS
    # =========================================================================
    @patch('s500_mission_fsm.mission_node.MavrosInterface')
    def test_alan_tarama_hold_duration_telemetry_gap_and_wp_start_time_refresh(self, mock_interface_cls):
        """
        Sahte saatle test:
        1. Bekleme süresi dolmadan waypoint ilerlememeli.
        2. Veri kesintisinde bekleme sayacı sıfırlanmalı; veri olmayan süre sayılamaz.
        3. Veri geri geldikten sonra tam bekleme süresi yeniden kesintisiz sağlanmalı.
        4. Waypoint tamamlandığında wp_start_time yeni saatle güncellenmeli.
        """
        clock = ControllableClock(start_time=1000.0)
        node = S500MissionNode()
        try:
            node.state = MissionState.ALAN_TARAMA
            node.survey_wps_x = [10.0, 20.0]
            node.survey_wps_y = [0.0, 0.0]
            node.current_wp_index = 0
            node.wp_hold = 2.0
            node.wp_radius = 1.0
            node.alt_tol = 1.0
            node.survey_alt = 32.5
            node.wp_start_time = clock.time()

            takeoff_pose = (0.0, 0.0, 0.0)
            wp0_pose = (10.0, 0.0, 32.5)
            node.interface.get_current_horizontal_speed = MagicMock(return_value=0.1)

            with patch('s500_mission_fsm.mission_node.time.time', side_effect=clock.time):
                # 1. Adım: Hedef penceresine gir (t=1000.0s)
                node._handle_alan_tarama(0.0, wp0_pose, takeoff_pose, 32.5)
                self.assertEqual(node.wp_arrival_time, 1000.0)
                self.assertEqual(node.current_wp_index, 0)

                # 2. Adım: 1.0s ilerle (t=1001.0s < 2.0s hold) -> Henüz ilerlememeli!
                clock.advance(1.0)
                node._handle_alan_tarama(1.0, wp0_pose, takeoff_pose, 32.5)
                self.assertEqual(node.current_wp_index, 0, "Hold süresi (2s) dolmadan WP ilerlememeli!")

                # 3. Adım: Telemetri kesildi (t=1001.5s, current_xyz=None) -> wp_arrival_time sıfırlanmalı!
                clock.advance(0.5)
                node._handle_alan_tarama(1.5, None, takeoff_pose, None)
                self.assertIsNone(node.wp_arrival_time, "Telemetri kesildiğinde wp_arrival_time SIFIRLANMALI!")
                self.assertEqual(node.current_wp_index, 0)

                # 4. Adım: Telemetri geri geldi (t=1002.0s) -> Sayaç sıfırdan başlamalı
                clock.advance(0.5)
                node._handle_alan_tarama(2.0, wp0_pose, takeoff_pose, 32.5)
                self.assertEqual(node.wp_arrival_time, 1002.0, "Telemetri gelince sayaç yeni saatten başlamalı!")
                self.assertEqual(node.current_wp_index, 0)

                # 5. Adım: 1.5s ilerle (t=1003.5s; t - wp_arrival_time = 1.5s < 2.0s) -> İlerlememeli!
                clock.advance(1.5)
                node._handle_alan_tarama(3.5, wp0_pose, takeoff_pose, 32.5)
                self.assertEqual(node.current_wp_index, 0, "Kesinti sonrası tam 2.0s kesintisiz bekleme gereklidir!")

                # 6. Adım: Tam bekleme tamamlandı (t=1004.1s; 1004.1 - 1002.0 = 2.1s >= 2.0s)
                clock.advance(0.6)
                node._handle_alan_tarama(4.1, wp0_pose, takeoff_pose, 32.5)
                self.assertEqual(node.current_wp_index, 1, "Kesintisiz 2.0s hold sonrasında WP 1'e geçilmeli!")
                self.assertIsNone(node.wp_arrival_time, "Yeni WP'ye geçildiğinde arrival_time sıfırlanmalı!")
                self.assertEqual(node.wp_start_time, 1004.1, "Yeni WP'de wp_start_time güncel saatle yenilenmeli!")
        finally:
            node.destroy_node()

    @patch('s500_mission_fsm.mission_node.MavrosInterface')
    def test_tarama_sonrasi_bekleme_complete_transitions_to_donus(self, mock_interface_cls):
        """
        TARAMA_SONRASI_BEKLEME durumunda sahte saatle kesintisiz bekleme (post_survey_hover_dur=10s)
        tamamlandığında KALKIS_NOKTASINA_DONUS durumuna geçilmelidir.
        """
        clock = ControllableClock(start_time=2000.0)
        node = S500MissionNode()
        try:
            node.state = MissionState.TARAMA_SONRASI_BEKLEME
            node.survey_wps_x = [20.0]
            node.survey_wps_y = [20.0]
            node.post_survey_hover_dur = 10.0
            node.post_survey_timeout_s = 30.0
            node.survey_alt = 32.5
            node.wp_radius = 1.0
            node.alt_tol = 1.0

            takeoff_pose = (0.0, 0.0, 0.0)
            last_wp_pose = (20.0, 20.0, 32.5)
            node.interface.get_current_horizontal_speed = MagicMock(return_value=0.1)

            with patch('s500_mission_fsm.mission_node.time.time', side_effect=clock.time):
                # 1. Bekleme başlasın (t=2000.0s)
                node._handle_tarama_sonrasi_bekleme(0.0, last_wp_pose, takeoff_pose, 32.5)
                self.assertEqual(node.post_survey_stable_start, 2000.0)
                self.assertEqual(node.state, MissionState.TARAMA_SONRASI_BEKLEME)

                # 2. 5 saniye ilerle (t=2005.0s < 10.0s)
                clock.advance(5.0)
                node._handle_tarama_sonrasi_bekleme(5.0, last_wp_pose, takeoff_pose, 32.5)
                self.assertEqual(node.state, MissionState.TARAMA_SONRASI_BEKLEME)

                # 3. Telemetri kopsun -> sayaç sıfırlansın
                clock.advance(1.0)
                node._handle_tarama_sonrasi_bekleme(6.0, None, takeoff_pose, None)
                self.assertIsNone(node.post_survey_stable_start)

                # 4. Telemetri geri gelsin (t=2007.0s)
                clock.advance(1.0)
                node._handle_tarama_sonrasi_bekleme(7.0, last_wp_pose, takeoff_pose, 32.5)
                self.assertEqual(node.post_survey_stable_start, 2007.0)

                # 5. Kesintisiz 10 saniye tamamlansın (t=2017.1s >= 2007.0 + 10.0)
                clock.advance(10.1)
                node._handle_tarama_sonrasi_bekleme(17.1, last_wp_pose, takeoff_pose, 32.5)
                self.assertEqual(
                    node.state,
                    MissionState.KALKIS_NOKTASINA_DONUS,
                    "10s kesintisiz kararlı bekleme sonrası KALKIS_NOKTASINA_DONUS durumuna geçilmeli!"
                )
        finally:
            node.destroy_node()

    @patch('s500_mission_fsm.mission_node.MavrosInterface')
    def test_waypoint_timeout_triggers_even_without_telemetry(self, mock_interface_cls):
        """Waypoint zaman aşımı sahte saatle wp_timeout aşılınca veri olmasa bile tetiklenmeli."""
        clock = ControllableClock(start_time=3000.0)
        node = S500MissionNode()
        try:
            node.state = MissionState.ALAN_TARAMA
            node.survey_wps_x = [10.0]
            node.survey_wps_y = [10.0]
            node.current_wp_index = 0
            node.wp_timeout = 10.0
            node.wp_start_time = clock.time()

            takeoff_pose = (0.0, 0.0, 0.0)

            with patch('s500_mission_fsm.mission_node.time.time', side_effect=clock.time):
                # 11 saniye ilerle (11.0 > wp_timeout 10.0), telemetri tamamen None
                clock.advance(11.0)
                node._handle_alan_tarama(11.0, None, takeoff_pose, None)

                self.assertEqual(node.state, MissionState.GOREV_IPTAL)
                self.assertIn("zaman aşımı", node.abort_reason)
        finally:
            node.destroy_node()

    @patch('s500_mission_fsm.mission_node.MavrosInterface')
    def test_return_to_takeoff_timeout_triggers_unconditionally(self, mock_interface_cls):
        """KALKIS_NOKTASINA_DONUS durumunda sahte saatle wp_timeout aşılınca iptal edilmeli."""
        node = S500MissionNode()
        try:
            node.state = MissionState.KALKIS_NOKTASINA_DONUS
            node.wp_timeout = 15.0
            takeoff_pose = (0.0, 0.0, 0.0)

            # elapsed=16.0 > wp_timeout=15.0 iken telemetri olmasa bile iptal edilmeli
            node._handle_donus(16.0, None, takeoff_pose, None)

            self.assertEqual(node.state, MissionState.GOREV_IPTAL)
            self.assertIn("zaman aşımına uğradı", node.abort_reason)
        finally:
            node.destroy_node()


class TestRealYamlParamsFileIntegration(unittest.TestCase):
    """
    Entegrasyon Testleri: ROS 2'nin gerçek '--ros-args --params-file' parametre yükleyicisi
    aracılığıyla mission_params.yaml dosyasının işlenmesi ve doğrulanması.
    """

    ACTUAL_YAML_PATH = "/home/ferruh/QUADCOPTER_TEST/mission_worktree/ros2_ws/src/s500_mission_fsm/config/mission_params.yaml"

    @patch('s500_mission_fsm.mission_node.MavrosInterface')
    def test_real_yaml_loading_via_params_file(self, mock_interface_cls):
        """
        Gerçek mission_params.yaml dosyasını '--ros-args --params-file' ile yükle:
        - custom_survey_waypoints_x ve y boş ve DOUBLE_ARRAY olmalı.
        - param_validation_error None olmalı.
        - 20x20m / 4.0m ayarlarıyla 6 şerit ve 12 waypoint üretilmeli.
        - Hiçbir mod, arm, kalkış, iniş veya konum hedefi komutu gönderilmemeli.
        """
        mock_if = mock_interface_cls.return_value
        ctx = Context()
        cli_args = ["--ros-args", "--params-file", self.ACTUAL_YAML_PATH]
        ctx.init(args=cli_args)

        try:
            node = S500MissionNode(context=ctx)
            try:
                # 1. Parametrelerin türü ve değerleri
                p_x = node.get_parameter('custom_survey_waypoints_x')
                p_y = node.get_parameter('custom_survey_waypoints_y')
                self.assertEqual(p_x.type_, Parameter.Type.DOUBLE_ARRAY, f"Beklenen DOUBLE_ARRAY, alınan: {p_x.type_}")
                self.assertEqual(p_y.type_, Parameter.Type.DOUBLE_ARRAY, f"Beklenen DOUBLE_ARRAY, alınan: {p_y.type_}")
                self.assertEqual(p_x.value, [])
                self.assertEqual(p_y.value, [])

                # 2. Hata durumu
                self.assertIsNone(node.param_validation_error, f"Beklenmeyen doğrulama hatası: {node.param_validation_error}")

                # 3. Üretilen rota (20x20m / 4m -> 6 şerit, 12 waypoint)
                self.assertEqual(len(node.survey_wps_x), 12, f"Beklenen 12 WP, üretilen: {len(node.survey_wps_x)}")
                self.assertEqual(len(node.survey_wps_y), 12)
                unique_y = sorted(list(set(node.survey_wps_y)))
                self.assertEqual(len(unique_y), 6, f"Beklenen 6 şerit (Y), üretilen: {len(unique_y)}")
                self.assertEqual(unique_y, [0.0, 4.0, 8.0, 12.0, 16.0, 20.0])

                # 4. Mod, arm, kalkış, iniş ve konum hedefi komutlarının gönderilmediğini doğrula
                mock_if.send_set_mode_async.assert_not_called()
                mock_if.send_arm_async.assert_not_called()
                mock_if.send_takeoff_async.assert_not_called()
                mock_if.send_land_async.assert_not_called()
                mock_if.publish_position_target.assert_not_called()
            finally:
                node.destroy_node()
        finally:
            ctx.shutdown()

    @patch('s500_mission_fsm.mission_node.MavrosInterface')
    def test_params_file_overrides_defaults_with_temp_yaml(self, mock_interface_cls):
        """
        Geçici YAML kopyasında varsayılandan farklı geçerli parametreler tanımlanarak
        testin sadece varsayılan değerlerle yanlışlıkla geçmediği kanıtlanır.
        (Asıl YAML dosyası değiştirilmez).
        """
        with open(self.ACTUAL_YAML_PATH, 'r') as f:
            yaml_content = yaml.safe_load(f)

        # Varsayılan değerlerden farklı geçerli değerler ata:
        # hover_duration_s: 5.0 -> 8.5
        # survey_lane_spacing_m: 4.0 -> 5.0 (20m / 5m = 4 aralık -> 5 şerit -> 10 WP)
        # survey_altitude_m: 32.5 -> 33.0
        yaml_content['/**']['ros__parameters']['hover_duration_s'] = 8.5
        yaml_content['/**']['ros__parameters']['survey_lane_spacing_m'] = 5.0
        yaml_content['/**']['ros__parameters']['survey_altitude_m'] = 33.0

        with tempfile.NamedTemporaryFile(mode='w', suffix='.yaml', delete=True) as temp_yaml:
            yaml.dump(yaml_content, temp_yaml)
            temp_yaml.flush()

            ctx = Context()
            cli_args = ["--ros-args", "--params-file", temp_yaml.name]
            ctx.init(args=cli_args)

            try:
                node = S500MissionNode(context=ctx)
                try:
                    # YAML dosyasından okunduğunu ve varsayılanların (5.0, 4.0, 32.5) ezildiğini doğrula
                    self.assertEqual(node.hover_dur, 8.5, f"hover_dur YAML'dan yüklenmedi! Alınan: {node.hover_dur}")
                    self.assertEqual(node.survey_lane_spacing, 5.0, f"spacing YAML'dan yüklenmedi! Alınan: {node.survey_lane_spacing}")
                    self.assertEqual(node.survey_alt, 33.0, f"survey_alt YAML'dan yüklenmedi! Alınan: {node.survey_alt}")

                    # 5m şerit aralığında 10 waypoint üretildiğini doğrula (varsayılan 12 değil)
                    self.assertEqual(len(node.survey_wps_x), 10, f"Beklenen 10 WP, üretilen: {len(node.survey_wps_x)}")
                    self.assertEqual(len(node.survey_wps_y), 10)
                    unique_y = sorted(list(set(node.survey_wps_y)))
                    self.assertEqual(unique_y, [0.0, 5.0, 10.0, 15.0, 20.0])
                    self.assertEqual(len(unique_y), 5)
                finally:
                    node.destroy_node()
            finally:
                ctx.shutdown()


if __name__ == '__main__':
    unittest.main()
