#!/usr/bin/env python3
"""
S500 Rota Planlayıcı (PathPlannerNode) ve GridPlanner Kapsamlı Birim Test Paketi
-------------------------------------------------------------------------------
Bu test dosyası:
1. GridPlanner Algoritmik Doğrulaması:
   - Çapraz köşe kesme koruması (diagonal cut check)
   - Morfolojik engel enflasyonu (cv2.dilate ve araç yarıçapı)
   - Engellenmiş başlangıç / hedef tespiti
   - Sınır dışı (out of bounds) koruması
2. PathPlannerNode Düğüm Davranışı (4 Zorunlu Senaryo):
   - Senaryo 1: Deterministik engel kaçınma ve çarpışmasız rota hesabı
   - Senaryo 2: Engellenmiş hedef (NO_PATH / GOAL_BLOCKED) ve Status 4 engeli
   - Senaryo 3: Geçersiz veri filtresi (position_valid=False, NaN/Inf koruması)
   - Senaryo 4: Start konumu kilitleme (jitter koruması) ve kesin FSM durum tetiklemesi
"""

import os
import sys
import math
import traceback
import numpy as np
import pytest
from unittest.mock import MagicMock, patch

# companion dizinini path'e ekle
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import rclpy
from rclpy.time import Time
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import String
from std_srvs.srv import Trigger
from vision_interfaces.msg import Detection, DetectionArray

from grid_path_planner import GridPlanner
from path_planner_node import PathPlannerNode


# ==============================================================================
# BÖLÜM 1: GridPlanner Algoritmik Birim Testleri
# ==============================================================================

def test_gridplanner_diagonal_cut_prevention():
    """1. Çapraz köşe kesme koruması: İki engel çapraz temas ettiğinde aralarından geçilmemelidir."""
    # 5x5 ızgara, enflasyonsuz (inflation_cells=0) test
    planner = GridPlanner(cell_size=1.0, vehicle_width=0.0, safety_margin=0.0)
    grid = np.zeros((5, 5), dtype=np.uint8)

    # (2, 1) ve (1, 2) konumlarına engel koyarak çapraz bir bariyer oluştur
    grid[1, 2] = GridPlanner.OBSTACLE
    grid[2, 1] = GridPlanner.OBSTACLE

    # Start: (1, 1), Goal: (2, 2)
    # Eğer çapraz kesmeye izin verilirse doğrudan (1,1) -> (2,2) geçebilir.
    # Çapraz koruma devredeyse bu aralıktan geçemez, engellerin etrafından dolaşmalıdır.
    resp, path, work_grid = planner.plan_path(grid, (1, 1), (2, 2))
    assert resp["status"] == "SUCCESS", "Rota bulunabilmelidir"
    assert path is not None

    # Yolun doğrudan (1,1) -> (2,2) adımını içermediğini doğrula
    path_tuples = list(zip(path[:-1], path[1:]))
    assert ((1, 1), (2, 2)) not in path_tuples, "Çapraz engel boşluğundan köşe kesilerek GEÇİLMEMELİDİR!"
    print("✓ Test 1: GridPlanner çapraz köşe kesme engellemesi başarıyla doğrulandı.")


def test_gridplanner_inflation_radius():
    """2. Morfolojik engel enflasyonu: Araç genişliği ve güvenlik payı kadar hücreler şişirilmelidir."""
    # res = 0.5m, veh_w = 0.6m, safety_m = 0.4m -> radius = 0.7m -> inflation_cells = ceil(0.7 / 0.5) = 2 hücre
    planner = GridPlanner(cell_size=0.5, vehicle_width=0.6, safety_margin=0.4)
    assert planner.inflation_cells == 2, f"Enflasyon hücresi 2 olmalıdır, hesaplanan: {planner.inflation_cells}"

    # Dinamik darboğaz algoritmasının sınır duvarı etkisini izole etmek için engeli ve noktaları sınırdan uzakta kur
    grid = np.zeros((21, 21), dtype=np.uint8)
    # Merkeze (10, 10) tek bir engel hücresi koy
    grid[10, 10] = GridPlanner.OBSTACLE

    resp, path, work_grid = planner.plan_path(grid, (3, 3), (17, 17))
    assert resp["status"] == "SUCCESS"
    assert resp["diagnostics"]["applied_inflation"] == 2

    # Enflasyon matrisinde (10, 10) merkezli 2 hücrelik yarıçaptaki komşular OBSTACLE (1) olmalıdır
    # (10, 10)'un hemen bitişiğindeki (10, 11), (11, 10) hücreleri 1 olmalı
    assert work_grid[10, 11] == 1, "Enflasyon hücresi (10, 11) dolu olmalıdır"
    assert work_grid[11, 10] == 1, "Enflasyon hücresi (11, 10) dolu olmalıdır"
    assert work_grid[10, 12] == 1, "2 hücre yarıçapındaki (10, 12) dolu olmalıdır"

    # Yeterince uzaktaki hücre serbest kalmalıdır
    assert work_grid[0, 0] == 0, "Uzak hücre serbest kalmalıdır"
    print("✓ Test 2: GridPlanner morfolojik engel enflasyonu (cv2.dilate) başarıyla doğrulandı.")


def test_gridplanner_blocked_start_and_goal():
    """3. Başlangıç veya hedefin engel üzerinde olması durumunda açık ret yanıtları."""
    planner = GridPlanner(cell_size=1.0, vehicle_width=0.0, safety_margin=0.0)
    grid = np.zeros((5, 5), dtype=np.uint8)
    grid[1, 1] = GridPlanner.OBSTACLE
    grid[3, 3] = GridPlanner.OBSTACLE

    # Start engelli
    resp_s, path_s, _ = planner.plan_path(grid, (1, 1), (4, 4))
    assert resp_s["status"] == "START_BLOCKED_PHYSICALLY"
    assert path_s is None

    # Goal engelli
    resp_g, path_g, _ = planner.plan_path(grid, (0, 0), (3, 3))
    assert resp_g["status"] == "GOAL_BLOCKED_PHYSICALLY"
    assert path_g is None
    print("✓ Test 3: GridPlanner START_BLOCKED_PHYSICALLY ve GOAL_BLOCKED_PHYSICALLY koruması başarıyla doğrulandı.")


def test_gridplanner_out_of_bounds():
    """4. Izgara sınırları dışındaki koordinatların reddedilmesi."""
    planner = GridPlanner(cell_size=1.0, vehicle_width=0.0, safety_margin=0.0)
    grid = np.zeros((5, 5), dtype=np.uint8)

    resp_s, _, _ = planner.plan_path(grid, (-1, 2), (3, 3))
    assert resp_s["status"] == "START_OUT_OF_BOUNDS"

    resp_g, _, _ = planner.plan_path(grid, (0, 0), (5, 3))  # w=5 için max indis 4
    assert resp_g["status"] == "GOAL_OUT_OF_BOUNDS"
    print("✓ Test 4: GridPlanner sınır dışı (out of bounds) denetimi başarıyla doğrulandı.")


# ==============================================================================
# BÖLÜM 2: PathPlannerNode Düğüm Birim Testleri
# ==============================================================================

def create_test_planner_node():
    if not rclpy.ok():
        rclpy.init()
    node = PathPlannerNode()
    status_events = []
    return_path_msgs = []
    path_msgs = []

    node.pub_status.publish = lambda msg: status_events.append(msg.data if hasattr(msg, 'data') else str(msg))
    node.pub_return_path.publish = lambda msg: return_path_msgs.append(msg)
    node.pub_path.publish = lambda msg: path_msgs.append(msg)

    return node, status_events, return_path_msgs, path_msgs


def make_detection(cls_name: str, x_enu: float, y_enu: float, valid: bool = True, conf: float = 0.9):
    d = Detection()
    d.class_name = cls_name
    d.confidence = float(conf)
    d.position_valid = bool(valid)
    d.local_x = float(x_enu)
    d.local_y = float(y_enu)
    return d


def test_node_scenario_1_deterministic_obstacle_avoidance():
    """Senaryo 1: Deterministik engel kaçınma ve çarpışmasız rota hesabı."""
    node, events, ret_msgs, path_msgs = create_test_planner_node()
    node.reset_state()

    # Start: ENU (0, 0) -> NED (0, 0)
    # Hedef: ENU (6, 6) -> NED (6, 6)
    # Engel: ENU (3, 3) -> NED (3, 3)
    det_msg = DetectionArray()
    det_msg.detections = [
        make_detection("start", 0.0, 0.0),
        make_detection("hedef", 6.0, 6.0),
        make_detection("engel", 3.0, 3.0)
    ]
    node.detections_callback(det_msg)

    # Doğrulamalar:
    assert node.start_pos_ned == (0.0, 0.0)
    assert node.goal_pos_ned == (6.0, 6.0)
    assert len(node.obstacles_ned) == 1
    assert any("PLANNING_SUCCESS" in ev for ev in events), "Çarpışmasız rota başarıyla hesaplanmalıdır"

    # Status 3 (Gözlem noktası) yayıncısı ve bayrağı olmamalı
    assert not hasattr(node, "pub_goto_obs")
    assert not hasattr(node, "observation_dispatched")

    # nav_msgs/Path yayınlanmış olmalı ve en az 2 nokta içermeli
    assert len(path_msgs) > 0
    assert len(path_msgs[-1].poses) >= 2

    # Rota noktalarının engelin (3.0, 3.0) içine girmediğini doğrula
    for p in path_msgs[-1].poses:
        dist_to_obs = math.hypot(p.pose.position.x - 3.0, p.pose.position.y - 3.0)
        # Güvenlik marjı ile engelin merkezine 0.5m'den yakın olmamalı
        assert dist_to_obs >= 0.5, f"Rota engele çok yakın ({dist_to_obs:.2f}m), çarpışma riski!"

    print("✓ Test 5 (Senaryo 1): Deterministik engel kaçınma ve YKI için rota yayını başarıyla doğrulandı.")


def test_node_scenario_2_blocked_goal_no_path():
    """Senaryo 2: Engellenmiş hedef (NO_PATH) durumunda rota üretilmemesi ve Status 4 engeli."""
    node, events, obs_msgs, path_msgs = create_test_planner_node()
    node.reset_state()

    # Hedefi (5, 5) etrafında tam bir duvarla çevrele
    obstacles = [
        make_detection("engel", 4.5, 5.0),
        make_detection("engel", 5.5, 5.0),
        make_detection("engel", 5.0, 4.5),
        make_detection("engel", 5.0, 5.5),
    ]
    det_msg = DetectionArray()
    det_msg.detections = [
        make_detection("start", 0.0, 0.0),
        make_detection("hedef", 5.0, 5.0),
    ] + obstacles

    node.cli_route_ready.call_async = MagicMock()
    node.detections_callback(det_msg)

    # Doğrulama: Rota planlama başarısız olmalı (GOAL_BLOCKED veya NO_PATH)
    assert any("PLANNING_FAILED" in ev for ev in events)
    assert len(node.last_path_ned) == 0, "Engellenmiş hedefe rota ÜRETİLMEMELİDİR!"
    # Asla ROUTE_READY çağrısı yapılmamalıdır
    node.cli_route_ready.call_async.assert_not_called()
    print("✓ Test 6 (Senaryo 2): Engellenmiş hedefte NO_PATH ve Status 4 engeli başarıyla doğrulandı.")


def test_node_scenario_3_invalid_and_stale_data_filtering():
    """Senaryo 3: position_valid=False ve NaN/Inf verilerin filtrelenmesi."""
    node, events, obs_msgs, path_msgs = create_test_planner_node()
    node.reset_state()

    # 1. position_valid = False olan hedef
    invalid_det = make_detection("hedef", 10.0, 10.0, valid=False)
    # 2. NaN koordinatlı engel
    nan_det = make_detection("engel", float('nan'), 5.0, valid=True)
    # 3. Inf koordinatlı start
    inf_det = make_detection("start", float('inf'), 0.0, valid=True)

    det_msg = DetectionArray()
    det_msg.detections = [invalid_det, nan_det, inf_det]
    node.detections_callback(det_msg)

    assert node.start_pos_ned is None, "Geçersiz start kaydedilmemelidir"
    assert node.goal_pos_ned is None, "position_valid=False olan hedef kaydedilmemelidir"
    assert len(node.obstacles_ned) == 0, "NaN koordinatlı engel kaydedilmemelidir"
    assert len(events) == 0, "Geçersiz tespitler için hiçbir olay üretilmemelidir"
    print("✓ Test 7 (Senaryo 3): Geçersiz ve güvenilmez veri filtreleme başarıyla doğrulandı.")


def test_node_scenario_4_start_lock_and_fsm_trigger_rules():
    """Senaryo 4: Start konumu kilitleme (jitter koruması) ve HEDEF_BEKLE'de kesin FSM tetikleme kuralı."""
    node, events, ret_msgs, path_msgs = create_test_planner_node()
    node.reset_state()

    # 1. İlk geçerli start tespiti
    det1 = DetectionArray()
    det1.detections = [make_detection("start", 1.0, 2.0)]
    node.detections_callback(det1)
    assert node.start_pos_ned == (2.0, 1.0)  # ENU -> NED (local_y, local_x)
    assert node.start_locked is True
    assert any("START_LOCKED:2.00:1.00" in ev for ev in events)

    # 2. Sonraki start tespiti (kamera titremesi sonucu farklı koordinat)
    det2 = DetectionArray()
    det2.detections = [make_detection("start", 1.8, 2.7)]  # Titreşim
    node.detections_callback(det2)
    # Start konumu KİLİTLİ kalmalı, değişmemelidir!
    assert node.start_pos_ned == (2.0, 1.0), "Kilitli start konumu kamera titreşimiyle DEĞİŞMEMELİDİR!"

    # 3. Oturum ve kalkış referansı kilitle
    node.current_session_id = 1001
    ref_msg = PoseStamped()
    ref_msg.header.stamp = node.get_clock().now().to_msg()
    ref_msg.header.frame_id = "ekf_origin_ned:session_1001"
    ref_msg.pose.position.x = 2.0
    ref_msg.pose.position.y = 1.0
    ref_msg.pose.position.z = -1.0
    node.takeoff_return_cb(ref_msg)

    # Otopilot HEDEF_BEKLE durumunda ve taze araç konumu veriliyor
    node.adapter_event_callback(String(data="OBSERVED_STATE:HEDEF_BEKLE"))
    pose_msg = PoseStamped()
    pose_msg.header.stamp = node.get_clock().now().to_msg()
    pose_msg.header.frame_id = "ekf_origin_ned:session_1001"
    pose_msg.pose.position.x = 2.0
    pose_msg.pose.position.y = 1.0
    pose_msg.pose.position.z = -34.0
    node.vehicle_pose_cb(pose_msg)

    node.cli_route_ready.service_is_ready = MagicMock(return_value=True)
    node.cli_route_ready.call_async = MagicMock()

    # 4. Hedef ekle ve rota üret -> Rota çıktığı anda HEDEF_BEKLE'de Status 4 DOĞRUDAN tetiklenmeli
    det3 = DetectionArray()
    det3.detections = [make_detection("hedef", 8.0, 8.0)]
    node.detections_callback(det3)

    assert len(node.last_path_ned) > 0
    # Ara durumlara (HEDEFE_GIT, HEDEF_KONUMUNDA_BEKLE) hiç gerek kalmadan çağrılmış olmalı!
    node.cli_route_ready.call_async.assert_called_once()
    assert node.route_ready_called is True
    assert any("ROUTE_READY_REQUESTED" in ev for ev in events)
    print("✓ Test 8 (Senaryo 4): Start kilitleme ve HEDEF_BEKLE'de doğrudan Status 4 tetiklemesi başarıyla doğrulandı.")


def test_gridplanner_boundary_wall_behavior_comparison():
    """5. Sınır dolgusu davranışı: strict_boundary_walls=False iken sınır hücreleri serbest, True iken duvar enflasyonuna dahil."""
    # Durum A: Açık hava uçuş sahası (strict_boundary_walls=False)
    # 10x10 tamamen boş bir ızgarada köşedeki (0, 0) ve (9, 9) asla bloke edilmemeli, rota bulunmalıdır.
    planner_open = GridPlanner(cell_size=0.5, vehicle_width=0.6, safety_margin=0.4, strict_boundary_walls=False)
    grid_empty = np.zeros((10, 10), dtype=np.uint8)
    resp_open, path_open, work_open = planner_open.plan_path(grid_empty, (0, 0), (9, 9))
    assert resp_open["status"] == "SUCCESS", "Açık hava sınırında (0, 0) ve (9, 9) serbest olmalı ve rota bulunmalıdır"
    assert work_open[0, 0] == 0, "Açık hava sınırında köşe hücresi sahte engele dönüşmemelidir"
    assert work_open[9, 9] == 0

    # Durum B: Katı duvarlı kapalı arena (strict_boundary_walls=True)
    # Aynı boş ızgarada sınırlar duvar sayıldığı için (0, 0) enflasyon yarıçapı içinde kalarak START_BLOCKED vermelidir.
    planner_strict = GridPlanner(cell_size=0.5, vehicle_width=0.6, safety_margin=0.4, strict_boundary_walls=True)
    resp_strict, path_strict, work_strict = planner_strict.plan_path(grid_empty, (0, 0), (9, 9))
    assert resp_strict["status"] == "START_BLOCKED", "Katı sınır modunda köşeler duvar enflasyonuyla bloke edilmelidir"
    assert work_strict[0, 0] == 1, "Katı sınır modunda köşe hücresi duvar enflasyonuna dahil olmalıdır"
    print("✓ Test 9: GridPlanner sınır dolgusu davranışı (açık hava serbestliği vs katı duvar) başarıyla doğrulandı.")


def test_ready_route_stale_position_blocks_status4():
    """10. Hazır rota varken eski (bayat, dt > 3.0s) konumda Status 4'ün engellenmesi."""
    node, events, ret_msgs, path_msgs = create_test_planner_node()
    node.reset_state()
    node.current_session_id = 2002
    node.takeoff_return_pos_ned = (0.0, 0.0)
    node.takeoff_return_pos_ned_3d = (0.0, 0.0, 0.0)
    node.goal_pos_ned = (10.0, 10.0)
    node.lua_fsm_state = "HEDEF_BEKLE"
    node.cli_route_ready.service_is_ready = MagicMock(return_value=True)
    node.cli_route_ready.call_async = MagicMock()

    # Hazır dönüş rotası
    node.return_path_ned = [(2.0, 2.0), (0.0, 0.0)]

    # Araç konumu zaman damgası eski (10s önce)
    old_time = Time(seconds=node.get_clock().now().seconds_nanoseconds()[0] - 10)
    pose_stale = PoseStamped()
    pose_stale.header.stamp = old_time.to_msg()
    pose_stale.header.frame_id = "ekf_origin_ned:session_2002"
    pose_stale.pose.position.x = 2.0
    pose_stale.pose.position.y = 2.0
    pose_stale.pose.position.z = -33.0
    node.vehicle_pose_cb(pose_stale)
    node.check_and_unlock_return_path()
    node.cli_route_ready.call_async.assert_not_called()
    print("✓ Test 10: Hazır rota varken eski konumda Status 4 blokajı doğrulandı.")


def test_valid_arrival_single_trigger_and_no_trigger_in_hedefe_git():
    """11. HEDEF_BEKLE'de tek tetikleme; tekrarlayan konumlarda ikinci çağrı olmaması."""
    node, events, ret_msgs, path_msgs = create_test_planner_node()
    node.reset_state()
    node.current_session_id = 3003
    node.takeoff_return_pos_ned = (0.0, 0.0)
    node.takeoff_return_pos_ned_3d = (0.0, 0.0, 0.0)
    node.goal_pos_ned = (5.0, 5.0)
    node.cli_route_ready.service_is_ready = MagicMock(return_value=True)
    node.cli_route_ready.call_async = MagicMock()

    # HEDEFE_GIT durumundayken konum gelse bile tetikleme OLMAMALI
    node.lua_fsm_state = "HEDEFE_GIT"
    pose = PoseStamped()
    pose.header.stamp = node.get_clock().now().to_msg()
    pose.header.frame_id = "ekf_origin_ned:session_3003"
    pose.pose.position.x = 2.0
    pose.pose.position.y = 2.0
    pose.pose.position.z = -33.0
    node.vehicle_pose_cb(pose)
    node.cli_route_ready.call_async.assert_not_called()

    # HEDEF_BEKLE durumuna geçildiğinde tetiklenmeli
    node.adapter_event_callback(String(data="OBSERVED_STATE:HEDEF_BEKLE"))
    node.cli_route_ready.call_async.assert_called_once()

    # Tekrarlanan callback'ler ikinci bir çağrı üretmemeli!
    node.adapter_event_callback(String(data="OBSERVED_STATE:HEDEF_BEKLE"))
    node.vehicle_pose_cb(pose)
    node.cli_route_ready.call_async.assert_called_once()
    print("✓ Test 11: HEDEF_BEKLE'de tek tetikleme ve HEDEFE_GIT koruması başarıyla doğrulandı.")


def test_return_path_planning_and_altitude_and_obstacles():
    """12. Dönüş rotasının bağımsız A*, fiziksel kalkış referansı, ref_z - 33 irtifası ve engel kontrolü."""
    node, events, obs_msgs, path_msgs = create_test_planner_node()
    return_path_msgs = []
    node.pub_return_path.publish = lambda msg: return_path_msgs.append(msg)
    node.reset_state()
    node.current_session_id = 4004

    # Kalkış referansı: z = -5.0m -> cruise_z = -5.0 - 33.0 = -38.0m olmalı!
    node.takeoff_return_pos_ned = (0.0, 0.0)
    node.takeoff_return_pos_ned_3d = (0.0, 0.0, -5.0)

    # Dönüş rotası üzerinde (3, 3) noktasına engel koy
    node.obstacles_ned = [(3.0, 3.0)]

    # (6, 6) noktasından (0, 0) kalkış referansına dönüş planla
    ok, msg = node.plan_return_path((6.0, 6.0))
    assert ok is True
    assert len(return_path_msgs) > 0
    ret_path = return_path_msgs[-1]

    # Başlangıç (6, 6) ve Bitiş (0, 0) kontrolü
    assert math.isclose(ret_path.poses[0].pose.position.x, 6.0, abs_tol=1e-2)
    assert math.isclose(ret_path.poses[0].pose.position.y, 6.0, abs_tol=1e-2)
    assert math.isclose(ret_path.poses[-1].pose.position.x, 0.0, abs_tol=1e-2)
    assert math.isclose(ret_path.poses[-1].pose.position.y, 0.0, abs_tol=1e-2)

    # İrtifa sözleşmesi: her waypoint'in z'si ref_z - 33 = -38.0 olmalı!
    for p in ret_path.poses:
        assert math.isclose(p.pose.position.z, -38.0, abs_tol=1e-2)

    # Engelden kaçınma kontrolü: hiçbir nokta engele (3, 3) 0.5m'den yakın olmamalı
    for p in ret_path.poses:
        dist_obs = math.hypot(p.pose.position.x - 3.0, p.pose.position.y - 3.0)
        assert dist_obs >= 0.5
    print("✓ Test 12: Dönüş rotası A*, kalkış referansı, ref_z - 33 irtifası ve engel kaçınma başarıyla doğrulandı.")


def test_planning_failure_invalidates_adapter_route():
    """13. Planlama başarısızlığında boş rotanın yayınlanması ve adaptördeki rotanın temizlenmesi."""
    node, events, obs_msgs, path_msgs = create_test_planner_node()
    return_path_msgs = []
    node.pub_return_path.publish = lambda msg: return_path_msgs.append(msg)
    node.reset_state()
    node.current_session_id = 5005

    # Önceden kabul edilmiş rota simülasyonu
    node.return_path_ned = [(5.0, 5.0), (0.0, 0.0)]

    # İptal et
    node.invalidate_return_path("TEST_FAILURE")
    assert len(node.return_path_ned) == 0
    assert len(return_path_msgs) > 0
    assert len(return_path_msgs[-1].poses) == 0  # Boş rota yayınlandı
    assert f"session_5005" in return_path_msgs[-1].header.frame_id
    print("✓ Test 13: Planlama başarısızlığında boş rota yayını ve iptal mekanizması başarıyla doğrulandı.")


def test_direct_status4_trigger_in_hedef_bekle_on_route_ready():
    """14. Yeni İzole Senaryo: HEDEF_BEKLE durumundayken rota hesaplandığı an doğrudan Status 4 tetiklenmeli."""
    node, events, ret_msgs, path_msgs = create_test_planner_node()
    node.reset_state()
    node.current_session_id = 9001
    node.lua_fsm_state = "HEDEF_BEKLE"
    node.cli_route_ready.service_is_ready = MagicMock(return_value=True)
    node.cli_route_ready.call_async = MagicMock()

    # Kalkış referansı
    ref_msg = PoseStamped()
    ref_msg.header.stamp = node.get_clock().now().to_msg()
    ref_msg.header.frame_id = "ekf_origin_ned:session_9001"
    ref_msg.pose.position.x = 0.0
    ref_msg.pose.position.y = 0.0
    ref_msg.pose.position.z = 0.0
    node.takeoff_return_cb(ref_msg)

    # Taze araç konumu (dronun HEDEF_BEKLE'deki konumu)
    pose = PoseStamped()
    pose.header.stamp = node.get_clock().now().to_msg()
    pose.header.frame_id = "ekf_origin_ned:session_9001"
    pose.pose.position.x = 2.0
    pose.pose.position.y = 2.0
    pose.pose.position.z = -33.0
    node.vehicle_pose_cb(pose)

    # Start ve Goal tespiti verip rota hesaplat
    # Start tespiti ENU (1,0) -> NED (0,1) iken kilitli kalkış referansı NED (0,0) kalır
    det_msg = DetectionArray()
    det_msg.detections = [
        make_detection("start", 1.0, 0.0),
        make_detection("hedef", 8.0, 8.0)
    ]
    # Tespit gelmeden önce erken ROUTE_READY çağrısı yapılmamalıdır
    node.cli_route_ready.call_async.assert_not_called()
    node.detections_callback(det_msg)

    # Doğrulamalar:
    # 1. Rota hesaplanmış olmalı
    assert len(node.last_path_ned) >= 2
    # 2. Dönüş rotası araç konumundan (2.0, 2.0) kalkış referansına (0.0, 0.0) yayınlanmış olmalı
    assert len(node.return_path_ned) >= 2
    assert math.isclose(node.return_path_ned[0][0], 2.0, abs_tol=1e-2)
    assert math.isclose(node.return_path_ned[0][1], 2.0, abs_tol=1e-2)
    assert math.isclose(node.return_path_ned[-1][0], 0.0, abs_tol=1e-2)
    assert math.isclose(node.return_path_ned[-1][1], 0.0, abs_tol=1e-2)
    # 3. Status 4 derhal ve 1 kez çağrılmış olmalı
    node.cli_route_ready.call_async.assert_called_once()
    assert node.route_ready_called is True
    print("✓ Test 14: HEDEF_BEKLE'de rota hazır olduğu an doğrudan Status 4 tetiklemesi başarıyla doğrulandı.")


def test_status3_never_sent_under_any_condition():
    """15. Status 3'ün (GOTO_OBSERVATION) hiçbir koşulda gönderilmediğinin doğrulanması."""
    node, events, ret_msgs, path_msgs = create_test_planner_node()
    node.reset_state()

    # Düğüm üzerinde /mission/goto_observation yayıncısı veya observation_dispatched olmamalı
    assert not hasattr(node, "pub_goto_obs"), "PathPlannerNode üzerinde pub_goto_obs publisher'ı olmamalıdır"
    assert not hasattr(node, "dispatch_goto_observation"), "dispatch_goto_observation fonksiyonu kaldırılmış olmalıdır"
    assert not hasattr(node, "observation_dispatched"), "observation_dispatched bayrağı kaldırılmış olmalıdır"

    # Rota planlaması çalışsa bile Status 3 yayını veya status mesajı üretilmemeli
    det_msg = DetectionArray()
    det_msg.detections = [
        make_detection("start", 0.0, 0.0),
        make_detection("hedef", 6.0, 6.0)
    ]
    node.detections_callback(det_msg)
    assert not any("OBSERVATION_DISPATCHED" in ev for ev in events), "Hiçbir event'te OBSERVATION_DISPATCHED olmamalıdır"
    print("✓ Test 15: Status 3'ün hiçbir koşulda gönderilmediği başarıyla doğrulandı.")


def test_return_path_fails_when_takeoff_goal_blocked_by_physical_obstacles():
    """16. Dönüş senaryosunda kilitli kalkış hedefi fiziksel engellerle çevriliyse rota üretilmemeli."""
    node, events, ret_msgs, path_msgs = create_test_planner_node()
    node.reset_state()
    node.current_session_id = 9002

    # Kilitli kalkış referansı (hedef): NED (0.0, 0.0)
    node.takeoff_return_pos_ned = (0.0, 0.0)
    node.takeoff_return_pos_ned_3d = (0.0, 0.0, 0.0)

    # Kalkış noktasının (0.0, 0.0) 4 tarafını fiziksel engellerle tamamen kapat (0.5 m mesafede)
    node.obstacles_ned = [
        (0.5, 0.0),
        (-0.5, 0.0),
        (0.0, 0.5),
        (0.0, -0.5),
    ]

    # reset_state() ve kurulum sonrasında listeleri temizle (sahte assert geçişlerini önle)
    events.clear()
    ret_msgs.clear()
    path_msgs.clear()

    # Dron serbest konumdan (6.0, 6.0) kalkış referansına dönüş planlamaya çalışır
    ok, msg = node.plan_return_path((6.0, 6.0))
    print(f"Test 16 return status: ok={ok}, msg={msg}")

    # Doğrulamalar:
    assert ok is False
    assert msg == "NO_RETURN_PATH:NO_PATH", f"Hata engel kuşatmasından (NO_RETURN_PATH:NO_PATH) gelmeli, dönen: {msg}"
    assert "START_BLOCKED" not in msg and "GOAL_BLOCKED" not in msg and "BLOCKED_PHYSICALLY" not in msg, \
        "Start veya Goal doğrudan engelli olmamalı, kuşatmadan kaynaklanmalı"
    assert any("RETURN_PLANNING_FAILED" in ev for ev in events), "RETURN_PLANNING_FAILED eventi üretilmelidir"
    assert len(node.return_path_ned) == 0, "Dönüş rotası listesi boş kalmalıdır"
    assert len(ret_msgs) > 0 and len(ret_msgs[-1].poses) == 0, "Temizlemeden sonra adaptöre boş Path yayınlanmış olmalıdır"
    print("✓ Test 16: Fiziksel olarak engelli kalkış konumuna dönüş rotası blokajı başarıyla doğrulandı.")


def test_physical_obstacle_retained_after_start_goal_inflation_relaxation():
    """17. Start ve goal çevresindeki enflasyon gevşetmesinden sonra fiziksel engellerin silinmediğinin doğrulanması."""
    planner = GridPlanner(cell_size=0.5, vehicle_width=0.6, safety_margin=0.4)
    grid = np.zeros((21, 21), dtype=np.uint8)
    start = (6, 6)
    goal = (15, 15)

    # Start'ın hemen yanına [y, x] = [6, 7] ve Goal'ün hemen yanına [15, 14] fiziksel engel koy
    grid[6, 7] = GridPlanner.OBSTACLE
    grid[15, 14] = GridPlanner.OBSTACLE

    resp, path, work_grid = planner.plan_path(grid, start, goal)

    # 1. Başlangıçta doğrudan engelli sayılmamalı, gevşetme dalına ulaşmış olmalı
    assert resp["status"] not in ("START_BLOCKED", "GOAL_BLOCKED", "START_BLOCKED_PHYSICALLY", "GOAL_BLOCKED_PHYSICALLY"), \
        f"Gevşetme dalına ulaşmadan bloke edilmemeli, status: {resp['status']}"
    # 2. Gevşetme dalı gerçekten çalışmış olmalı
    assert resp["diagnostics"]["applied_inflation"] > 0, "Enflasyon uygulanmış ve gevşetme dalı çalışmış olmalıdır"
    # 3. Engel hücreleri hem work_grid hem orijinal grid üzerinde korunmalıdır
    assert work_grid[6, 7] == 1, "Start bitişiğindeki fiziksel engel work_grid'de OBSTACLE (1) kalmalıdır"
    assert work_grid[15, 14] == 1, "Goal bitişiğindeki fiziksel engel work_grid'de OBSTACLE (1) kalmalıdır"
    assert grid[6, 7] == GridPlanner.OBSTACLE
    assert grid[15, 14] == GridPlanner.OBSTACLE
    print("✓ Test 17: Start/goal çevresi gevşetmesinde fiziksel engellerin korunduğu başarıyla doğrulandı.")


def run_all_tests():
    print("\n" + "="*75)
    print("S500 ROTA PLANLAYICI & GRIDPLANNER BİRİM TESTLERİ")
    print("="*75)
    tests = [
        test_gridplanner_diagonal_cut_prevention,
        test_gridplanner_inflation_radius,
        test_gridplanner_blocked_start_and_goal,
        test_gridplanner_out_of_bounds,
        test_gridplanner_boundary_wall_behavior_comparison,
        test_node_scenario_1_deterministic_obstacle_avoidance,
        test_node_scenario_2_blocked_goal_no_path,
        test_node_scenario_3_invalid_and_stale_data_filtering,
        test_node_scenario_4_start_lock_and_fsm_trigger_rules,
        test_ready_route_stale_position_blocks_status4,
        test_valid_arrival_single_trigger_and_no_trigger_in_hedefe_git,
        test_return_path_planning_and_altitude_and_obstacles,
        test_planning_failure_invalidates_adapter_route,
        test_direct_status4_trigger_in_hedef_bekle_on_route_ready,
        test_status3_never_sent_under_any_condition,
        test_return_path_fails_when_takeoff_goal_blocked_by_physical_obstacles,
        test_physical_obstacle_retained_after_start_goal_inflation_relaxation,
    ]
    passed = 0
    failed = 0
    for t in tests:
        try:
            t()
            passed += 1
        except Exception as e:
            failed += 1
            print(f"✗ FAIL: {t.__name__} -> {e}")
            traceback.print_exc()
    print("="*75)
    print(f"TEST SONUÇLARI: {passed} / {len(tests)} BAŞARILI | {failed} BAŞARISIZ")
    print("="*75 + "\n")
    if failed > 0:
        sys.exit(1)


if __name__ == '__main__':
    run_all_tests()
