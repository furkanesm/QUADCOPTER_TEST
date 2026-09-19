"""
Unit tests for S500MissionNode logic:
1. Altitude rules & bounds (30-35m cruise band, 35m ceiling, exemptions in takeoff/landing).
2. Pre-validation of target coordinates (reject out-of-bounds targets before sending).
3. Pilot takeover handling (cease motion commands, no auto-re-GUIDED).
4. Telemetry loss & no auto-resume upon reconnection.
5. Landing completion dual-criteria (ON_GROUND + DISARMED).
6. Explicit start trigger (no auto-arming on startup).
"""

import time
import pytest
from unittest.mock import MagicMock, patch

import rclpy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State, ExtendedState

from s500_mission_fsm.mission_node import S500MissionNode, MissionState
from s500_mission_fsm.mavros_interface import LandedStatus


@pytest.fixture(scope="module")
def rclpy_init():
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture
def mission_node(rclpy_init):
    node = S500MissionNode()
    # Mock MAVROS interface methods
    node.interface.send_set_mode_async = MagicMock()
    node.interface.send_arm_async = MagicMock()
    node.interface.send_takeoff_async = MagicMock()
    node.interface.send_land_async = MagicMock()
    node.interface.publish_position_target = MagicMock()
    return node


# ====================================================================
# TEST 1: BAŞLANGIÇTA BEKLEME VE AÇIK TETİKLEME GEREKSİNİMİ
# ====================================================================

def test_initial_state_is_bekleme_no_auto_arm(mission_node):
    """Kural: Node başlayınca otomatik arm/kalkış yapmamalı; BEKLEME durumunda kalmalıdır."""
    assert mission_node.state == MissionState.BEKLEME

    # FSM döngüsü çalışsa bile hiçbir komut gitmemeli
    mission_node._fsm_loop()
    assert mission_node.state == MissionState.BEKLEME
    mission_node.interface.send_arm_async.assert_not_called()
    mission_node.interface.send_takeoff_async.assert_not_called()


def test_explicit_start_service_transitions_to_hazirlik(mission_node):
    """Kural: /mission/start servisi açıkça çağrıldığında HAZIRLIK durumuna geçmelidir."""
    mission_node.state = MissionState.BEKLEME

    req = MagicMock()
    resp = MagicMock()
    res = mission_node._srv_start_cb(req, resp)

    assert res.success is True
    assert mission_node.state == MissionState.HAZIRLIK


# ====================================================================
# TEST 2: HEDEF İRTİFA ÖN-DOĞRULAMASI (KOMUT GÖNDERİLMEDEN RET)
# ====================================================================

def test_validate_target_altitude_rejects_out_of_bounds(mission_node):
    """Kural: 30-35m bandı dışındaki seyir hedefleri komut gönderilmeden reddedilmelidir."""
    # 29.9m (< 30.0m) -> Red
    assert mission_node._validate_target_altitude(29.9) is False
    # 35.1m (> 35.0m) -> Red
    assert mission_node._validate_target_altitude(35.1) is False
    # 5.0m (eski hedef) -> Kesinlikle Red
    assert mission_node._validate_target_altitude(5.0) is False

    # 32.5m (nominal hedef) -> Kabul
    assert mission_node._validate_target_altitude(32.5) is True
    # 30.0m ve 35.0m (sınır noktaları) -> Kabul
    assert mission_node._validate_target_altitude(30.0) is True
    assert mission_node._validate_target_altitude(35.0) is True


# ====================================================================
# TEST 3: GERÇEK TELEMETRİDE İRTİFA BANDI DENETİMİ
# ====================================================================

def test_ceiling_violation_triggers_irtifa_ihlali_in_any_state(mission_node):
    """Kural: 35.0m tavan sınırı her durumda (kalkış dahil) aşılırsa IRTITA_IHLALI tetiklenir."""
    mission_node.state = MissionState.KALKIS

    # 35.2m (> 35.0m) ihlali
    ok = mission_node._check_altitude_bounds(35.2)

    assert ok is False
    assert mission_node.state == MissionState.IRTITA_IHLALI
    assert "35m Tavan İhlali" in mission_node.abort_reason
    # Tepe irtifa kaydedilmiş olmalı
    assert mission_node.max_mission_rel_alt_observed >= 35.2


def test_cruise_floor_violation_triggers_irtifa_ihlali(mission_node):
    """Kural: Seyir sırasında (HAVADA_BEKLEME veya KISA_ROTA) irtifa < 30.0m olursa ihlal tetiklenir."""
    mission_node.state = MissionState.KISA_ROTA

    # 29.5m (< 30.0m)
    ok = mission_node._check_altitude_bounds(29.5)

    assert ok is False
    assert mission_node.state == MissionState.IRTITA_IHLALI
    assert "30m Seyir Tabanı İhlali" in mission_node.abort_reason


def test_takeoff_and_landing_are_exempt_from_30m_floor(mission_node):
    """Kural: Kalkış ve inişte 30m altına inilmesi YANLIŞLIKLA ihlal OLUŞTURMAMALIDIR."""
    # Kalkışta 10 metrede tırmanırken:
    mission_node.state = MissionState.KALKIS
    assert mission_node._check_altitude_bounds(10.0) is True
    assert mission_node.state == MissionState.KALKIS

    # İnişte 5 metrede alçalırken:
    mission_node.state = MissionState.INIS
    assert mission_node._check_altitude_bounds(5.0) is True
    assert mission_node.state == MissionState.INIS


# ====================================================================
# TEST 4: PİLOT MÜDAHALESİ (TAKEOVER) VE GÜVENLİK
# ====================================================================

def test_pilot_takeover_ceases_autonomous_commands_no_auto_re_guided(mission_node):
    """Kural: Pilot uçuş modunu GUIDED dışına aldığında otonom komutlar ve tekrar GUIDED isteme kesilmelidir."""
    mission_node.state = MissionState.KISA_ROTA

    # Telemetri canlı
    mission_node.interface.is_telemetry_healthy = MagicMock(return_value=True)
    # Pilot kumandadan LOITER seçti
    mission_node.interface.get_flight_mode = MagicMock(return_value="LOITER")

    mission_node._fsm_loop()

    assert mission_node.state == MissionState.PILOT_MUDAHALESI
    # Asla otonom setpoint basılmamalı
    mission_node.interface.publish_position_target.assert_not_called()
    # Asla tekrar GUIDED istenmemeli
    mission_node.interface.send_set_mode_async.assert_not_called()


# ====================================================================
# TEST 5: BAĞLANTI KOPMASI VE YENİDEN BAĞLANTIDA OTOMATİK DEVAM ETMEME
# ====================================================================

def test_telemetry_loss_aborts_mission_and_does_not_auto_resume(mission_node):
    """Kural: Telemetri kesintisi görevi iptal eder; bağlantı geri gelse de otomatik devam etmez."""
    mission_node.state = MissionState.KISA_ROTA

    # Telemetri kesildi
    mission_node.interface.is_telemetry_healthy = MagicMock(return_value=False)
    mission_node._fsm_loop()

    assert mission_node.state == MissionState.GOREV_IPTAL
    assert "Telemetri kesildi" in mission_node.abort_reason

    # Bağlantı geri geldiğini varsayalım
    mission_node.interface.is_telemetry_healthy = MagicMock(return_value=True)
    mission_node._fsm_loop()

    # Durum kesinlikle GOREV_IPTAL olarak kalmalı, kendiliğinden KISA_ROTA'ya dönmemeli!
    assert mission_node.state == MissionState.GOREV_IPTAL


# ====================================================================
# TEST 6: İNİŞİN ÇİFT KATMANLI DOĞRULANMASI (ON_GROUND + DISARMED)
# ====================================================================

def test_landing_requires_both_on_ground_and_disarmed(mission_node):
    """Kural: İnişin tamamlanması için hem teyitli ON_GROUND hem de DISARMED doğrulanmalıdır."""
    mission_node.state = MissionState.INIS

    # Durum 1: Motorlar disarmed oldu ama ON_GROUND teyidi henüz yok -> TAMAMLANDI'ya GEÇEMEZ
    mission_node.interface.is_confirmed_landed = MagicMock(return_value=False)
    mission_node.interface.is_armed = MagicMock(return_value=False)
    mission_node._handle_inis(elapsed=10.0)
    assert mission_node.state == MissionState.INIS

    # Durum 2: Yere temas etti (ON_GROUND) ama motorlar hala dönüyor (ARMED) -> TAMAMLANDI'ya GEÇEMEZ
    mission_node.interface.is_confirmed_landed = MagicMock(return_value=True)
    mission_node.interface.is_armed = MagicMock(return_value=True)
    mission_node._handle_inis(elapsed=12.0)
    assert mission_node.state == MissionState.INIS

    # Durum 3: Hem ON_GROUND teyitli hem motorlar DISARMED -> TAMAMLANDI
    mission_node.interface.is_confirmed_landed = MagicMock(return_value=True)
    mission_node.interface.is_armed = MagicMock(return_value=False)
    mission_node._handle_inis(elapsed=15.0)
    assert mission_node.state == MissionState.TAMAMLANDI
