"""
Unit tests for MavrosInterface safety guarantees:
1. Explicit Takeoff Pose Locking (no accidental latching in callbacks, ground & stability checks).
2. Grounded State Confirmation (no disarmed == landed assumptions, strict UNKNOWN on stale/missing).
3. Independent Velocity & Telemetry Freshness (no returning 0.0 on stale data, strict None).
"""

import time
import pytest
from unittest.mock import MagicMock

from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import State, ExtendedState

from s500_mission_fsm.mavros_interface import MavrosInterface, LandedStatus


@pytest.fixture
def mock_node():
    node = MagicMock()
    node.create_subscription.return_value = MagicMock()
    node.create_publisher.return_value = MagicMock()
    node.create_client.return_value = MagicMock()
    node.get_logger().info = MagicMock()
    node.get_logger().error = MagicMock()
    node.get_logger().warn = MagicMock()
    return node


@pytest.fixture
def interface(mock_node):
    return MavrosInterface(mock_node)


# ====================================================================
# TEST GRUBU 1: KALKIŞ KONUMU KİLİTLEME GÜVENLİĞİ
# ====================================================================

def test_pose_cb_does_not_auto_lock_takeoff(interface):
    """Kural 1: _pose_cb içinde ilk gelen konum ASLA otomatik olarak kalkış noktası yapılamaz."""
    msg = PoseStamped()
    msg.pose.position.x = 10.0
    msg.pose.position.y = 20.0
    msg.pose.position.z = 0.0

    interface._pose_cb(msg)

    assert interface.is_takeoff_pose_locked() is False
    assert interface.get_takeoff_pose() is None


def test_capture_takeoff_pose_fails_if_stale_or_missing(interface):
    """Kural 1b: Konum verisi hiç gelmemiş veya eskiyse kalkış konumu kilitlenemez."""
    success, reason = interface.capture_takeoff_pose(sample_count=3, max_age_s=1.0)
    assert success is False
    assert "güncel değil" in reason


def test_capture_takeoff_pose_fails_if_not_confirmed_on_ground(interface):
    """Kural 1c: Araç yerde doğrulanmadıysa (örn. ExtendedState eksikse) kalkış konumu kilitlenemez."""
    for _ in range(5):
        msg = PoseStamped()
        msg.pose.position.x = 1.0
        msg.pose.position.y = 2.0
        msg.pose.position.z = 0.1
        interface._pose_cb(msg)

    # İniş/yer durumu henüz gelmedi (LandedStatus.UNKNOWN)
    success, reason = interface.capture_takeoff_pose(sample_count=3, check_landed=True)
    assert success is False
    assert "Araç yerde doğrulanmadı" in reason


def test_capture_takeoff_pose_fails_if_unstable_variance(interface):
    """Kural 1d: Konum örnekleri gürültülü veya dalgalıysa (varyans > eşik) kilitlenemez."""
    ext_msg = ExtendedState()
    ext_msg.landed_state = ExtendedState.LANDED_STATE_ON_GROUND
    interface._ext_state_cb(ext_msg)

    coords = [(0.0, 0.0, 0.0), (1.5, 0.0, 0.0), (0.2, 0.0, 0.0)]
    for x, y, z in coords:
        msg = PoseStamped()
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = z
        interface._pose_cb(msg)

    success, reason = interface.capture_takeoff_pose(sample_count=3, max_variance_m=0.3, check_landed=True)
    assert success is False
    assert "kararsız" in reason


def test_capture_takeoff_pose_succeeds_and_cannot_be_overwritten(interface):
    """Kural 1e: Şartlar sağlandığında ortalama kilitlenir ve tekrar üzerine yazılamaz."""
    ext_msg = ExtendedState()
    ext_msg.landed_state = ExtendedState.LANDED_STATE_ON_GROUND
    interface._ext_state_cb(ext_msg)

    for _ in range(5):
        msg = PoseStamped()
        msg.pose.position.x = 2.0
        msg.pose.position.y = 4.0
        msg.pose.position.z = 0.05
        interface._pose_cb(msg)

    success, msg_str = interface.capture_takeoff_pose(sample_count=5, max_variance_m=0.2, check_landed=True)
    assert success is True
    assert interface.is_takeoff_pose_locked() is True

    locked = interface.get_takeoff_pose()
    assert locked is not None
    assert pytest.approx(locked[0], abs=1e-3) == 2.0
    assert pytest.approx(locked[1], abs=1e-3) == 4.0
    assert pytest.approx(locked[2], abs=1e-3) == 0.05

    # Tekrar üzerine yazma girişimi reddedilmeli
    success_again, reason_again = interface.capture_takeoff_pose()
    assert success_again is False
    assert "zaten kilitlenmiş" in reason_again
    assert interface.get_takeoff_pose() == locked


# ====================================================================
# TEST GRUBU 2: İNİŞ DURUMU DEĞERLENDİRME GÜVENLİĞİ
# ====================================================================

def test_landed_status_returns_unknown_when_no_data(interface):
    """Kural 2a: ExtendedState hiç gelmediğinde durum UNKNOWN olmalıdır."""
    assert interface.get_landed_status() == LandedStatus.UNKNOWN
    assert interface.is_confirmed_landed() is False


def test_disarmed_alone_is_not_landed(interface):
    """Kural 2b: Aracın disarmed olması tek başına iniş teyidi SAYILAMAZ."""
    state_msg = State()
    state_msg.connected = True
    state_msg.armed = False  # Disarmed
    interface._state_cb(state_msg)

    assert interface.get_landed_status() == LandedStatus.UNKNOWN
    assert interface.is_confirmed_landed() is False


def test_landed_status_returns_unknown_when_stale(interface):
    """Kural 2c: ExtendedState verisi eskidiyse iniş durumu UNKNOWN'a düşmelidir."""
    ext_msg = ExtendedState()
    ext_msg.landed_state = ExtendedState.LANDED_STATE_ON_GROUND
    interface._ext_state_cb(ext_msg)

    # 2.0 saniye öncesine al (stale)
    interface._last_ext_state_time = time.time() - 2.0

    assert interface.get_landed_status(max_age_s=1.0) == LandedStatus.UNKNOWN
    assert interface.is_confirmed_landed(max_age_s=1.0) is False


def test_landed_status_strictly_confirms_on_ground_when_fresh(interface):
    """Kural 2d: Sadece güncel LANDED_STATE_ON_GROUND durumunda True döner."""
    ext_msg = ExtendedState()
    ext_msg.landed_state = ExtendedState.LANDED_STATE_ON_GROUND
    interface._ext_state_cb(ext_msg)

    assert interface.get_landed_status(max_age_s=1.0) == LandedStatus.ON_GROUND
    assert interface.is_confirmed_landed(max_age_s=1.0) is True

    ext_msg.landed_state = ExtendedState.LANDED_STATE_IN_AIR
    interface._ext_state_cb(ext_msg)
    assert interface.get_landed_status() == LandedStatus.IN_AIR
    assert interface.is_confirmed_landed() is False


# ====================================================================
# TEST GRUBU 3: HIZ VE TELEMETRİ GÜNCELLİĞİ (FRESHNESS)
# ====================================================================

def test_vertical_speed_returns_none_when_missing(interface):
    """Kural 3a: Hız verisi yokken 0.0 DEĞİL, None dönmelidir."""
    assert interface.get_current_vertical_speed() is None
    assert interface.get_current_horizontal_speed() is None


def test_vertical_speed_returns_none_when_stale(interface):
    """Kural 3b: Hız verisi eskidiyse 0.0 DEĞİL, None dönmelidir (araç durdu sanılmamalı)."""
    vel_msg = TwistStamped()
    vel_msg.twist.linear.z = -1.5
    interface._vel_cb(vel_msg)

    assert interface.get_current_vertical_speed(max_age_s=1.0) == -1.5

    # Zaman aşımı
    interface._last_vel_time = time.time() - 1.5
    assert interface.get_current_vertical_speed(max_age_s=1.0) is None
    assert interface.get_current_horizontal_speed(max_age_s=1.0) is None


def test_nan_inf_rejected_in_velocity_and_pose(interface):
    """Kural 3c: NaN veya Inf verileri geçerli hız veya konum sayılmaz, None döner."""
    vel_msg = TwistStamped()
    vel_msg.twist.linear.z = float('nan')
    interface._vel_cb(vel_msg)
    assert interface.get_current_vertical_speed() is None

    pose_msg = PoseStamped()
    pose_msg.pose.position.x = float('inf')
    pose_msg.pose.position.y = 0.0
    pose_msg.pose.position.z = 0.0
    interface._pose_cb(pose_msg)
    assert interface.get_current_xyz() is None
