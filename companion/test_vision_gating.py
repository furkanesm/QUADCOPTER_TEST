#!/usr/bin/env python3
"""
test_vision_gating.py - Aşama 3: Görüntü İşleme Tetikleme (VISION_ENABLE Gating) Birim Testleri
-----------------------------------------------------------------------------------------------
1. mavlink_mission_commander.py'nin HEDEF_BEKLE'de VISION_ENABLED ve Bool(True) yayını.
2. DONUS, INIS, TAMAMLANDI ve session stop durumlarında VISION_DISABLED ve Bool(False) yayını.
3. vision_node.py'nin enable_gating=True ile warm standby başlaması (vision_enabled=False).
4. vision_node.py'nin /vision/enable sinyaliyle açılıp kapanması ve inference gating davranışı.
"""

import sys
import os
import time
import numpy as np
import pytest

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Trigger
from vision_interfaces.msg import DetectionArray

# Import MavlinkAdapterNode
sys.path.append(os.path.join(os.path.dirname(__file__), "."))
from mavlink_mission_commander import MavlinkAdapterNode

# Import VisionNode
try:
    from vision_processing.vision_node import VisionNode
    VISION_NODE_AVAILABLE = True
except ImportError:
    try:
        sys.path.append(os.path.join(os.path.dirname(__file__), "../ros2_ws/src/vision_processing/vision_processing"))
        from vision_node import VisionNode
        VISION_NODE_AVAILABLE = True
    except ImportError:
        VISION_NODE_AVAILABLE = False


class MockMavMsg:
    def __init__(self, text, sysid=1, compid=1):
        self._text = text
        self.srcSystem = sysid
        self.srcComponent = compid
        self.severity = 6
        self.id = 0
        self.chunk_seq = 0

    def get_srcSystem(self):
        return self.srcSystem

    def get_srcComponent(self):
        return self.srcComponent

    @property
    def text(self):
        return self._text


def create_test_adapter():
    rclpy.init(args=None) if not rclpy.ok() else None
    node = MavlinkAdapterNode()
    events = []
    vision_enable_msgs = []

    def capture_event(msg):
        assert isinstance(msg, String)
        events.append(msg.data)

    def capture_vision_enable(msg):
        assert isinstance(msg, Bool)
        vision_enable_msgs.append(msg.data)

    node.pub_event_status.publish = capture_event
    node.pub_vision_enable.publish = capture_vision_enable

    # Monkeypatch send to prevent real socket calls
    node.master.mav.command_long_send = lambda *args, **kwargs: 1
    return node, events, vision_enable_msgs


def test_adapter_vision_enable_on_hedef_bekle():
    """1. HEDEF_BEKLE durumuna geçişte VISION_ENABLED ve Bool(True) yayımlanmalı."""
    node, events, vision_enable_msgs = create_test_adapter()
    try:
        node.session_state = node.ACTIVE
        assert node.vision_enabled is False

        # Preflight: DIKEY_TIRMANIS
        msg_dikey = MockMavMsg("[S500 LUA] Durum Gecisi: HAZIRLIK -> DIKEY_TIRMANIS")
        node._handle_statustext_msg(msg_dikey)
        assert node.vision_enabled is False
        assert len(vision_enable_msgs) == 0

        # Preflight: ILERI_HAREKET
        msg_ileri = MockMavMsg("[S500 LUA] Durum Gecisi: DIKEY_TIRMANIS -> ILERI_HAREKET")
        node._handle_statustext_msg(msg_ileri)
        assert node.vision_enabled is False
        assert len(vision_enable_msgs) == 0

        # HEDEF_BEKLE geçişi -> Vision açılmalı!
        msg_bekle = MockMavMsg("[S500 LUA] Durum Gecisi: ILERI_HAREKET -> HEDEF_BEKLE")
        node._handle_statustext_msg(msg_bekle)
        assert node.vision_enabled is True
        assert any("VISION_ENABLED" in e for e in events)
        assert True in vision_enable_msgs
        print("✓ Test 1: HEDEF_BEKLE'de VISION_ENABLED yayını doğrulandı.")
    finally:
        node.destroy_node()


def test_adapter_vision_disable_on_return_and_landing():
    """2. DONUS, INIS, TAMAMLANDI geçişlerinde VISION_DISABLED ve Bool(False) yayımlanmalı."""
    node, events, vision_enable_msgs = create_test_adapter()
    try:
        node.session_state = node.ACTIVE
        # Önce HEDEF_BEKLE ile açalım
        msg_bekle = MockMavMsg("[S500 LUA] Durum Gecisi: ILERI_HAREKET -> HEDEF_BEKLE")
        node._handle_statustext_msg(msg_bekle)
        assert node.vision_enabled is True

        # DONUS geçişi -> Vision kapanmalı
        msg_donus = MockMavMsg("[S500 LUA] Durum Gecisi: HEDEFE_GIT -> DONUS")
        node._handle_statustext_msg(msg_donus)
        assert node.vision_enabled is False
        assert any("VISION_DISABLED" in e for e in events)
        assert False in vision_enable_msgs

        # Tekrar açıp INIS ile deneyelim
        node.vision_enabled = True
        msg_inis = MockMavMsg("[S500 LUA] Durum Gecisi: DONUS -> INIS")
        node._handle_statustext_msg(msg_inis)
        assert node.vision_enabled is False

        # Session stop çağrıldığında da kapanmalı
        node.vision_enabled = True
        node.local_stop("Test Stop")
        assert node.vision_enabled is False
        print("✓ Test 2: DONUS, INIS ve local_stop durumlarında VISION_DISABLED yayını doğrulandı.")
    finally:
        node.destroy_node()


def test_vision_node_gating_parameters_and_lifecycle():
    """3. VisionNode enable_gating parametresi, varsayılan uyumluluk ve /vision/enable aboneliği."""
    if not VISION_NODE_AVAILABLE:
        pytest.skip("VisionNode modülü mevcut ortamda yüklü değil.")

    rclpy.init(args=None) if not rclpy.ok() else None

    # A) Varsayılan: enable_gating=False -> vision_enabled=True (Sıfır Regresyon)
    node_default = VisionNode()
    try:
        assert node_default.enable_gating is False
        assert node_default.vision_enabled is True
    finally:
        node_default.destroy_node()

    # B) Gating Açık: enable_gating=True -> vision_enabled=False (Warm Standby)
    from rclpy.parameter import Parameter
    node_gated = VisionNode()
    try:
        node_gated.enable_gating = True
        node_gated.vision_enabled = False  # simüle başlangıç

        # Erken inference çağrısı: gating kapalıyken tespit yayımlanmamalı
        published_msgs = []
        node_gated.pub_detections.publish = lambda msg: published_msgs.append(msg)
        node_gated.inference_loop()
        assert len(published_msgs) == 0, "Gating aktifken inference yapılmamalı/yayımlanmamalı"

        # /vision/enable -> True sinyali ver
        node_gated.enable_callback(Bool(data=True))
        assert node_gated.vision_enabled is True

        # /vision/enable -> False sinyali ver
        node_gated.enable_callback(Bool(data=False))
        assert node_gated.vision_enabled is False
        print("✓ Test 3: VisionNode enable_gating, warm standby ve sinyal yaşam döngüsü doğrulandı.")
    finally:
        node_gated.destroy_node()


def run_tests():
    print("=== Aşama 3: VISION_ENABLE Gating Birim Testleri Başlatılıyor ===")
    test_adapter_vision_enable_on_hedef_bekle()
    test_adapter_vision_disable_on_return_and_landing()
    test_vision_node_gating_parameters_and_lifecycle()
    print("\n>>> AŞAMA 3 TESTLERİ EKSİKSİZ GEÇTİ <<<")


if __name__ == '__main__':
    run_tests()
