import sys
import os
import time
import pytest
from unittest.mock import MagicMock, patch

sys.path.append(os.path.dirname(os.path.abspath(__file__)))

_mocked_keys = ['rclpy', 'rclpy.node', 'rclpy.qos', 'rclpy.time', 'std_msgs', 'std_msgs.msg', 'std_srvs', 'std_srvs.srv', 'vision_interfaces', 'vision_interfaces.msg']
_orig_modules = {k: sys.modules.get(k) for k in _mocked_keys}

sys.modules['rclpy'] = MagicMock()
sys.modules['rclpy.node'] = MagicMock()
sys.modules['rclpy.qos'] = MagicMock()
sys.modules['rclpy.time'] = MagicMock()
sys.modules['std_msgs'] = MagicMock()
sys.modules['std_msgs.msg'] = MagicMock()
class MockString:
    def __init__(self, data=""):
        self.data = data
sys.modules['std_msgs.msg'].String = MockString
sys.modules['std_srvs'] = MagicMock()
sys.modules['std_srvs.srv'] = MagicMock()
sys.modules['vision_interfaces'] = MagicMock()
sys.modules['vision_interfaces.msg'] = MagicMock()

class DummyNode:
    def __init__(self, name):
        self.name = name
        self._params = {}

    def declare_parameter(self, name, default):
        p = MagicMock()
        p.value = default
        self._params[name] = p
        return p

    def get_parameter(self, name):
        if name in self._params:
            return self._params[name]
        p = MagicMock()
        return p
    def get_logger(self):
        return MagicMock()
    def create_subscription(self, *args, **kwargs): return MagicMock()
    
    def create_publisher(self, *args, **kwargs): 
        pub = MagicMock()
        pub.publish_calls = [] # to track String instances
        def custom_publish(msg):
            pub.publish_calls.append(msg)
        pub.publish.side_effect = custom_publish
        return pub
        
    def create_service(self, *args, **kwargs): return MagicMock()
    def create_client(self, *args, **kwargs): return MagicMock()
    def create_timer(self, *args, **kwargs): return MagicMock()
    def get_clock(self): 
        mock_clock = MagicMock()
        mock_time = MagicMock()
        mock_time.nanoseconds = 0
        mock_clock.now.return_value = mock_time
        return mock_clock

sys.modules['rclpy.node'].Node = DummyNode

with patch('pymavlink.mavutil.mavlink_connection'):
    with patch('threading.Thread'):
        from mavlink_mission_commander import MavlinkAdapterNode

def create_mock_msg(node, p1, p2, p4, p6, p5=0xAB, cmd=31010, tgt_sys=1, tgt_comp=191, src_sys=1, src_comp=1, param7=1.0):
    msg = MagicMock()
    msg.get_type.return_value = 'COMMAND_LONG'
    msg.target_system = tgt_sys
    msg.target_component = tgt_comp
    msg.get_srcSystem.return_value = src_sys
    msg.get_srcComponent.return_value = src_comp
    msg.command = cmd
    msg.param1 = float(p1)
    msg.param2 = float(p2)
    msg.param4 = float(p4)
    msg.param5 = float(p5)
    msg.param6 = float(p6)
    msg.param7 = float(param7) 
    return msg

def inject_rx(node, msg):
    def mock_recv(*args, **kwargs):
        node._running = False
        return msg
    node._running = True
    node.master.recv_match.side_effect = mock_recv
    node.mavlink_rx_thread()
    node.process_rx_queue()

@pytest.fixture(scope="module", autouse=True)
def cleanup_sys_modules():
    yield
    for k, v in _orig_modules.items():
        if v is not None:
            sys.modules[k] = v
        elif k in sys.modules:
            del sys.modules[k]

@pytest.fixture
def manual_time():
    current_time = [0.0]
    def mock_mono():
        return current_time[0]
    def advance(seconds):
        current_time[0] += seconds
        
    with patch('time.monotonic', side_effect=mock_mono):
        yield advance

@pytest.fixture
def node():
    with patch('pymavlink.mavutil.mavlink_connection'):
        with patch('threading.Thread'):
            n = MavlinkAdapterNode()
            n.master = MagicMock()
            n.master.mav.command_long_send = MagicMock()
            return n

def test_start_session_seq_allocation(node, manual_time):
    # SESSION_START SEQ=1, first normal event SEQ=2
    node.srv_start_cb(MagicMock(), MagicMock())
    assert node.session_state == 1
    assert node.handshake_pending['seq'] == 1
    
    ack = create_mock_msg(node, p1=0xAC, p2=0, p4=1, p6=node.session_id)
    inject_rx(node, ack)
    assert node.session_state == 2
    
    msg_time = MagicMock()
    node.queue_command(1, 10.0, 10.0, 0.9, msg_time)
    assert 2 in node.pending_commands
    
def test_rx_validation_wrong_params(node, manual_time):
    node.srv_start_cb(MagicMock(), MagicMock())
    
    # Wrong Target System
    inject_rx(node, create_mock_msg(node, p1=0xAC, p2=0, p4=1, p6=node.session_id, tgt_sys=99))
    assert node.session_state == 1
    
    # Wrong Target Component
    inject_rx(node, create_mock_msg(node, p1=0xAC, p2=0, p4=1, p6=node.session_id, tgt_comp=99))
    assert node.session_state == 1
    
    # Wrong Source System
    inject_rx(node, create_mock_msg(node, p1=0xAC, p2=0, p4=1, p6=node.session_id, src_sys=99))
    assert node.session_state == 1
    
    # Wrong Command ID
    inject_rx(node, create_mock_msg(node, p1=0xAC, p2=0, p4=1, p6=node.session_id, cmd=99999))
    assert node.session_state == 1
    
    # Wrong Version (param7 != 1)
    inject_rx(node, create_mock_msg(node, p1=0xAC, p2=0, p4=1, p6=node.session_id, param7=2.0))
    assert node.session_state == 1
    
    # Wrong SEQ explicitly
    inject_rx(node, create_mock_msg(node, p1=0xAC, p2=0, p4=999, p6=node.session_id))
    assert node.session_state == 1
    
    # Wrong SESSION_ID explicitly
    inject_rx(node, create_mock_msg(node, p1=0xAC, p2=0, p4=1, p6=999999))
    assert node.session_state == 1

    # Valid payload works
    inject_rx(node, create_mock_msg(node, p1=0xAC, p2=0, p4=1, p6=node.session_id))
    assert node.session_state == 2

def test_start_session_rejected(node, manual_time):
    node.srv_start_cb(MagicMock(), MagicMock())
    assert node.session_state == 1
    
    ack = create_mock_msg(node, p1=0xAC, p2=1, p4=1, p6=node.session_id)
    inject_rx(node, ack)
    assert node.session_state == 0 # REJECTED keeps session closed (IDLE)

def test_double_start_service(node, manual_time):
    # STARTING
    node.srv_start_cb(MagicMock(), MagicMock())
    seq1 = node.handshake_pending['seq']
    sess_id1 = node.session_id
    
    node.srv_start_cb(MagicMock(), MagicMock())
    assert node.handshake_pending['seq'] == seq1
    assert node.session_id == sess_id1
    
    # ACTIVE
    inject_rx(node, create_mock_msg(node, p1=0xAC, p2=0, p4=seq1, p6=sess_id1))
    assert node.session_state == 2
    
    node.srv_start_cb(MagicMock(), MagicMock())
    assert node.session_state == 2
    assert node.handshake_pending is None # Queue untouched

def test_retry_tx_failed(node, manual_time):
    node.srv_start_cb(MagicMock(), MagicMock())
    
    node.master.mav.command_long_send.side_effect = Exception("Offline")
    
    for _ in range(6):
        manual_time(1.5)
        node.retry_loop()
    
    assert node.session_state == 0
    assert node.handshake_pending is None
    
    # Check published message
    published_msgs = [m.data for m in node.pub_event_status.publish_calls]
    assert "HANDSHAKE_TX_FAILED:1" in published_msgs

def test_retry_timeout(node, manual_time):
    node.srv_start_cb(MagicMock(), MagicMock())
    
    node.master.mav.command_long_send.side_effect = None
    node.master.mav.command_long_send.reset_mock()
    
    # Advance time sequentially to trigger sends
    for i in range(5):
        manual_time(1.5) # advance enough to trigger next tx
        node.retry_loop()
        
    assert node.master.mav.command_long_send.call_count == 5
    
    # Validate payload for all sends is identical
    first_call_args = node.master.mav.command_long_send.call_args_list[0][0]
    for call in node.master.mav.command_long_send.call_args_list:
        assert call[0] == first_call_args
        # Target sys=1, comp=1, cmd=31010, seq=1, MSG=0xAC=172, session=node.session_id
        assert call[0][8] == 172.0 
        assert call[0][7] == 1.0 
        assert call[0][9] == float(node.session_id)
        
    # Waiting after last send without ACK causes timeout
    manual_time(1.5)
    node.retry_loop()
    
    assert node.session_state == 0
    published_msgs = [m.data for m in node.pub_event_status.publish_calls]
    assert "HANDSHAKE_TIMEOUT:1" in published_msgs
    
def test_session_cleanup_on_stop(node, manual_time):
    node.srv_start_cb(MagicMock(), MagicMock())
    inject_rx(node, create_mock_msg(node, p1=0xAC, p2=0, p4=1, p6=node.session_id))
    
    node.queue_command(1, 10.0, 10.0, 0.9, MagicMock())
    assert 1 in node.last_sent_types
    
    node.local_stop("TEST")
    
    assert len(node.last_sent_types) == 0
    assert len(node.pending_commands) == 0
    assert node.session_state == 0
    
    # Start -> Accept -> Send target again. Should not be suppressed.
    node.srv_start_cb(MagicMock(), MagicMock())
    inject_rx(node, create_mock_msg(node, p1=0xAC, p2=0, p4=node.handshake_pending['seq'], p6=node.session_id))
    
    # This event is conceptually the same item. Spam filter uses distance and type.
    node.queue_command(1, 10.0, 10.0, 0.9, MagicMock())
    # Should be successfully queued because last_sent_types was cleared!
    assert len(node.pending_commands) == 1

def test_late_ack_ignored(node, manual_time):
    node.srv_start_cb(MagicMock(), MagicMock())
    sess_id = node.session_id
    seq = node.handshake_pending['seq']
    
    # Wait for timeout
    for _ in range(6):
        manual_time(1.5)
        node.retry_loop()
        
    assert node.session_state == 0
    
    # Late ACK arrives
    inject_rx(node, create_mock_msg(node, p1=0xAC, p2=0, p4=seq, p6=sess_id))
    assert node.session_state == 0 # Still IDLE

def test_seq_limit_rollover(node, manual_time):
    node.srv_start_cb(MagicMock(), MagicMock())
    inject_rx(node, create_mock_msg(node, p1=0xAC, p2=0, p4=1, p6=node.session_id))
    
    node.seq_counter = 16777215
    node.queue_command(1, 10.0, 10.0, 0.9, MagicMock())
    
    assert node.session_state == 0
    node.srv_start_cb(MagicMock(), MagicMock())
    assert node.handshake_pending['seq'] == 1 

def test_idle_starting_restricts_normal_sends(node, manual_time):
    node.master.mav.command_long_send.reset_mock()
    
    # Try sending events/liveliness in IDLE
    node.queue_command(1, 10.0, 10.0, 0.9, MagicMock())
    node.liveliness_loop()
    assert node.master.mav.command_long_send.call_count == 0
    
    # Try sending in STARTING
    node.srv_start_cb(MagicMock(), MagicMock())
    node.queue_command(1, 10.0, 10.0, 0.9, MagicMock())
    node.liveliness_loop()
    # Still 0. Only handshake is sent through retry_loop, not here.
    assert node.master.mav.command_long_send.call_count == 0

