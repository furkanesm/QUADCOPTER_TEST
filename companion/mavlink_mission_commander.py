#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
import time
import random
import threading
import math
import queue
from pymavlink import mavutil
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Trigger
from std_msgs.msg import String
from mavros_msgs.srv import WaypointPush, WaypointPull
from mavros_msgs.msg import Waypoint
import yaml

from vision_interfaces.msg import DetectionArray
import sys
import os

# Add local path for class_mapping resolution explicitly if running standalone
sys.path.append(os.path.dirname(os.path.abspath(__file__)))
from class_mapping import ClassMapper


class RobustStatustextAssembler:
    """
    ArduPilot MAVLink STATUSTEXT parçalarını kaynak yalıtımlı, zaman aşımlı,
    bellek sınırlı ve tamlık (0..final_seq) garantili olarak birleştiren yardımcı sınıf.
    """
    def __init__(self, timeout_s: float = 2.0, max_pending: int = 50):
        self.timeout_s = timeout_s
        self.max_pending = max_pending
        self.pending = {}

    def _purge_stale(self, now: float):
        """Zaman aşımına uğramış parçalı mesajları bellekten temizler."""
        stale_keys = [k for k, v in self.pending.items() if (now - v['last_seen']) > self.timeout_s]
        for k in stale_keys:
            del self.pending[k]
        if len(self.pending) > self.max_pending:
            sorted_keys = sorted(self.pending.keys(), key=lambda k: self.pending[k]['first_seen'])
            for k in sorted_keys[: len(self.pending) - self.max_pending]:
                del self.pending[k]

    def feed(self, msg, now=None):
        if now is None:
            now = time.monotonic()
        self._purge_stale(now)

        src_sys = msg.get_srcSystem() if hasattr(msg, 'get_srcSystem') else getattr(msg, 'srcSystem', 1)
        src_comp = msg.get_srcComponent() if hasattr(msg, 'get_srcComponent') else getattr(msg, 'srcComponent', 1)
        chunk_id = getattr(msg, 'id', 0)
        chunk_seq = getattr(msg, 'chunk_seq', 0)

        # Ham bayt yükünü güvenli çıkar
        if hasattr(msg, '_text_raw') and msg._text_raw is not None:
            raw_payload = bytes(msg._text_raw)
        elif isinstance(getattr(msg, 'text', None), bytes):
            raw_payload = msg.text
        elif isinstance(getattr(msg, 'text', None), str):
            raw_payload = msg.text.encode('utf-8', errors='ignore')
        else:
            raw_payload = b""

        # Null-terminator ve son parça tespiti
        if b'\x00' in raw_payload:
            chunk_bytes = raw_payload.split(b'\x00', 1)[0]
            is_last = True
        else:
            chunk_bytes = raw_payload
            is_last = (len(raw_payload) < 50)

        # 1. Tekil (parçasız) standart STATUSTEXT (id == 0)
        if chunk_id == 0:
            try:
                text = chunk_bytes.decode('utf-8', errors='replace').strip()
            except Exception:
                text = chunk_bytes.decode('latin-1', errors='ignore').strip()
            return text if text else None

        # 2. Çok parçalı (chunked) STATUSTEXT (id != 0)
        key = (src_sys, src_comp, chunk_id)

        # ID reuse koruması: Aynı anahtar için yeni bir 0. parça geldiğinde eski yarım kayıt silinir
        if key in self.pending and chunk_seq == 0:
            del self.pending[key]

        if key not in self.pending:
            self.pending[key] = {
                'chunks': {},
                'final_seq': None,
                'first_seen': now,
                'last_seen': now
            }

        entry = self.pending[key]
        entry['chunks'][chunk_seq] = chunk_bytes
        entry['last_seen'] = now

        if is_last:
            entry['final_seq'] = chunk_seq

        # Tamlık kontrolü: 0..final_seq arasındaki bütün parçalar bulunmadan çıktı üretilmez
        if entry['final_seq'] is not None:
            final_seq = entry['final_seq']
            if all(i in entry['chunks'] for i in range(final_seq + 1)):
                full_bytes = b"".join(entry['chunks'][i] for i in range(final_seq + 1))
                del self.pending[key]
                try:
                    text = full_bytes.decode('utf-8', errors='replace').strip()
                except Exception:
                    text = full_bytes.decode('latin-1', errors='ignore').strip()
                return text if text else None

        return None


class MavlinkAdapterNode(Node):
    ALLOWED_FRAMES_NED = {"ekf_origin_ned", "ned"}
    ALLOWED_STATES_GOTO_OBS = {"HEDEF_BEKLE", "HEDEF_KONUMUNDA_BEKLE"}
    ALLOWED_STATES_ROUTE_READY = {"HEDEF_BEKLE", "HEDEF_KONUMUNDA_BEKLE"}

    def __init__(self):
        super().__init__('mavlink_mission_commander')
        
        # Connections and Routing
        self.declare_parameter('mavlink_connection', 'udp:127.0.0.1:14551')
        self.declare_parameter('liveliness_hz', 2.0)
        self.declare_parameter('retry_hz', 1.0)
        
        # Identity
        self.declare_parameter('target_system', 1)
        self.declare_parameter('target_component', 1)
        self.declare_parameter('my_system_id', 1)
        self.declare_parameter('my_component_id', 191) 
        
        # Classes 
        self.declare_parameter('start_classes', ['start', 's_class', 'kb_b'])
        self.declare_parameter('goal_classes', ['hedef', 'h_class'])
        
        # Timeouts and Thresholds
        self.declare_parameter('data_stale_timeout_s', 2.0)
        self.declare_parameter('event_expiry_s', 15.0)
        self.declare_parameter('max_queue_size', 20)
        self.declare_parameter('spatial_spam_dist_m', 2.0)
        self.declare_parameter('spam_timeout_s', 10.0)
        self.declare_parameter('max_obs_dist_m', 50.0)
        self.declare_parameter('cmd_stale_timeout_s', 2.0)
        self.declare_parameter('transition_timeout_s', 5.0)

        self.conn_str = self.get_parameter('mavlink_connection').value
        
        self.live_hz = self.get_parameter('liveliness_hz').value
        self.retry_hz = self.get_parameter('retry_hz').value
        if not (math.isfinite(self.live_hz) and self.live_hz > 0 and math.isfinite(self.retry_hz) and self.retry_hz > 0):
            self.live_hz = 2.0
            self.retry_hz = 1.0
            
        self.sys_id = self.get_parameter('target_system').value
        self.comp_id = self.get_parameter('target_component').value
        self.my_sys = self.get_parameter('my_system_id').value
        self.my_comp = self.get_parameter('my_component_id').value
        self.start_classes = self.get_parameter('start_classes').value
        self.goal_classes = self.get_parameter('goal_classes').value
        
        self.stale_timeout = self.get_parameter('data_stale_timeout_s').value
        self.expiry_s = self.get_parameter('event_expiry_s').value
        self.max_queue = self.get_parameter('max_queue_size').value
        self.spam_dist = self.get_parameter('spatial_spam_dist_m').value
        self.spam_timeout = self.get_parameter('spam_timeout_s').value
        self.max_obs_dist = self.get_parameter('max_obs_dist_m').value
        self.cmd_stale_timeout = self.get_parameter('cmd_stale_timeout_s').value
        self.transition_timeout = self.get_parameter('transition_timeout_s').value
        
        self._running = True
        self.session_id = random.randint(1, 16000000)
        
        config_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'camera_config.yaml')
        try:
            with open(config_path, 'r') as f:
                self.cam_config = yaml.safe_load(f)
        except Exception as e:
            self.get_logger().error(f"[ADAPTER] Failed to load camera_config.yaml: {e}")
            self.cam_config = None
            
        self.current_alt = None
        
        # Session states
        self.IDLE = 0
        self.STARTING = 1
        self.ACTIVE = 2
        self.session_state = self.IDLE
        self.session_epoch = 0
        self.lua_fsm_state = "UNKNOWN"
        self.handshake_pending = None
        
        self.statustext_assembler = RobustStatustextAssembler(timeout_s=2.0)
        
        self.master = mavutil.mavlink_connection(
            self.conn_str, 
            source_system=self.my_sys, 
            source_component=self.my_comp
        )
        self.get_logger().info(f"[ADAPTER] Seeking telemetry from {self.conn_str}...")
        
        self.rx_queue = queue.Queue(maxsize=100)
        
        # Status Trackers
        self.last_vision_source_time = None
        self.current_liveliness_state = 0
        
        # Rate-limited Loggers (Monotonic tracking)
        self.last_tx_err_log_time = 0.0
        self.last_rx_err_log_time = 0.0
        
        self.mapper = ClassMapper(self.start_classes, self.goal_classes)
        
        self.pending_commands = {}
        self.acked_commands = {}
        self.seq_counter = 0
        self.last_sent_types = {1: None, 2: None, 3: None, 4: None}
        self.sub_detections = self.create_subscription(
            DetectionArray, '/vision/detections', self.det_callback, qos_profile_sensor_data
        )
        self.sub_goto_obs = self.create_subscription(
            PoseStamped, '/mission/goto_observation', self.goto_observation_callback, 10
        )
        self.sub_local_pos = self.create_subscription(
            PoseStamped, '/mavros/local_position/pose', self.local_pos_cb, qos_profile_sensor_data
        )
        self.pub_event_status = self.create_publisher(String, '/adapter/event_status', 10)
        
        self.srv_start = self.create_service(Trigger, '/adapter/start_session', self.srv_start_cb)
        self.srv_stop = self.create_service(Trigger, '/adapter/stop_session', self.srv_stop_cb)
        self.srv_route_ready = self.create_service(Trigger, '/mission/route_ready', self.route_ready_srv_cb)
        
        # Test Mission Upload
        self.cli_wp_push = self.create_client(WaypointPush, '/mavros/mission/push')
        self.cli_wp_pull = self.create_client(WaypointPull, '/mavros/mission/pull')
        self.srv_test_route = self.create_service(Trigger, '/adapter/test_route_upload', self.test_route_upload_srv_cb)
        
        self.retry_timer = self.create_timer(1.0 / self.retry_hz, self.retry_loop)
        self.live_timer = self.create_timer(1.0 / self.live_hz, self.liveliness_loop)
        
        self.rx_thread = threading.Thread(target=self.mavlink_rx_thread, daemon=True)
        self.rx_thread.start()

    def local_pos_cb(self, msg: PoseStamped):
        self.current_alt = msg.pose.position.z

    def get_dynamic_gsd(self):
        if not self.cam_config:
            return 0.05
        sensor_w = self.cam_config['camera']['sensor']['width_mm']
        focal_l = self.cam_config['camera']['lens']['focal_length_mm']
        img_w = self.cam_config['camera']['image']['width_px']
        
        alt = self.current_alt
        if alt is None or alt <= 0.5:
            alt = self.cam_config['camera']['assumptions']['default_altitude_m']
            self.get_logger().debug(f"[ADAPTER] Real-time altitude unavailable or invalid ({self.current_alt}). Sticking to config default: {alt}m")
            
        gsd = (alt * sensor_w) / (focal_l * img_w)
        return gsd

    def test_route_upload_srv_cb(self, request, response):
        if self.session_state != self.ACTIVE:
            response.success = False
            response.message = "Session not ACTIVE."
            return response
            
        threading.Thread(target=self._test_route_upload_thread_func, daemon=True).start()
        response.success = True
        response.message = "Initiating DISARMED test mission upload in background..."
        return response
        
    def _test_route_upload_thread_func(self):
        import rclpy
        self.get_logger().info("[ADAPTER TEST] Starting mock mission upload...")
        
        # Create a helper node for synchronous wait
        helper = Node("_mock_upload_helper")
        try:
            cli_push = helper.create_client(WaypointPush, "/mavros/mission/push")
            cli_pull = helper.create_client(WaypointPull, "/mavros/mission/pull")
            
            if not cli_push.wait_for_service(timeout_sec=5.0) or not cli_pull.wait_for_service(timeout_sec=5.0):
                self.get_logger().error("[ADAPTER TEST] MAVROS mission services not available!")
                return
                
            wp1 = Waypoint()
            wp1.frame = 6 # GLOBAL_RELATIVE_ALT
            wp1.command = 16 # MAV_CMD_NAV_WAYPOINT
            wp1.is_current = True
            wp1.autocontinue = True
            wp1.param1 = 0.0
            wp1.param2 = 1.0 # 1m accept radius
            wp1.param3 = 0.0
            wp1.param4 = 0.0
            wp1.x_lat = 41.0
            wp1.y_long = 29.0
            wp1.z_alt = 33.0
            
            wp2 = Waypoint()
            wp2.frame = 6
            wp2.command = 16
            wp2.is_current = False
            wp2.autocontinue = True
            wp2.param1 = 0.0
            wp2.param2 = 1.0
            wp2.param3 = 0.0
            wp2.param4 = 0.0
            wp2.x_lat = 41.0001
            wp2.y_long = 29.0001
            wp2.z_alt = 33.0
            
            req_push = WaypointPush.Request()
            req_push.start_index = 0
            req_push.waypoints = [wp1, wp2]
            
            self.get_logger().info("[ADAPTER TEST] Pushing 2 waypoints via MAVROS...")
            f_push = cli_push.call_async(req_push)
            rclpy.spin_until_future_complete(helper, f_push, timeout_sec=5.0)
            res_push = f_push.result()
            
            if not res_push or not res_push.success:
                self.get_logger().error("[ADAPTER TEST] WaypointPush failed!")
                return
                
            self.get_logger().info("[ADAPTER TEST] Push successful. Pulling mission back...")
            
            req_pull = WaypointPull.Request()
            f_pull = cli_pull.call_async(req_pull)
            rclpy.spin_until_future_complete(helper, f_pull, timeout_sec=5.0)
            res_pull = f_pull.result()
            
            if not res_pull or not res_pull.success:
                self.get_logger().error("[ADAPTER TEST] WaypointPull failed!")
                return
                
            self.get_logger().info(f"[ADAPTER TEST] Pull successful! Recovered {res_pull.wp_received} waypoints.")
            
            # Send ROUTE_READY
            self.queue_command(4, 0.0, 0.0, 1.0, self.get_clock().now(), allowed_states=self.ALLOWED_STATES_ROUTE_READY)
            self.get_logger().info("[ADAPTER TEST] Sent ROUTE_READY (Status=4).")
            
        finally:
            helper.destroy_node()

    def get_next_seq(self):
        if self.seq_counter >= 16777215:
            self.get_logger().error("[ADAPTER FATAL] SEQ reached Float32 capacity (16777215). Controlled new session needed!")
            self.local_stop("SEQ_LIMIT")
            self.pub_event_status.publish(String(data="FATAL_SESSION_LIMIT"))
            return None
        self.seq_counter += 1
        return self.seq_counter

    def srv_start_cb(self, request, response):
        if self.session_state != self.IDLE:
            response.success = True
            response.message = f"Currently in state {'ACTIVE' if self.session_state == self.ACTIVE else 'STARTING'}, request ignored."
            return response
            
        self.session_state = self.STARTING
        self.session_epoch += 1
        self.session_id = random.randint(1, 16000000)
        self.seq_counter = 0
        self.lua_fsm_state = "UNKNOWN"
        self.pending_commands.clear()
        self.acked_commands.clear()
        self.last_sent_types = {1: None, 2: None, 3: None, 4: None}
        seq = self.get_next_seq()
        if seq is None:
            response.success = False
            response.message = "Could not initialize SEQ."
            return response
        
        self.handshake_pending = {
            'seq': seq,
            'retries': 0,
            'successful_txs': 0,
            'last_tx_mono': 0.0,
            'queue_ts_mono': time.monotonic()
        }
        self.get_logger().info(f"[ADAPTER] Starting session handshake, SEQ={seq}, ID={self.session_id}, Epoch={self.session_epoch}")
        response.success = True
        response.message = "Session start handshake initiated."
        return response

    def local_stop(self, reason):
        self.session_state = self.IDLE
        self.lua_fsm_state = "UNKNOWN"
        self.handshake_pending = None
        self.last_sent_types.clear()
        for seq, cmd in self.pending_commands.items():
            self.pub_event_status.publish(String(data=f"CANCELLED:{seq}:{cmd['msg_type']}"))
        self.pending_commands.clear()
        self.acked_commands.clear()
        self.get_logger().warn(f"[ADAPTER] Local session stopped: {reason}")
        
    def srv_stop_cb(self, request, response):
        self.local_stop("Stop service called")
        response.success = True
        response.message = "Local session stopped and queues cleared."
        return response

    def goto_observation_callback(self, msg: PoseStamped):
        now_ros = self.get_clock().now()

        if self.session_state != self.ACTIVE:
            return

        # 1. Erken gönderim engeli: UNKNOWN veya izin verilmeyen state
        if self.lua_fsm_state not in self.ALLOWED_STATES_GOTO_OBS:
            err = f"REJECTED_FSM_STATE_NOT_ALLOWED:3:state_is_{self.lua_fsm_state}"
            self.get_logger().warn(f"[ADAPTER] {err}")
            self.pub_event_status.publish(String(data=err))
            return

        # 2. Ortak Origin / Frame doğrulaması (yalnızca ekf_origin_ned ve ned)
        frame_id = (msg.header.frame_id or "").strip().lower()
        if not frame_id:
            err = "REJECTED_EMPTY_FRAME_ID:3"
            self.get_logger().warn(f"[ADAPTER] {err}")
            self.pub_event_status.publish(String(data=err))
            return

        if frame_id not in self.ALLOWED_FRAMES_NED:
            err = f"REJECTED_UNVERIFIED_FRAME_ID:3:{frame_id}_requires_ekf_origin_ned"
            self.get_logger().warn(f"[ADAPTER] {err}")
            self.pub_event_status.publish(String(data=err))
            return

        x_ned = float(msg.pose.position.x)
        y_ned = float(msg.pose.position.y)
        if not (math.isfinite(x_ned) and math.isfinite(y_ned)):
            self.pub_event_status.publish(String(data="REJECTED_INVALID_COORDINATES:3"))
            return

        # 3. Mesafe Sınırı: EKF Origin (0,0) referansına göre
        dist_from_origin = math.hypot(x_ned, y_ned)
        if dist_from_origin > self.max_obs_dist:
            err = f"REJECTED_SAFETY_BOUNDS_EXCEEDED:3:{dist_from_origin:.1f}m_gt_{self.max_obs_dist}m"
            self.get_logger().warn(f"[ADAPTER] {err}")
            self.pub_event_status.publish(String(data=err))
            return

        # 4. Kaynak zaman damgası kontrolü
        if msg.header.stamp.sec == 0 and msg.header.stamp.nanosec == 0:
            err = "REJECTED_EMPTY_SOURCE_TIMESTAMP:3"
            self.get_logger().warn(f"[ADAPTER] {err}")
            self.pub_event_status.publish(String(data=err))
            return

        msg_time = Time.from_msg(msg.header.stamp, clock_type=now_ros.clock_type)
        dt_source = (now_ros - msg_time).nanoseconds / 1e9
        if dt_source < 0.0 or dt_source > self.cmd_stale_timeout:
            err = f"REJECTED_STALE_SOURCE_TIMESTAMP:3:age_{dt_source:.2f}s"
            self.get_logger().warn(f"[ADAPTER] {err}")
            self.pub_event_status.publish(String(data=err))
            return

        # 5. Mükerrer İstek Engeli (ACK almış ve geçiş bekleyen kayıtlar kontrol edilir)
        for seq, cmd in self.acked_commands.items():
            if cmd['msg_type'] == 3 and cmd['transition_state'] == 'ACKED_TRANSITION_PENDING':
                if math.hypot(cmd['x'] - x_ned, cmd['y'] - y_ned) < self.spam_dist:
                    err = f"REJECTED_ALREADY_ACKED_AWAITING_TRANSITION:3:seq_{seq}"
                    self.get_logger().warn(f"[ADAPTER] {err}")
                    self.pub_event_status.publish(String(data=err))
                    return

        self.queue_command(3, x_ned, y_ned, 1.0, msg_time, allowed_states=self.ALLOWED_STATES_GOTO_OBS)

    def route_ready_srv_cb(self, request, response):
        now_ros = self.get_clock().now()

        if self.session_state != self.ACTIVE:
            response.success = False
            response.message = "REJECTED_SESSION_NOT_ACTIVE:4"
            return response

        # Erken gönderim engeli: UNKNOWN veya izin verilmeyen state
        if self.lua_fsm_state not in self.ALLOWED_STATES_ROUTE_READY:
            err = f"REJECTED_FSM_STATE_NOT_ALLOWED:4:state_is_{self.lua_fsm_state}"
            self.get_logger().warn(f"[ADAPTER] {err}")
            self.pub_event_status.publish(String(data=err))
            response.success = False
            response.message = err
            return response

        # Zaten geçiş bekleyen veya kuyrukta olan ROUTE_READY kontrolü
        for seq, cmd in self.acked_commands.items():
            if cmd['msg_type'] == 4 and cmd['transition_state'] == 'ACKED_TRANSITION_PENDING':
                err = f"REJECTED_ALREADY_ACKED_AWAITING_TRANSITION:4:seq_{seq}"
                self.get_logger().warn(f"[ADAPTER] {err}")
                self.pub_event_status.publish(String(data=err))
                response.success = False
                response.message = err
                return response

        for seq, cmd in self.pending_commands.items():
            if cmd['msg_type'] == 4:
                err = f"REJECTED_ALREADY_PENDING:4:seq_{seq}"
                self.get_logger().warn(f"[ADAPTER] {err}")
                self.pub_event_status.publish(String(data=err))
                response.success = False
                response.message = err
                return response

        seq = self.queue_command(4, 0.0, 0.0, 1.0, now_ros, allowed_states=self.ALLOWED_STATES_ROUTE_READY)
        if seq is not None:
            response.success = True
            response.message = f"QUEUED:{seq}:4"
        else:
            response.success = False
            response.message = "REJECTED_QUEUE_FAILED:4"
        return response

    def _is_valid_target_payload(self, d):
        if not d.position_valid:
            return False, 0
        
        x_ned = float(d.local_y)
        y_ned = float(d.local_x)
        conf = float(d.confidence)
        
        if not (math.isfinite(x_ned) and math.isfinite(y_ned)):
            return False, 0
            
        if not math.isfinite(conf) or conf <= 0.0 or conf > 1.0:
            return False, 0
            
        msg_type = self.mapper.get_msg_type(d.class_name, self.get_logger().warn)
        if msg_type > 0:
            return True, msg_type
            
        return False, 0

    def det_callback(self, msg: DetectionArray):
        ros_now = self.get_clock().now()
        
        # Validate Source Timestamp
        if msg.header.stamp.sec == 0 and msg.header.stamp.nanosec == 0:
            is_stale = True
            msg_time = None
        else:
            msg_time = Time.from_msg(msg.header.stamp, clock_type=ros_now.clock_type)
            age_s = (ros_now - msg_time).nanoseconds / 1e9
            
            if age_s < 0.0:
                is_stale = True # Future time / backward jump is erroneous
            elif age_s > self.stale_timeout:
                is_stale = True
            else:
                is_stale = False
                
        if not is_stale and msg_time is not None:
            self.last_vision_source_time = msg_time

        valid_targets = []
        if not is_stale and msg.detections:
            for d in msg.detections:
                ok, ctype = self._is_valid_target_payload(d)
                if ok:
                    valid_targets.append((d, ctype))
                    
        if is_stale:
            self.current_liveliness_state = 0
        elif not msg.detections:
            self.current_liveliness_state = 1
        elif not valid_targets:
            self.current_liveliness_state = 2
        else:
            self.current_liveliness_state = 3

        if self.session_state != self.ACTIVE or is_stale:
            return
            
        for d, msg_type in valid_targets:
            x_ned = d.local_y
            y_ned = d.local_x
            conf = d.confidence
            self.queue_command(msg_type, x_ned, y_ned, conf, msg_time)

    def queue_command(self, mtype, x, y, conf, msg_time_ros, allowed_states=None):
        if self.session_state != self.ACTIVE:
            return None
            
        if len(self.pending_commands) >= self.max_queue:
            return None
            
        now_mono = time.monotonic()
        
        prev = self.last_sent_types.get(mtype)
        if prev is not None:
            time_diff = now_mono - prev['ts_mono']
            dist = math.hypot(prev['x'] - x, prev['y'] - y)
            if dist < self.spam_dist and time_diff < self.spam_timeout:
                return None

        for k, v in self.pending_commands.items():
            if v['msg_type'] == mtype and math.hypot(v['x'] - x, v['y'] - y) < self.spam_dist:
                return None
                
        seq = self.get_next_seq()
        if seq is None:
            return None
        
        self.pending_commands[seq] = {
            'msg_type': mtype,
            'x': float(x),
            'y': float(y),
            'conf': float(conf),
            'session_id': self.session_id,
            'source_time': msg_time_ros,
            'queue_ts_mono': now_mono,
            'first_tx_mono': 0.0,
            'last_tx_mono': 0.0,
            'retries': 0,
            'successful_txs': 0,
            'allowed_states': allowed_states,
            'early_transition_seen': False,
            'early_transition_state': None,
            'early_transition_time': None
        }
        
        self.last_sent_types[mtype] = {'x': x, 'y': y, 'ts_mono': now_mono}
        self.pub_event_status.publish(String(data=f"QUEUED:{seq}:{mtype}"))
        return seq

    def process_rx_queue(self):
        while not self.rx_queue.empty():
            try:
                msg = self.rx_queue.get_nowait()
                mtype = msg.get_type() if hasattr(msg, 'get_type') and callable(msg.get_type) else getattr(msg, '_type', '')
                if mtype == 'STATUSTEXT':
                    self._handle_statustext_msg(msg)
                elif mtype == 'COMMAND_LONG':
                    self._handle_ack_msg(msg)
            except queue.Empty:
                break

    def _handle_ack_msg(self, msg):
        acked_type = int(msg.param1)
        result = int(msg.param2)
        acked_seq = int(msg.param4)
        acked_sess = int(msg.param6)
        
        if result not in [0, 1]:
            return
            
        if acked_type == 0xAC:
            if self.session_state != self.STARTING:
                return
            if self.handshake_pending is None or acked_seq != self.handshake_pending['seq'] or acked_sess != self.session_id:
                return
            if result == 0:
                self.session_state = self.ACTIVE
                self.get_logger().info(f"[ADAPTER] SESSION_START ACCEPTED: SEQ={acked_seq}")
                self.pub_event_status.publish(String(data=f"HANDSHAKE_ACCEPTED:{acked_seq}"))
            else:
                self.session_state = self.IDLE
                self.get_logger().warn(f"[ADAPTER] SESSION_START REJECTED: SEQ={acked_seq}")
                self.pub_event_status.publish(String(data=f"HANDSHAKE_REJECTED:{acked_seq}"))
            self.handshake_pending = None
            return

        if self.session_state != self.ACTIVE:
            return
            
        if acked_sess != self.session_id:
            return
            
        if acked_seq in self.pending_commands:
            cmd = self.pending_commands[acked_seq]
            
            if cmd['msg_type'] != acked_type:
                self.get_logger().error(f"[PROTOCOL] Lua ACKed SEQ={acked_seq} with wrong MSG_TYPE {acked_type}!")
                return
            
            now_mono = time.monotonic()
            if result == 0:
                self.get_logger().info(f"[ADAPTER] ACCEPTED: SEQ={acked_seq}")
                self.pub_event_status.publish(String(data=f"ACCEPTED:{acked_seq}:{acked_type}"))

                # Status 1 ve 2: Yalnızca kayıt/ACK, FSM geçişi beklenmez
                if acked_type in (1, 2):
                    del self.pending_commands[acked_seq]
                    return

                # Status 3 ve 4: FSM geçiş takibi
                if acked_type in (3, 4):
                    del self.pending_commands[acked_seq]
                    if cmd.get('early_transition_seen', False):
                        # Log -> ACK sırası: Erken log görülmüştü, ACK ile kesin teyit
                        target_state = cmd.get('early_transition_state', 'UNKNOWN')
                        self.acked_commands[acked_seq] = {
                            'msg_type': acked_type,
                            'x': cmd['x'],
                            'y': cmd['y'],
                            'session_id': acked_sess,
                            'ack_time_mono': now_mono,
                            'first_tx_mono': cmd.get('first_tx_mono', now_mono),
                            'last_tx_mono': cmd.get('last_tx_mono', now_mono),
                            'retries': cmd.get('retries', 0),
                            'successful_txs': cmd.get('successful_txs', 0),
                            'transition_state': 'TRANSITION_CONFIRMED',
                            'transition_time_mono': cmd.get('early_transition_time', now_mono)
                        }
                        self.get_logger().info(f"[ADAPTER] CONFIRMED FSM TRANSITION (Log->ACK order) -> {target_state} (Seq={acked_seq})")
                        self.pub_event_status.publish(String(data=f"CORRELATED_TRANSITION:{target_state}:{acked_seq}:{acked_type}"))
                    else:
                        # ACK -> Log sırası: Henüz log gelmedi, geçiş beklemede
                        self.acked_commands[acked_seq] = {
                            'msg_type': acked_type,
                            'x': cmd['x'],
                            'y': cmd['y'],
                            'session_id': acked_sess,
                            'ack_time_mono': now_mono,
                            'first_tx_mono': cmd.get('first_tx_mono', now_mono),
                            'last_tx_mono': cmd.get('last_tx_mono', now_mono),
                            'retries': cmd.get('retries', 0),
                            'successful_txs': cmd.get('successful_txs', 0),
                            'transition_state': 'ACKED_TRANSITION_PENDING',
                            'transition_time_mono': None
                        }
                        self.pub_event_status.publish(String(data=f"ACKED_TRANSITION_PENDING:{acked_seq}:{acked_type}"))
            elif result == 1:
                self.get_logger().warn(f"[ADAPTER] REJECTED: SEQ={acked_seq}")
                self.pub_event_status.publish(String(data=f"REJECTED:{acked_seq}:{acked_type}"))
                del self.pending_commands[acked_seq]

    def _handle_statustext_msg(self, msg):
        # 1. STATUSTEXT kaynak filtresi: Yalnızca beklenen otopilottan gelen mesajlar işlenir
        src_sys = msg.get_srcSystem() if hasattr(msg, 'get_srcSystem') else getattr(msg, 'srcSystem', 0)
        src_comp = msg.get_srcComponent() if hasattr(msg, 'get_srcComponent') else getattr(msg, 'srcComponent', 0)
        if src_sys != self.sys_id or src_comp != self.comp_id:
            return

        now_mono = time.monotonic()
        text = self.statustext_assembler.feed(msg, now_mono)
        if not text:
            return

        # Genel FSM Durumu Takibi
        if text.startswith("[S500 LUA] Durum Gecisi:"):
            parts = text.split("->")
            if len(parts) > 1:
                target_state = parts[1].split()[0].replace("(", "").strip()
                self.lua_fsm_state = target_state
                self.get_logger().info(f"[ADAPTER] Observed Lua FSM State: {target_state}")
                self.pub_event_status.publish(String(data=f"OBSERVED_STATE:{target_state}"))

        # GOTO_OBSERVATION (Status 3) Korelasyonu
        if "GOTO_OBSERVATION alindi (" in text:
            try:
                log_sess = None
                if "Session ID: " in text:
                    log_sess = int(text.split("Session ID: ")[1].split(",")[0].strip())
                seq = int(text.split("Seq: ")[1].split(",")[0].split(")")[0].strip())
                self._correlate_transition(seq=seq, target_type=3, target_state="HEDEFE_GIT", now_mono=now_mono, log_session_id=log_sess)
            except Exception as e:
                self.get_logger().error(f"[ADAPTER] Failed to parse GOTO_OBSERVATION log: {e}")

        # ROUTE_READY (Status 4) Korelasyonu
        if "ROUTE_READY alindi (" in text:
            try:
                log_sess = None
                if "Session ID: " in text:
                    log_sess = int(text.split("Session ID: ")[1].split(",")[0].strip())
                seq = int(text.split("Seq: ")[1].split(")")[0].split(",")[0].strip())
                self._correlate_transition(seq=seq, target_type=4, target_state="DONUS", now_mono=now_mono, log_session_id=log_sess)
            except Exception as e:
                self.get_logger().error(f"[ADAPTER] Failed to parse ROUTE_READY log: {e}")

    def _correlate_transition(self, seq: int, target_type: int, target_state: str, now_mono: float, log_session_id=None):
        if self.session_state != self.ACTIVE:
            err = f"AMBIGUOUS_SESSION_LOG_IGNORED:{target_type}:seq_{seq}:session_not_active"
            self.get_logger().warn(f"[ADAPTER] {err}")
            self.pub_event_status.publish(String(data=err))
            return

        # 3. Oturumlar Arasında Yanlış Geçiş Doğrulaması Koruması
        if log_session_id is not None:
            if log_session_id != self.session_id:
                err = f"AMBIGUOUS_SESSION_LOG_IGNORED:{target_type}:seq_{seq}:log_session_{log_session_id}_mismatch_current_{self.session_id}"
                self.get_logger().warn(f"[ADAPTER] {err}")
                self.pub_event_status.publish(String(data=err))
                return
        else:
            if self.session_epoch > 1:
                err = f"AMBIGUOUS_SESSION_LOG_IGNORED:{target_type}:seq_{seq}:session_epoch_{self.session_epoch}_lacks_session_in_log"
                self.get_logger().warn(f"[ADAPTER] {err}")
                self.pub_event_status.publish(String(data=err))
                return

        # Sıra A: ACK -> Log (Komut daha önce ACK almış ve beklemede)
        if seq in self.acked_commands:
            cmd = self.acked_commands[seq]
            if cmd['session_id'] != self.session_id or cmd['msg_type'] != target_type:
                err = f"AMBIGUOUS_SESSION_LOG_IGNORED:{target_type}:seq_{seq}:mismatched_session_or_type"
                self.get_logger().warn(f"[ADAPTER] {err}")
                self.pub_event_status.publish(String(data=err))
                return

            if cmd['transition_state'] == 'TRANSITION_CONFIRMED':
                return

            # 3. Zaman aşımına uğrayan kayıt sonradan kesin başarıya dönüşmesin
            if cmd['transition_state'] == 'TRANSITION_TIMEOUT_UNVERIFIED':
                err = f"LATE_TRANSITION_LOG_AFTER_TIMEOUT:{target_type}:seq_{seq}"
                self.get_logger().warn(f"[ADAPTER] {err}")
                self.pub_event_status.publish(String(data=err))
                return

            # Başarılı gönderim kanıtı kontrolü
            if cmd.get('successful_txs', 0) == 0 or cmd.get('first_tx_mono', 0.0) <= 0.0:
                err = f"AMBIGUOUS_SESSION_LOG_IGNORED:{target_type}:seq_{seq}:not_transmitted_yet"
                self.get_logger().warn(f"[ADAPTER] {err}")
                self.pub_event_status.publish(String(data=err))
                return

            if now_mono < cmd.get('first_tx_mono', 0.0):
                err = f"AMBIGUOUS_TIMESTAMP_LOG_IGNORED:{target_type}:seq_{seq}:arrived_before_tx"
                self.get_logger().warn(f"[ADAPTER] {err}")
                self.pub_event_status.publish(String(data=err))
                return

            cmd['transition_state'] = 'TRANSITION_CONFIRMED'
            cmd['transition_time_mono'] = now_mono
            self.get_logger().info(f"[ADAPTER] CONFIRMED FSM TRANSITION (ACK->Log order) -> {target_state} (Seq={seq})")
            self.pub_event_status.publish(String(data=f"CORRELATED_TRANSITION:{target_state}:{seq}:{target_type}"))
            return

        # Sıra B: Log -> ACK (Komut gönderilmiş fakat henüz ACK gelmemiş)
        if seq in self.pending_commands:
            cmd = self.pending_commands[seq]
            if cmd['session_id'] != self.session_id or cmd['msg_type'] != target_type:
                err = f"AMBIGUOUS_SESSION_LOG_IGNORED:{target_type}:seq_{seq}:mismatched_session_or_type"
                self.get_logger().warn(f"[ADAPTER] {err}")
                self.pub_event_status.publish(String(data=err))
                return

            # Başarılı gönderim kanıtı kontrolü
            if cmd.get('successful_txs', 0) == 0 or cmd.get('first_tx_mono', 0.0) <= 0.0:
                err = f"AMBIGUOUS_SESSION_LOG_IGNORED:{target_type}:seq_{seq}:not_transmitted_yet"
                self.get_logger().warn(f"[ADAPTER] {err}")
                self.pub_event_status.publish(String(data=err))
                return

            cmd['early_transition_seen'] = True
            cmd['early_transition_state'] = target_state
            cmd['early_transition_time'] = now_mono
            self.get_logger().info(f"[ADAPTER] Early transition log observed before ACK for Seq={seq} -> {target_state}")
            self.pub_event_status.publish(String(data=f"EARLY_TRANSITION_OBSERVED:{target_state}:{seq}:{target_type}"))
            return

        # Belirsiz / Takip edilmeyen log durumu
        err = f"AMBIGUOUS_SESSION_LOG_IGNORED:{target_type}:seq_{seq}:untracked_in_session"
        self.get_logger().warn(f"[ADAPTER] {err}")
        self.pub_event_status.publish(String(data=err))

    def retry_loop(self):
        self.process_rx_queue()
        
        now_mono = time.monotonic()
        
        # 5. ACK sonrası bekleme süresi kontrolü (ACKED_TRANSITION_PENDING zaman aşımı)
        for aseq, acmd in list(self.acked_commands.items()):
            if acmd.get('transition_state') == 'ACKED_TRANSITION_PENDING':
                elapsed = now_mono - acmd['ack_time_mono']
                if elapsed > self.transition_timeout:
                    acmd['transition_state'] = 'TRANSITION_TIMEOUT_UNVERIFIED'
                    err = f"TRANSITION_TIMEOUT_UNVERIFIED:{acmd['msg_type']}:seq_{aseq}:after_{elapsed:.1f}s"
                    self.get_logger().warn(f"[ADAPTER] {err}")
                    self.pub_event_status.publish(String(data=err))

        if self.session_state == self.STARTING and self.handshake_pending is not None:
            cmd = self.handshake_pending
            if now_mono - cmd['last_tx_mono'] > (1.0 / self.retry_hz):
                if cmd['retries'] >= 5:
                    if cmd['successful_txs'] == 0:
                        self.get_logger().error(f"[ADAPTER] SESSION_START TX_FAILED (After {cmd['retries']} attempts).")
                        self.pub_event_status.publish(String(data=f"HANDSHAKE_TX_FAILED:{cmd['seq']}"))
                    else:
                        self.get_logger().error(f"[ADAPTER] SESSION_START TIMEOUT (After {cmd['retries']} tries).")
                        self.pub_event_status.publish(String(data=f"HANDSHAKE_TIMEOUT:{cmd['seq']}"))
                    self.session_state = self.IDLE
                    self.handshake_pending = None
                else:
                    success = self.send_user_cmd(0.0, 0.0, 0.0, cmd['seq'], 0xAC)
                    if success:
                        cmd['successful_txs'] += 1
                    cmd['last_tx_mono'] = now_mono
                    cmd['retries'] += 1
            return
            
        if self.session_state != self.ACTIVE:
             return
             
        ros_now = self.get_clock().now()
        
        to_delete = []
        for seq, cmd in list(self.pending_commands.items()):
            # 2. Erken log görüldüğünde zaman aşımı kontrolü (Log -> ACK sırası)
            # Lua komutu işlediğinden tekrar gönderilmez; ancak gecikmiş ACK için sınırlı bekleme uygulanır
            if cmd.get('early_transition_seen', False):
                ack_wait_time = now_mono - cmd.get('early_transition_time', cmd['queue_ts_mono'])
                if ack_wait_time > self.transition_timeout:
                    err = f"EARLY_TRANSITION_ACK_TIMEOUT_UNVERIFIED:{cmd['msg_type']}:seq_{seq}:after_{ack_wait_time:.1f}s"
                    self.get_logger().warn(f"[ADAPTER] {err}")
                    self.pub_event_status.publish(String(data=err))
                    to_delete.append(seq)
                continue

            # İzin verilen durum kontrolü (eğer erken log görülmediyse)
            allowed_states = cmd.get('allowed_states')
            if allowed_states is not None and self.lua_fsm_state not in allowed_states:
                self.get_logger().warn(f"[ADAPTER] Event SEQ={seq}/{cmd['msg_type']} DROPPED: FSM state is {self.lua_fsm_state} (not in {allowed_states})")
                self.pub_event_status.publish(String(data=f"DROPPED_INVALID_STATE:{seq}:{cmd['msg_type']}"))
                to_delete.append(seq)
                continue

            age_s = (ros_now - cmd['source_time']).nanoseconds / 1e9
            if age_s < 0.0 or age_s > self.stale_timeout:
                self.get_logger().error(f"[ADAPTER] Event SEQ={seq} EXPIRED due to Source Data Age ({age_s:.1f} > {self.stale_timeout}s).")
                self.pub_event_status.publish(String(data=f"EXPIRED:{seq}:{cmd['msg_type']}"))
                to_delete.append(seq)
                continue
                
            queue_lifetime = now_mono - cmd['queue_ts_mono']
            if queue_lifetime > self.expiry_s:
                self.get_logger().error(f"[ADAPTER] Event SEQ={seq} EXPIRED due to Queue Lifetime ({queue_lifetime:.1f} > {self.expiry_s}s).")
                self.pub_event_status.publish(String(data=f"EXPIRED:{seq}:{cmd['msg_type']}"))
                to_delete.append(seq)
                continue
                
            if now_mono - cmd['last_tx_mono'] > (1.0 / self.retry_hz):
                if cmd['retries'] >= 5:
                    self.get_logger().error(f"[ADAPTER] Event SEQ={seq}/{cmd['msg_type']} TIMEOUT (After {cmd['retries']} tries).")
                    self.pub_event_status.publish(String(data=f"TIMEOUT:{seq}:{cmd['msg_type']}"))
                    to_delete.append(seq)
                    continue
                
                # Başarılı gönderim kontrolü: first_tx_mono ve successful_txs yalnızca success durumunda artar
                success = self.send_user_cmd(cmd['x'], cmd['y'], cmd['conf'], seq, cmd['msg_type'])
                if success:
                    cmd['successful_txs'] += 1
                    if cmd['first_tx_mono'] == 0.0:
                        cmd['first_tx_mono'] = now_mono
                cmd['last_tx_mono'] = now_mono
                cmd['retries'] += 1
                
        for seq in to_delete:
            if seq in self.pending_commands:
                del self.pending_commands[seq]

    def liveliness_loop(self):
        if self.session_state != self.ACTIVE:
            return

        ros_now = self.get_clock().now()
        
        if self.last_vision_source_time is None:
            self.current_liveliness_state = 0
        else:
            age_s = (ros_now - self.last_vision_source_time).nanoseconds / 1e9
            if age_s < 0.0 or age_s > self.stale_timeout:
                self.current_liveliness_state = 0
            
        try:
            self.master.mav.command_long_send(
                self.sys_id, self.comp_id, 31010, 0,
                float(self.current_liveliness_state), 0.0, 0.0, 0.0,
                float(0xAA), float(self.session_id), 1.0
            ) 
        except Exception as e:
            now = time.monotonic()
            if now - self.last_tx_err_log_time > 2.0:
                self.get_logger().error(f"Liveliness Tx Error: {e}")
                self.last_tx_err_log_time = now

    def send_user_cmd(self, x, y, conf, seq, mtype):
        try:
            self.master.mav.command_long_send(
                self.sys_id, self.comp_id, 31010, 0,
                float(x), float(y), float(conf), float(seq),
                float(mtype), float(self.session_id), 1.0
            )
            return True
        except Exception as e:
            now = time.monotonic()
            if now - self.last_tx_err_log_time > 2.0:
                self.get_logger().error(f"Event Tx Error: {e}")
                self.last_tx_err_log_time = now
            return False

    def is_valid_int(self, val):
        return (not math.isnan(val)) and (not math.isinf(val)) and float(val).is_integer()

    def mavlink_rx_thread(self):
        while self._running:
            try:
                msg = self.master.recv_match(type=['COMMAND_LONG', 'HEARTBEAT', 'STATUSTEXT'], blocking=True, timeout=0.5)
                if not msg:
                    continue
                
                if msg.get_type() == 'HEARTBEAT':
                    continue

                # 1. STATUSTEXT kaynak filtresi: Yalnızca beklenen otopilot kuyruğa alınır
                if msg.get_type() == 'STATUSTEXT':
                    if msg.get_srcSystem() == self.sys_id and msg.get_srcComponent() == self.comp_id:
                        self.rx_queue.put(msg, block=False)
                    continue

                if msg.get_type() == 'COMMAND_LONG':
                    if msg.target_system != self.my_sys or msg.target_component != self.my_comp:
                        continue
                        
                    if msg.get_srcSystem() != self.sys_id or msg.get_srcComponent() != self.comp_id:
                        continue
                        
                    if not (self.is_valid_int(msg.param5) and int(msg.param5) == 0xAB):
                        continue
                        
                    if not (self.is_valid_int(msg.command) and int(msg.command) == 31010):
                        continue
                        
                    if not (self.is_valid_int(msg.param7) and int(msg.param7) == 1):
                        continue
                        
                    if not (self.is_valid_int(msg.param1) and self.is_valid_int(msg.param2) and 
                            self.is_valid_int(msg.param4) and self.is_valid_int(msg.param6)):
                        continue
                        
                    self.rx_queue.put(msg, block=False)
                    
            except queue.Full:
                now = time.monotonic()
                if now - self.last_rx_err_log_time > 2.0:
                    self.get_logger().error("[ADAPTER] MAVLink RX Queue is FULL, dropping incoming payload.")
                    self.last_rx_err_log_time = now
            except Exception as e:
                now = time.monotonic()
                if now - self.last_rx_err_log_time > 5.0:
                    self.get_logger().error(f"[ADAPTER Dbg] RX Thread handled exception: {e}")
                    self.last_rx_err_log_time = now
                
    def destroy_node(self):
        self._running = False
        if self.rx_thread.is_alive():
             self.rx_thread.join(timeout=1.0)
        try:
             self.master.close()
        except Exception:
             pass
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = MavlinkAdapterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
