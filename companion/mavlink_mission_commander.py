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
from std_msgs.msg import String

from vision_interfaces.msg import DetectionArray
import sys
import os

# Add local path for class_mapping resolution explicitly if running standalone
sys.path.append(os.path.dirname(os.path.abspath(__file__)))
from class_mapping import ClassMapper


class MavlinkAdapterNode(Node):
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
        
        self._running = True
        self.session_id = random.randint(1, 16000000)
        
        # Session is strictly blocked until handshake enables it.
        # UNIMPLEMENTED: Handshake logic (e.g. ROS Service toggle via '/mission/start').
        self.session_active = False 
        
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
        self.seq_counter = 1
        self.last_sent_types = {1: None, 2: None}
        
        self.sub_detections = self.create_subscription(
            DetectionArray, '/vision/detections', self.det_callback, qos_profile_sensor_data
        )
        self.pub_event_status = self.create_publisher(String, '/adapter/event_status', 10)
        
        self.retry_timer = self.create_timer(1.0 / self.retry_hz, self.retry_loop)
        self.live_timer = self.create_timer(1.0 / self.live_hz, self.liveliness_loop)
        
        self.rx_thread = threading.Thread(target=self.mavlink_rx_thread, daemon=True)
        self.rx_thread.start()

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

        if not self.session_active or is_stale:
            return
            
        for d, msg_type in valid_targets:
            x_ned = d.local_y
            y_ned = d.local_x
            conf = d.confidence
            self.queue_command(msg_type, x_ned, y_ned, conf, msg_time)

    def queue_command(self, mtype, x, y, conf, msg_time_ros):
        if not self.session_active:
            return
            
        if len(self.pending_commands) >= self.max_queue:
            return
            
        now_mono = time.monotonic()
        
        prev = self.last_sent_types.get(mtype)
        if prev is not None:
            time_diff = now_mono - prev['ts_mono']
            dist = math.hypot(prev['x'] - x, prev['y'] - y)
            if dist < self.spam_dist and time_diff < self.spam_timeout:
                return

        for k, v in self.pending_commands.items():
            if v['msg_type'] == mtype and math.hypot(v['x'] - x, v['y'] - y) < self.spam_dist:
                return
                
        seq = self.seq_counter
        if self.seq_counter >= 16777215:
            self.get_logger().error("[ADAPTER FATAL] SEQ reached Float32 capacity (16777215). Controlled new session needed!")
            self.session_active = False
            self.pub_event_status.publish(String(data="FATAL_SESSION_LIMIT"))
            return
            
        self.seq_counter += 1
        
        self.pending_commands[seq] = {
            'msg_type': mtype,
            'x': float(x),
            'y': float(y),
            'conf': float(conf),
            'source_time': msg_time_ros,
            'queue_ts_mono': now_mono,
            'last_tx_mono': 0.0,
            'retries': 0
        }
        
        self.last_sent_types[mtype] = {'x': x, 'y': y, 'ts_mono': now_mono}

    def process_rx_queue(self):
        while not self.rx_queue.empty():
            try:
                msg = self.rx_queue.get_nowait()
                self._handle_ack_msg(msg)
            except queue.Empty:
                break

    def _handle_ack_msg(self, msg):
        if not self.session_active:
            return
            
        acked_type = int(msg.param1)
        result = int(msg.param2)
        acked_seq = int(msg.param4)
        acked_sess = int(msg.param6)
        
        if result not in [0, 1]:
            return
            
        if acked_sess != self.session_id:
            return
            
        if acked_seq in self.pending_commands:
            cmd = self.pending_commands[acked_seq]
            
            if cmd['msg_type'] != acked_type:
                self.get_logger().error(f"[PROTOCOL] Lua ACKed SEQ={acked_seq} with wrong MSG_TYPE {acked_type}!")
                return
            
            if result == 0:
                self.get_logger().info(f"[ADAPTER] ACCEPTED: SEQ={acked_seq}")
                self.pub_event_status.publish(String(data=f"ACCEPTED:{acked_seq}:{acked_type}"))
            elif result == 1:
                self.get_logger().warn(f"[ADAPTER] REJECTED: SEQ={acked_seq}")
                self.pub_event_status.publish(String(data=f"REJECTED:{acked_seq}:{acked_type}"))
                
            del self.pending_commands[acked_seq]

    def retry_loop(self):
        self.process_rx_queue()
        
        if not self.session_active:
             return
             
        now_mono = time.monotonic()
        ros_now = self.get_clock().now()
        
        to_delete = []
        for seq, cmd in self.pending_commands.items():
            # Expiry is exclusively determined by the source observation age against ROS Time
            # Separating Source limits from Queue lifetime
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
                
                self.send_user_cmd(cmd['x'], cmd['y'], cmd['conf'], seq, cmd['msg_type'])
                cmd['last_tx_mono'] = now_mono
                cmd['retries'] += 1
                
        for seq in to_delete:
            del self.pending_commands[seq]

    def liveliness_loop(self):
        if not self.session_active:
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
        except Exception as e:
            now = time.monotonic()
            if now - self.last_tx_err_log_time > 2.0:
                self.get_logger().error(f"Event Tx Error: {e}")
                self.last_tx_err_log_time = now

    def is_valid_int(self, val):
        return (not math.isnan(val)) and (not math.isinf(val)) and float(val).is_integer()

    def mavlink_rx_thread(self):
        while self._running:
            try:
                msg = self.master.recv_match(type=['COMMAND_LONG', 'HEARTBEAT'], blocking=True, timeout=0.5)
                if not msg:
                    continue
                
                if msg.get_type() == 'HEARTBEAT':
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
