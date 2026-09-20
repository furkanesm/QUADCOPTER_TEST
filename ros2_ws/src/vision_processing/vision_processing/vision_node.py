#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
from vision_interfaces.msg import Detection, DetectionArray
import cv2
import math
import os
import glob
import time
from typing import Optional, List

try:
    from ultralytics import YOLO
except ImportError:
    pass  # Node will drop out if YOLO doesn't exist at runtime

from .geometry import RaycastProjector

class ObjectTrack:
    def __init__(self, t_id, cls_id, cls_name, first_time):
        self.track_id = t_id
        self.class_id = cls_id
        self.class_name = cls_name
        self.last_seen = first_time
        self.seen_count = 1
        
        # Current state
        self.cx = 0.0
        self.cy = 0.0
        self.metric_x = 0.0
        self.metric_y = 0.0
        self.metric_valid = False
        self.conf = 0.0

class MultiObjectTracker:
    def __init__(self, timeout_s=2.0, pix_thresh=40.0, world_thresh_m=2.0):
        self.tracks = []
        self.timeout_s = timeout_s
        self.pix_thresh = pix_thresh
        self.world_thresh_m = world_thresh_m
        self.next_id = 1
        
    def update(self, current_time, detections: List[dict]):
        # Age tracks
        self.tracks = [t for t in self.tracks if (current_time - t.last_seen) < self.timeout_s]
        
        assigned = []
        for det in detections:
            best_t = None
            best_dist = float('inf')
            
            for t in self.tracks:
                if t in assigned or t.class_id != det['cls_id']:
                    continue
                    
                if det['metric_valid'] and t.metric_valid:
                    dist = math.hypot(det['metric_x'] - t.metric_x, det['metric_y'] - t.metric_y)
                    if dist < self.world_thresh_m and dist < best_dist:
                        best_dist = dist
                        best_t = t
                else:
                    dist = math.hypot(det['cx'] - t.cx, det['cy'] - t.cy)
                    if dist < self.pix_thresh and dist < best_dist:
                        best_dist = dist
                        best_t = t
                        
            if best_t is not None:
                assigned.append(best_t)
                best_t.last_seen = current_time
                best_t.seen_count += 1
                best_t.cx = det['cx']
                best_t.cy = det['cy']
                best_t.metric_x = det['metric_x']
                best_t.metric_y = det['metric_y']
                best_t.metric_valid = det['metric_valid']
                best_t.conf = det['conf']
                det['track_id'] = best_t.track_id
            else:
                new_t = ObjectTrack(self.next_id, det['cls_id'], det['cls_name'], current_time)
                self.next_id += 1
                new_t.cx = det['cx']
                new_t.cy = det['cy']
                new_t.metric_x = det['metric_x']
                new_t.metric_y = det['metric_y']
                new_t.metric_valid = det['metric_valid']
                new_t.conf = det['conf']
                self.tracks.append(new_t)
                det['track_id'] = new_t.track_id
                
        return self.tracks

class VisionNode(Node):
    def __init__(self):
        super().__init__('vision_node')
        
        self.declare_parameter('model_path', 'best.pt')
        self.declare_parameter('confidence_threshold', 0.55)
        self.declare_parameter('inference_rate_hz', 10.0)
        self.declare_parameter('device', 'cpu')
        self.declare_parameter('class_filter', ['']) 
        self.declare_parameter('publish_debug_image', False)
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('pose_topic', '/mavros/local_position/pose')
        self.declare_parameter('imgsz', 640)
        self.declare_parameter('test_image_dir', '')
        
        # Geometry Config
        self.declare_parameter('has_calibration', False)
        self.declare_parameter('cam_fx', 1200.0)
        self.declare_parameter('cam_fy', 1200.0)
        self.declare_parameter('cam_cx', 960.0)
        self.declare_parameter('cam_cy', 600.0)
        self.declare_parameter('cam_dists', [0.0, 0.0, 0.0, 0.0, 0.0])
        self.declare_parameter('mount_rpy', [0.0, 0.0, 0.0]) # Radians
        self.declare_parameter('mount_xyz', [0.0, 0.0, 0.0]) # Meters
        self.declare_parameter('ground_plane_z', 0.0)
        
        # Telemetry Time Sync
        self.declare_parameter('pose_age_tolerance_s', 0.2)
        
        # Fetch generic config
        model_path = self.get_parameter('model_path').value
        self.conf_thres = self.get_parameter('confidence_threshold').value
        self.rate_hz = self.get_parameter('inference_rate_hz').value
        self.device = self.get_parameter('device').value
        self.class_filter = self.get_parameter('class_filter').value
        if len(self.class_filter) == 1 and self.class_filter[0] == '':
            self.class_filter = []
            
        self.pub_debug = self.get_parameter('publish_debug_image').value
        image_topic = self.get_parameter('image_topic').value
        pose_topic = self.get_parameter('pose_topic').value
        self.imgsz = self.get_parameter('imgsz').value
        self.test_dir = self.get_parameter('test_image_dir').value
        
        # Fetch geometry config
        self.has_calib = self.get_parameter('has_calibration').value
        fx = self.get_parameter('cam_fx').value
        fy = self.get_parameter('cam_fy').value
        cx = self.get_parameter('cam_cx').value
        cy = self.get_parameter('cam_cy').value
        dists = self.get_parameter('cam_dists').value
        mrpy = self.get_parameter('mount_rpy').value
        mxyz = self.get_parameter('mount_xyz').value
        self.gnd_z = self.get_parameter('ground_plane_z').value
        self.pose_age_tol = self.get_parameter('pose_age_tolerance_s').value
        
        if self.has_calib:
            self.projector = RaycastProjector(fx, fy, cx, cy, dists, mxyz, mrpy)
            self.get_logger().info("[VISION] RaycastProjector ENABLED with provided configuration.")
        else:
            self.projector = None
            self.get_logger().warn("[VISION] Running WITHOUT valid camera calibration. Position mapping will be SKIPPED.")

        self.bridge = CvBridge()
        self.latest_frame = None
        self.latest_frame_ts = None
        self.latest_pose = None
        self.latest_pose_ts = None
        
        self.tracker = MultiObjectTracker(timeout_s=3.0)

        # Model Init
        self.model = YOLO(model_path)
        self.model.to(self.device)
        self.class_names = self.model.names

        self.is_test_mode = len(self.test_dir) > 0 and os.path.isdir(self.test_dir)
        if self.is_test_mode:
            self.test_images = sorted(glob.glob(os.path.join(self.test_dir, '*.[jp][pn]*[g]')))
            self.test_img_idx = 0
            self.get_logger().warn("[VISION] TEST MODU AKTIF.")
        else:
            self.sub_image = self.create_subscription(
                Image, image_topic, self.image_callback, qos_profile_sensor_data)
        
        self.sub_pose = self.create_subscription(
            PoseStamped, pose_topic, self.pose_callback, qos_profile_sensor_data)

        self.pub_detections = self.create_publisher(DetectionArray, '/vision/detections', 10)
        
        if self.pub_debug:
            self.pub_debug_img = self.create_publisher(Image, '/vision/debug_image', 1)

        self.timer = self.create_timer(1.0 / self.rate_hz, self.inference_loop)

        self.get_logger().info("[VISION] Node configured and ready.")

    def image_callback(self, msg: Image):
        self.latest_frame = msg
        self.latest_frame_ts = time.time()

    def pose_callback(self, msg: PoseStamped):
        self.latest_pose = msg
        self.latest_pose_ts = time.time()

    def inference_loop(self):
        cv_image = None
        now = time.time()
        
        if self.is_test_mode:
            if len(self.test_images) == 0:
                return
            img_path = self.test_images[self.test_img_idx]
            cv_image = cv2.imread(img_path)
            self.test_img_idx = (self.test_img_idx + 1) % len(self.test_images)
        else:
            if self.latest_frame is None:
                return
                
            frame_age = now - self.latest_frame_ts
            if frame_age > 1.0:
                # self.get_logger().warn("Camera frame is too old.")
                return
                
            try:
                cv_image = self.bridge.imgmsg_to_cv2(self.latest_frame, desired_encoding='bgr8')
            except Exception as e:
                self.get_logger().error(f"Baglanti Hatasi (ImgMsb->CV2): {e}")
                return

        # Prepare outputs
        results = self.model.predict(
            source=cv_image, 
            imgsz=self.imgsz, 
            device=self.device, 
            conf=self.conf_thres, 
            verbose=False
        )

        det_msg = DetectionArray()
        det_msg.header.stamp = self.get_clock().now().to_msg()
        det_msg.altitude_used = float('nan')
        
        # Check pose limits
        valid_pose = False
        if self.latest_pose_ts is not None and abs(now - self.latest_pose_ts) <= self.pose_age_tol:
            valid_pose = True
            det_msg.header.frame_id = self.latest_pose.header.frame_id
        
        det_msg.pose_valid = valid_pose

        current_detections = []

        for r in results:
            boxes = r.boxes
            for box in boxes:
                cls_id = int(box.cls[0])
                cls_name = self.class_names[cls_id]
                conf = float(box.conf[0])
                
                if len(self.class_filter) > 0 and cls_name not in self.class_filter:
                    continue

                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                cx = float((x1 + x2) / 2.0)
                cy = float((y1 + y2) / 2.0)
                
                det_dict = {
                    'cls_id': cls_id, 'cls_name': cls_name, 'conf': conf,
                    'cx': cx, 'cy': cy, 'w': float(x2-x1), 'h': float(y2-y1),
                    'metric_x': float('nan'), 'metric_y': float('nan'),
                    'metric_valid': False, 'reason': 'NOT_COMPUTED'
                }
                
                if not self.has_calib:
                    det_dict['reason'] = 'NO_CALIBRATION'
                elif not valid_pose:
                    det_dict['reason'] = 'POSE_STALE_OR_MISSING'
                else:
                    px = self.latest_pose.pose.position.x
                    py = self.latest_pose.pose.position.y
                    pz = self.latest_pose.pose.position.z
                    qx = self.latest_pose.pose.orientation.x
                    qy = self.latest_pose.pose.orientation.y
                    qz = self.latest_pose.pose.orientation.z
                    qw = self.latest_pose.pose.orientation.w
                    
                    proj = self.projector.unproject_pixel(
                        cx, y2, # Target contact point is usually the bottom of the bounding box
                        [px, py, pz], [qx, qy, qz, qw],
                        self.gnd_z
                    )
                    
                    if proj['valid']:
                        det_dict['metric_valid'] = True
                        det_dict['metric_x'] = proj['x']
                        det_dict['metric_y'] = proj['y']
                        det_dict['reason'] = 'OK'
                    else:
                        det_dict['reason'] = proj['reason']

                current_detections.append(det_dict)
                
                if self.pub_debug:
                    cv2.rectangle(cv_image, (int(x1), int(y1)), (int(x2), int(y2)), (0,150,0), 2)
                    cv2.putText(cv_image, f"{cls_name} {conf:.2f}", (int(x1), int(y1)-5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,150,0), 2)

        # Let the tracker enforce consistency
        self.tracker.update(now, current_detections)

        # Translate track states into output messages
        for det in current_detections:
            d = Detection()
            d.class_name = det['cls_name']
            d.class_id = det['cls_id']
            d.confidence = det['conf']
            d.bbox_cx = det['cx']
            d.bbox_cy = det['cy']
            d.bbox_w = det['w']
            d.bbox_h = det['h']
            
            # The receiver knows this is an ENU mapping if they inspect header frame_id.
            # Local translation from ENU to NED is handled by the Jetson-Lua adapter, 
            # NOT the vision node, as the vision node shouldn't mangle its own frame's conventions.
            d.position_valid = det['metric_valid']
            d.local_x = det['metric_x']
            d.local_y = det['metric_y']
            
            det_msg.detections.append(d)
                
            if self.pub_debug:
                if det['metric_valid']:
                    cv2.putText(cv_image, f"M:({det['metric_x']:.1f}, {det['metric_y']:.1f})", (int(det['cx'])-20, int(det['cy'])), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0,255,255), 1)

        self.pub_detections.publish(det_msg)

        if self.pub_debug:
            try:
                dbg_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
                self.pub_debug_img.publish(dbg_msg)
            except Exception:
                pass


def main(args=None):
    rclpy.init(args=args)
    node = VisionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
