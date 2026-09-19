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
from typing import Optional, List

try:
    from ultralytics import YOLO
except ImportError:
    pass  # We will handle it in the node

class VisionNode(Node):
    def __init__(self):
        super().__init__('vision_node')
        
        # 1. Deklare Edilen Parametreler
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

        # Parametreleri al
        model_path = self.get_parameter('model_path').value
        self.conf_thres = self.get_parameter('confidence_threshold').value
        self.rate_hz = self.get_parameter('inference_rate_hz').value
        self.device = self.get_parameter('device').value
        self.class_filter = self.get_parameter('class_filter').value
        if len(self.class_filter) == 1 and self.class_filter[0] == '':
            self.class_filter = [] # Boş liste -> Hepsi
            
        self.pub_debug = self.get_parameter('publish_debug_image').value
        image_topic = self.get_parameter('image_topic').value
        pose_topic = self.get_parameter('pose_topic').value
        self.imgsz = self.get_parameter('imgsz').value
        self.test_dir = self.get_parameter('test_image_dir').value

        self.bridge = CvBridge()
        self.latest_frame = None
        self.latest_pose: Optional[PoseStamped] = None

        # Model Yükleme
        try:
            self.model = YOLO(model_path)
            self.get_logger().info(f"[VISION] Model yuklendi: {model_path} (Cihaz: {self.device})")
        except Exception as e:
            self.get_logger().error(f"[VISION] Model yuklenemedi! Hata: {str(e)}")
            raise e

        self.model.to(self.device)
        self.class_names = self.model.names
        
        model_train_imgsz = self.model.model.args.get('imgsz', 'Bilinmiyor') if hasattr(self.model, 'model') and hasattr(self.model.model, 'args') else 'Bilinmiyor'
        self.get_logger().info(f"[VISION] Sınıf Listesi ({len(self.class_names)}): {self.class_names}")
        self.get_logger().info(f"[VISION] Model Egitim Çözünürlüğü (imgsz): {model_train_imgsz}")
        self.get_logger().info(f"[VISION] Hedef Infernece imgsz: {self.imgsz}")

        # Test Dir vs Topic Secimi
        self.is_test_mode = len(self.test_dir) > 0 and os.path.isdir(self.test_dir)
        if self.is_test_mode:
            self.test_images = sorted(glob.glob(os.path.join(self.test_dir, '*.[jp][pn]*[g]')))
            self.test_img_idx = 0
            self.get_logger().warn(f"[VISION] TEST MODU AKTIF. {len(self.test_images)} resim {self.test_dir} icinden kullanilacak.")
        else:
            # Publisher ve Subscriberlar
            self.sub_image = self.create_subscription(
                Image, image_topic, self.image_callback, qos_profile_sensor_data)
            self.get_logger().info(f"[VISION] Abone olundu: {image_topic} (Sensor QoS)")

        self.sub_pose = self.create_subscription(
            PoseStamped, pose_topic, self.pose_callback, qos_profile_sensor_data)
        self.get_logger().info(f"[VISION] Abone olundu: {pose_topic} (Sensor QoS)")

        self.pub_detections = self.create_publisher(DetectionArray, '/vision/detections', 10)
        
        if self.pub_debug:
            self.pub_debug_img = self.create_publisher(Image, '/vision/debug_image', 1)

        self.timer = self.create_timer(1.0 / self.rate_hz, self.inference_loop)
        self.first_frame_checked = False

    def image_callback(self, msg: Image):
        # Sadece son kareyi tut, islenmeyenler atilir (Latest drop-oldest davranisi)
        self.latest_frame = msg

    def pose_callback(self, msg: PoseStamped):
        self.latest_pose = msg

    def inference_loop(self):
        cv_image = None
        if self.is_test_mode:
            if len(self.test_images) == 0:
                return
            img_path = self.test_images[self.test_img_idx]
            cv_image = cv2.imread(img_path)
            self.test_img_idx = (self.test_img_idx + 1) % len(self.test_images)
        else:
            if self.latest_frame is None:
                return
            try:
                cv_image = self.bridge.imgmsg_to_cv2(self.latest_frame, desired_encoding='bgr8')
                self.latest_frame = None  # Islendi olarak isaretle
            except Exception as e:
                self.get_logger().error(f"Baglanti Hatasi (ImgMsb->CV2): {e}")
                return

        if cv_image is None:
            return

        h, w = cv_image.shape[:2]
        
        # Ilk frame AR uyarisi
        if not self.first_frame_checked:
            self.get_logger().info(f"[VISION] Ilk Gelen Görüntü Boyutu: {w}x{h}")
            aspect_ratio = w / float(h)
            if abs(aspect_ratio - 1.0) < 0.1:  # Kameranın kare oldugunu gosterir
                # Egitim 16:9 model ise AR farki olacaktir
                # Modelimiz genellikle 16:9 veya kare olabilir. Eger model 16:9 ile egitildiyse
                # kare resimde daralma/cekilme yasayabilir, letterbox iyi uygulanmali.
                # Ultralytics letterbox otomatik yapar, ancak uyari birakiyoruz.
                self.get_logger().warn("[VISION UYARI] Gelen goruntu KARE (~1:1). "
                    "Eger orjinal Model egitimi 16:9 ise Domain Shift ve FOV'da kayiplar (padding nedeniyle) olabilir!")
            self.first_frame_checked = True

        results = self.model.predict(
            source=cv_image, 
            imgsz=self.imgsz, 
            device=self.device, 
            conf=self.conf_thres, 
            verbose=False
        )

        det_msg = DetectionArray()
        det_msg.header.stamp = self.get_clock().now().to_msg()
        det_msg.pose_valid = False
        det_msg.altitude_used = float('nan')
        
        if self.latest_pose:
            det_msg.header.frame_id = self.latest_pose.header.frame_id
        
        # Sonuclari isleme (Ultralytics zaten resize'ı orjinal resim coords'ina cevirir)
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
                
                d = Detection()
                d.class_name = cls_name
                d.class_id = cls_id
                d.confidence = conf
                d.bbox_cx = cx
                d.bbox_cy = cy
                d.bbox_w = float(x2 - x1)
                d.bbox_h = float(y2 - y1)
                
                # FAZ 1 : Dönüsüm yok
                d.position_valid = False
                d.local_x = float('nan')
                d.local_y = float('nan')
                
                det_msg.detections.append(d)
                
                if self.pub_debug:
                    cv2.rectangle(cv_image, (int(x1), int(y1)), (int(x2), int(y2)), (0,255,0), 2)
                    cv2.putText(cv_image, f"{cls_name} {conf:.2f}", (int(x1), int(y1)-5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 2)

        self.pub_detections.publish(det_msg)

        if self.pub_debug:
            try:
                dbg_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
                self.pub_debug_img.publish(dbg_msg)
            except Exception as e:
                pass

def main(args=None):
    rclpy.init(args=args)
    node = VisionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
