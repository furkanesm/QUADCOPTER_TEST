"""
detector.py — YOLO Inference Modülü
=====================================
Orijinal kaynak: otonomi_kontrol/src/vision_processing/vision_processing/vision_node.py

NEDEN BU ŞEKİLDE:
- vision_node.py ROS2'ye bağımlıydı (rclpy, sensor_msgs, cv_bridge).
  Bu dosya ROS2 bağımlılığını tamamen kaldırıp sadece ultralytics + opencv kullanır.
- Sınıf yapısı korundu: model yükleme → frame al → inference → sonuç döndür.
- Detection sonuçları basit bir namedtuple olarak döner,
  böylece ana script kolayca filtreleme yapabilir.
- detect() iki liste döner: targets (hedef) ve obstacles (engel).
"""

import os
import sys
from collections import namedtuple
from typing import List, Optional, Tuple

import cv2

# ── Detection sonuç yapısı ──────────────────────────────
# Her bir tespit için: sınıf adı, sınıf id, güven skoru, bbox merkezi ve bbox köşeleri (piksel)
Detection = namedtuple("Detection", [
    "class_name", "class_id", "confidence",
    "center_x", "center_y",
    "x1", "y1", "x2", "y2",
])


class TargetDetector:
    """
    best.pt modeliyle YOLO inference çalıştıran minimal sınıf.

    Kullanım:
        detector = TargetDetector("../best.pt", target_class_id=0, target_class_name="Hedef")
        targets, obstacles = detector.detect(cv_image)
        for d in targets:
            print(f"{d.class_name}: {d.confidence:.1%}")
    """

    def __init__(self, model_path: str, confidence_threshold: float = 0.55,
                 target_class_name: Optional[str] = None,
                 target_class_id: int = 0):
        """
        Args:
            model_path: best.pt dosyasının yolu
            confidence_threshold: Minimum güven eşiği (0-1 arası)
            target_class_name: Aranan sınıf adı ('Hedef').
            target_class_id: Aranan sınıf ID'si (0).
        """
        try:
            from ultralytics import YOLO
        except ImportError:
            print("HATA: 'ultralytics' paketi bulunamadı.")
            print("Kurulum: pip install ultralytics")
            sys.exit(1)

        abs_path = os.path.abspath(model_path)
        if not os.path.isfile(abs_path):
            print(f"HATA: Model dosyası bulunamadı: {abs_path}")
            sys.exit(1)

        self.model = YOLO(abs_path)
        self.confidence_threshold = confidence_threshold
        self.target_class_name = target_class_name
        self.target_class_id = target_class_id

        # Model'deki sınıf isimlerini logla
        print(f"[DETECTOR] Model yüklendi: {abs_path}")
        print(f"[DETECTOR] Sınıflar: {self.model.names}")
        print(f"[DETECTOR] Güven eşiği: {self.confidence_threshold}")

        # Hedef sınıf doğrulaması — model.names[target_id] == target_name
        if self.target_class_name is not None:
            actual_name = self.model.names.get(self.target_class_id, None)
            assert actual_name is not None and actual_name.lower() == self.target_class_name.lower(), \
                (f"[DETECTOR] HATA: model.names[{self.target_class_id}] = '{actual_name}', "
                 f"beklenen = '{self.target_class_name}'")
            print(f"[DETECTOR] Hedef sınıf filtresi: {self.target_class_name} ({self.target_class_id})")
        else:
            print("[DETECTOR] Hedef sınıf filtresi: YOK (tüm sınıflar kabul)")

    def detect(self, frame) -> Tuple[List[Detection], List[Detection]]:
        """
        Tek bir frame üzerinde YOLO inference çalıştırır.

        Args:
            frame: OpenCV BGR formatında numpy array (cv2.imread çıktısı)

        Returns:
            (targets, obstacles) tuple'ı:
              - targets: Hedef sınıf (class_id==target_class_id, conf>=threshold)
              - obstacles: Engel tespitleri (diğer sınıflar, conf>=threshold) — sadece log/grid için
        """
        if frame is None:
            print("[DETECTOR] UYARI: Boş frame, inference atlanıyor")
            return [], []

        results = self.model(frame, verbose=False)

        targets: List[Detection] = []
        obstacles: List[Detection] = []

        for result in results:
            boxes = result.boxes
            for box in boxes:
                class_id = int(box.cls[0])
                class_name = self.model.names[class_id]
                confidence = float(box.conf[0])

                # Güven eşiği filtresi
                if confidence < self.confidence_threshold:
                    continue

                # Bbox koordinatları (piksel)
                x1, y1, x2, y2 = box.xyxy[0].tolist()
                center_x = (x1 + x2) / 2.0
                center_y = (y1 + y2) / 2.0

                det = Detection(
                    class_name=class_name,
                    class_id=class_id,
                    confidence=confidence,
                    center_x=center_x,
                    center_y=center_y,
                    x1=x1, y1=y1, x2=x2, y2=y2,
                )

                # Hedef sınıf filtresi: sadece class_id == target_class_id → targets
                if self.target_class_name is not None:
                    if class_id == self.target_class_id:
                        targets.append(det)
                    else:
                        # Engel logla ama tetikleme yapma
                        print(f"[DETECTOR] Engel loglandı (tetikleme yok): "
                              f"sınıf={class_name}({class_id}), conf={confidence:.2f}, "
                              f"bbox=[{x1:.0f},{y1:.0f},{x2:.0f},{y2:.0f}]")
                        obstacles.append(det)
                else:
                    # Filtre yok → tüm tespitler target kabul edilir
                    targets.append(det)

        return targets, obstacles


def detect_from_file(image_path: str, model_path: str = "../best.pt",
                     confidence_threshold: float = 0.55,
                     target_class_name: str = "Hedef",
                     target_class_id: int = 0) -> Tuple[List[Detection], List[Detection]]:
    """
    Yardımcı fonksiyon: Tek bir görüntü dosyasında tespit çalıştırır.
    Test ve debug için kullanılır.

    Kullanım:
        python detector.py test_image.jpg
    """
    frame = cv2.imread(image_path)
    if frame is None:
        print(f"HATA: Görüntü okunamadı: {image_path}")
        return [], []

    detector = TargetDetector(
        model_path, confidence_threshold,
        target_class_name=target_class_name,
        target_class_id=target_class_id,
    )
    targets, obstacles = detector.detect(frame)

    print(f"\n--- Hedef tespitler ({len(targets)}) ---")
    if not targets:
        print("  Hiçbir hedef tespit edilmedi.")
    else:
        for i, d in enumerate(targets, 1):
            print(f"  [{i}] {d.class_name}(id={d.class_id}): {d.confidence:.1%} "
                  f"@ ({d.center_x:.0f}, {d.center_y:.0f}) "
                  f"bbox=[{d.x1:.0f},{d.y1:.0f},{d.x2:.0f},{d.y2:.0f}]")

    print(f"\n--- Engel tespitler ({len(obstacles)}) ---")
    if not obstacles:
        print("  Hiçbir engel tespit edilmedi.")
    else:
        for i, d in enumerate(obstacles, 1):
            print(f"  [{i}] {d.class_name}(id={d.class_id}): {d.confidence:.1%} "
                  f"@ ({d.center_x:.0f}, {d.center_y:.0f}) "
                  f"bbox=[{d.x1:.0f},{d.y1:.0f},{d.x2:.0f},{d.y2:.0f}]")

    return targets, obstacles


if __name__ == "__main__":
    # Bağımsız test: python detector.py <görüntü_dosyası> [model_yolu]
    if len(sys.argv) < 2:
        print("Kullanım: python detector.py <görüntü.jpg> [model.pt]")
        sys.exit(1)

    img_path = sys.argv[1]
    mdl_path = sys.argv[2] if len(sys.argv) > 2 else "../best.pt"
    detect_from_file(img_path, mdl_path)
