import argparse
import json
import os
from pathlib import Path
from PIL import Image, ImageDraw, ImageFont

import torch
import torchvision.transforms.functional as TF
from torchvision.models.segmentation import lraspp_mobilenet_v3_large
from ultralytics import YOLO
import numpy as np

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--imgsz", type=int, default=1280, help="YOLO image size")
    parser.add_argument("--conf", type=float, default=0.25, help="YOLO confidence threshold")
    args = parser.parse_args()

    # Paths (Absolute or relative to this script's directory)
    base_dir = os.path.dirname(os.path.abspath(__file__))
    model_dir = os.path.join(base_dir, "offline_models")
    output_dir = os.path.join(base_dir, "offline_results")
    os.makedirs(output_dir, exist_ok=True)

    yolo_path = os.path.join(model_dir, "best.pt")
    seg_path = os.path.join(model_dir, "best_road_seg_model.pth")
    img_path = os.path.join(model_dir, "deneme.png")

    device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
    print(f"Using device: {device}")

    # ==========================
    # 1. YOLO Detection
    # ==========================
    print(f"Loading YOLO from {yolo_path}")
    yolo_model = YOLO(yolo_path)
    print(f"YOLO task type: {yolo_model.task}")
    print(f"YOLO classes: {yolo_model.names}")

    img_pil = Image.open(img_path).convert("RGB")
    w, h = img_pil.size
    print(f"Original image size: {w}x{h}")

    # Run YOLO 
    yolo_results = yolo_model(img_pil, imgsz=args.imgsz, conf=args.conf)[0]
    
    detections = []
    for box in yolo_results.boxes:
        # Original coords
        x1, y1, x2, y2 = box.xyxy[0].tolist()
        cls_id = int(box.cls[0].item())
        conf = float(box.conf[0].item())
        cls_name = yolo_model.names[cls_id]
        detections.append({
            "class_id": cls_id,
            "class_name": cls_name,
            "confidence": conf,
            "bbox": [x1, y1, x2, y2]
        })

    # ==========================
    # 2. Road Segmentation
    # ==========================
    print(f"Loading Semantic Segmentation from {seg_path}")
    seg_model = lraspp_mobilenet_v3_large(weights=None, weights_backbone=None)
    low_channels = seg_model.classifier.low_classifier.in_channels
    high_channels = seg_model.classifier.high_classifier.in_channels
    seg_model.classifier.low_classifier = torch.nn.Conv2d(low_channels, 1, 1)
    seg_model.classifier.high_classifier = torch.nn.Conv2d(high_channels, 1, 1)

    checkpoint = torch.load(seg_path, map_location='cpu', weights_only=True)
    state_dict = checkpoint.get("model_state_dict", checkpoint)
    seg_model.load_state_dict(state_dict, strict=True)
    seg_model.to(device)
    seg_model.eval()

    # Preprocessing
    img_seg = TF.resize(img_pil, [600, 960], interpolation=TF.InterpolationMode.BILINEAR)
    tensor_seg = TF.to_tensor(img_seg)
    tensor_seg = TF.normalize(tensor_seg, mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225])
    tensor_seg = tensor_seg.unsqueeze(0).to(device)

    # Inference
    print("Running Segmentation inference...")
    with torch.inference_mode():
        seg_out = seg_model(tensor_seg)['out']  # [1, 1, 600, 960]
        seg_mask = torch.sigmoid(seg_out) > 0.5
        
        # Resize mask to original dims
        seg_mask_resized = TF.resize(seg_mask, [h, w], interpolation=TF.InterpolationMode.NEAREST)
        # Convert to numpy 0/255
        seg_mask_np = seg_mask_resized.squeeze().cpu().numpy().astype(np.uint8) * 255

    # ==========================
    # 3. Generating Outputs
    # ==========================
    # detection_overlay.png
    det_overlay = img_pil.copy()
    draw = ImageDraw.Draw(det_overlay)
    for det in detections:
        x1, y1, x2, y2 = det['bbox']
        label = f"{det['class_name']} {det['confidence']:.2f}"
        draw.rectangle([x1, y1, x2, y2], outline="red", width=3)
        draw.text((x1, y1), label, fill="red")
    
    det_overlay_path = os.path.join(output_dir, "detection_overlay.png")
    det_overlay.save(det_overlay_path)

    # road_mask.png
    road_mask_pil = Image.fromarray(seg_mask_np, mode="L")
    road_mask_path = os.path.join(output_dir, "road_mask.png")
    road_mask_pil.save(road_mask_path)

    # road_overlay.png
    mask_rgba = Image.fromarray(seg_mask_np, mode="L")
    colored_mask = Image.new("RGBA", img_pil.size, (0, 255, 0, 100))
    road_overlay = img_pil.copy().convert("RGBA")
    road_overlay = Image.composite(colored_mask, road_overlay, mask_rgba)
    road_overlay_path = os.path.join(output_dir, "road_overlay.png")
    road_overlay.convert("RGB").save(road_overlay_path)

    # combined_overlay.png
    combined_overlay = road_overlay.copy()
    draw_comb = ImageDraw.Draw(combined_overlay)
    for det in detections:
        x1, y1, x2, y2 = det['bbox']
        label = f"{det['class_name']} {det['confidence']:.2f}"
        draw_comb.rectangle([x1, y1, x2, y2], outline="red", width=3)
        draw_comb.text((x1, y1), label, fill="red")
    
    combined_path = os.path.join(output_dir, "combined_overlay.png")
    combined_overlay.convert("RGB").save(combined_path)

    # results.json
    results_dict = {
        "models": {
            "yolo_path": yolo_path,
            "seg_path": seg_path
        },
        "settings": {
            "imgsz": args.imgsz,
            "conf": args.conf,
            "device": str(device)
        },
        "image": {
            "path": img_path,
            "original_width": w,
            "original_height": h
        },
        "yolo_classes": yolo_model.names,
        "detections": detections,
        "segmentation_target_size": [600, 960],
        "segmentation_normalization": {
            "mean": [0.485, 0.456, 0.406],
            "std": [0.229, 0.224, 0.225]
        },
        "segmentation_threshold": 0.5
    }

    results_path = os.path.join(output_dir, "results.json")
    with open(results_path, "w", encoding="utf-8") as f:
        json.dump(results_dict, f, indent=4, ensure_ascii=False)

    print(f"Results saved to {output_dir}")
    print(f"YOLO boxes found: {len(detections)}")
    for cl in set([d['class_name'] for d in detections]):
        count = sum([1 for d in detections if d['class_name'] == cl])
        print(f" - {cl}: {count}")

if __name__ == '__main__':
    main()
