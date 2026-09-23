import os
import sys
import json
import glob
import cv2
import numpy as np

dataset_root = r"D:\Unity\Drone Arazisi\DroneDatasetOutput"
out_dir = r"c:\Users\Murat Ege\OneDrive\Masaüstü\DroneTest\QUADCOPTER_TEST\companion\dataset_analysis"
os.makedirs(out_dir, exist_ok=True)

report_file = os.path.join(out_dir, "dataset_report.txt")
log_f = open(report_file, "w", encoding="utf-8")

def log(msg):
    print(msg)
    log_f.write(msg + "\n")

log("=== UNITY DATASET VALIDATION REPORT ===")
log(f"Dataset Root: {dataset_root}")

runs = [d for d in os.listdir(dataset_root) if os.path.isdir(os.path.join(dataset_root, d)) and d.startswith("run_")]
runs.sort()

main_run = None
max_frames = 0
all_run_stats = {}

for run in runs:
    run_path = os.path.join(dataset_root, run)
    img_dir = os.path.join(run_path, "images", "train")
    sem_dir = os.path.join(run_path, "semantic")
    
    if not os.path.isdir(img_dir) or not os.path.isdir(sem_dir):
        log(f"\nRun: {run} - SKIPPED (Missing images/train or semantic)")
        continue
        
    imgs = [f for f in os.listdir(img_dir) if f.startswith("frame_") and f.endswith((".png", ".jpg"))]
    sems = [f for f in os.listdir(sem_dir) if f.startswith("frame_") and f.endswith(".png")]
    
    # Strip extensions to match pairs
    img_stems = set([os.path.splitext(f)[0] for f in imgs])
    sem_stems = set([os.path.splitext(f)[0] for f in sems])
    
    matched = img_stems.intersection(sem_stems)
    log(f"\nRun: {run}")
    log(f"  RGB Items: {len(imgs)}")
    log(f"  Semantic Items: {len(sems)}")
    log(f"  Exact Matches: {len(matched)}")
    
    all_run_stats[run] = {
        "rgb": len(imgs),
        "sem": len(sems),
        "matched": len(matched),
        "img_dir": img_dir,
        "sem_dir": sem_dir,
        "frames": sorted(list(matched))
    }
    
    if len(matched) > max_frames:
        max_frames = len(matched)
        main_run = run

    # Read manifest if exists
    manifest_path = os.path.join(run_path, "manifest.json")
    if os.path.exists(manifest_path):
        try:
            with open(manifest_path, "r", encoding="utf-8") as f:
                manifest = json.load(f)
                log(f"  Manifest Exists - Random seed / Layout indicator: {manifest.get('layout_seed', 'Unknown')}")
        except Exception as e:
            log(f"  Manifest Error: {e}")

log("\n=== MAIN RUN ANALYSIS ===")
if main_run is None:
    log("No valid runs found!")
    sys.exit(1)

log(f"Identified Main Run (Largest): {main_run} with {max_frames} frames.")
log("WARNING: Proceeding without merging smaller runs to preserve data split integrity.")

main_data = all_run_stats[main_run]
frames = main_data["frames"]

# Sample frames for collage and dimension verification
import random
# deterministic seed for reproducible review
random.seed(42)
sample_frames = random.sample(frames, min(12, len(frames)))

log("\nVerifying subset dimensions & exact class mapping (12 samples)...")
collage_pairs = []
class_totals = {0: 0, 1: 0, 2: 0, 3: 0}
pixels_sum = 0
dim_errors = []

for frame in sample_frames:
    rgb_path = os.path.join(main_data["img_dir"], frame + ".png") # Try png first
    if not os.path.exists(rgb_path):
        rgb_path = os.path.join(main_data["img_dir"], frame + ".jpg")
        
    sem_path = os.path.join(main_data["sem_dir"], frame + ".png")
    
    # Read strict Windows paths
    rb = np.fromfile(rgb_path, dtype=np.uint8)
    img = cv2.imdecode(rb, cv2.IMREAD_COLOR)
    
    mb = np.fromfile(sem_path, dtype=np.uint8)
    mask = cv2.imdecode(mb, cv2.IMREAD_UNCHANGED)
    
    h1, w1 = img.shape[:2]
    h2, w2 = mask.shape[:2]
    
    if (h1, w1) != (h2, w2):
        dim_errors.append(f"{frame}: RGB {w1}x{h1} != MASK {w2}x{h2}")
        continue
        
    if (h1, w1) != (1200, 1920):
        dim_errors.append(f"{frame}: Deviates from 1920x1200 -> It is {w1}x{h1}!")
        
    # Multi-channel checking without silent grayscale
    if len(mask.shape) == 3:
        b = mask[:,:,0]; g = mask[:,:,1]; r = mask[:,:,2]
        if not (np.array_equal(b, g) and np.array_equal(g, r)):
            log(f"  {frame} - MASK ERROR: RGB channels are not equal! Color leakage detected.")
            continue
        valid_mask = b
    else:
        valid_mask = mask
        
    # Collect Pixel Counts safely
    for c in [0, 1, 2, 3]:
        class_totals[c] += int(np.sum(valid_mask == c))
    pixels_sum += (h1 * w1)
    
    # Prepare overlay for collage
    overlay = img.copy()
    overlay[valid_mask == 1] = overlay[valid_mask == 1] * 0.5 + np.array([0, 255, 0]) * 0.5 # Green Path
    overlay[valid_mask == 3] = overlay[valid_mask == 3] * 0.5 + np.array([0, 0, 255]) * 0.5 # Red Target
    # Bounding obstacle boundaries
    obs_mask = (valid_mask == 2).astype(np.uint8) * 255
    contours, _ = cv2.findContours(obs_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cv2.drawContours(overlay, contours, -1, (255, 0, 0), 2)
    
    # resize for tiny thumbnail in 12-grid collage so it doesn't OOM
    thumb = cv2.resize(overlay, (480, 300))
    cv2.putText(thumb, frame, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,255,255), 2)
    collage_pairs.append(thumb)

log("\n--- Sample Dimension Report ---")
if dim_errors:
    for e in dim_errors:
        log(f"  WARN: {e}")
else:
    log("  All 12 samples match strictly 1920x1200. No deviations.")

log("\n--- Sample Semantic Distribution ---")
log(f"  Total pixels sampled: {pixels_sum}")
for k, v in class_totals.items():
    log(f"  Class {k}: {v} ({(v/pixels_sum*100):.2f}%)")

log("\n--- Diversity & Split Strategy Proposal ---")
log("Checking metadata to identify track layout differences (ignoring camera height/angles as track layouts).")
# Randomly peek at 50 frames' metadata if exists
meta_dir = os.path.join(dataset_root, main_run, "metadata")
track_layouts = set()

if os.path.isdir(meta_dir):
    meta_files = [f for f in os.listdir(meta_dir) if f.endswith(".json")]
    check_subset = random.sample(meta_files, min(len(meta_files), 100))
    for mf in check_subset:
        with open(os.path.join(meta_dir, mf), "r") as f:
            try:
                mdata = json.load(f)
                # Differentiate by generated track seed/ID
                track_id = mdata.get("track_seed", mdata.get("layout_id", "Unknown"))
                track_layouts.add(track_id)
            except:
                pass

if len(track_layouts) > 0:
    log(f"Discovered {len(track_layouts)} distinct track layouts in the sampled subset.")
    if len(track_layouts) == 1:
        log("CAUTION: Only 1 track layout pattern detected! Success on test splits cannot prove independent generalizability!")
        log("PROPOSAL: Since it's one track, you can only train/test on camera variations. Split frames chronologically or by camera height groupings to test view-angle invariance.")
    else:
        log("PROPOSAL: Group validation frames STRICTLY by 'track_seed' or 'layout_id'. Entire track variants must be held out to truly measure independent validation performance.")
else:
    log("WARN: No structural metadata (track_seed/layout_id) found. If dataset is sequential, split geographically or sequentially (e.g. last 10% of time-series) to avoid overlapping identical tracks.")

# Create College
if len(collage_pairs) > 0:
    rows = []
    # 3 cols x 4 rows
    for i in range(0, len(collage_pairs), 3):
        row = collage_pairs[i:i+3]
        if len(row) < 3: # pad with black
            for _ in range(3 - len(row)):
                row.append(np.zeros_like(collage_pairs[0]))
        rows.append(np.concatenate(row, axis=1))
    
    grid_img = np.concatenate(rows, axis=0)
    col_path = os.path.join(out_dir, "dataset_preview.png")
    cv2.imencode(".png", grid_img)[1].tofile(col_path)
    log(f"\nSaved 12-image preview collage to: {col_path}")

log_f.close()
