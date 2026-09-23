import argparse
import sys
import os
import cv2
import numpy as np
import json
import logging

# Set up local import without modifying system paths structurally
sys.path.append(os.path.dirname(os.path.abspath(__file__)))
from grid_path_planner import GridPlanner

# Classes
UNKNOWN_CLASS = 0
FREE_CLASS = 1
OBSTACLE_CLASS = 2
TARGET_CLASS = 3

def extract_mask_layer(mask_img):
    """
    Unity masks might be saved as RGB. If so, channels will be equal or only one used.
    Returns 2D grid of classes (0, 1, 2, 3).
    """
    if len(mask_img.shape) == 3:
        b = mask_img[:, :, 0]
        g = mask_img[:, :, 1]
        r = mask_img[:, :, 2]
        # Check if R==G==B
        if not (np.array_equal(b, g) and np.array_equal(g, r)):
            logging.error("Mask channels are not equal. Unrecognized Unity encoding.")
            sys.exit(1)
        grid = r.copy()
    else:
        grid = mask_img.copy()
        
    unique_vals = np.unique(grid)
    for val in unique_vals:
        if val not in [0, 1, 2, 3]:
            logging.error(f"Invalid mask value found: {val}. Allowed: 0,1,2,3.")
            sys.exit(1)
            
    return grid

def find_target_point(grid):
    """
    Find target from TARGET_CLASS. Should be exactly 1 connected component.
    If the geometric centroid falls outside the target mask (e.g. concave 'H'),
    it picks the target pixel closest to that centroid.
    """
    target_pixels_mask = (grid == TARGET_CLASS).astype(np.uint8)
    if np.sum(target_pixels_mask) == 0:
        return None
    
    num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(target_pixels_mask, connectivity=8)
    # num_labels includes background (0). 
    # If there is exactly 1 target component, num_labels will be 2.
    if num_labels > 2:
        logging.error("Multiple disconnected target components found! Refusing to arbitrarily pick one.")
        sys.exit(1)
        
    # The centroid of the single component
    cx, cy = centroids[1]
    cx, cy = int(round(cx)), int(round(cy))
    
    if grid[cy, cx] == TARGET_CLASS:
        return (cx, cy)
        
    # The centroid is outside the mask. Find nearest pixel that IS on the target mask.
    target_coords = np.argwhere(grid == TARGET_CLASS) # (y, x)
    dist = np.sum((target_coords - np.array([cy, cx]))**2, axis=1)
    best_idx = np.argmin(dist)
    best_y, best_x = target_coords[best_idx]
    
    return (int(best_x), int(best_y))

def validate_coordinates(grid, pt, strict_free=False):
    h, w = grid.shape
    x, y = pt
    if not (0 <= x < w and 0 <= y < h):
        return False
    # Start and Goal must not be strictly obstacle
    if grid[y, x] == OBSTACLE_CLASS:
        return False
    if strict_free and grid[y, x] == UNKNOWN_CLASS:
        return False
    return True

def apply_connection_mask(grid, connect_mask_img):
    """
    Apply a connection mask.
    Only explicit 255 pixels are considered.
    They must only convert UNKNOWN to FREE.
    If they touch OBSTACLE, reject.
    Returns the modified grid, boolean mask of added connections, and number of converted pixels.
    """
    if len(connect_mask_img.shape) == 3:
        conn = connect_mask_img[:, :, 0].copy()
    else:
        conn = connect_mask_img.copy()
        
    unique_vals = np.unique(conn)
    for val in unique_vals:
        if val not in [0, 255]:
            logging.error(f"Connection mask has invalid values {val}. Only 0 and 255 allowed.")
            sys.exit(1)
            
    conn_pixels = (conn == 255)
    
    # Check if touching OBSTACLE
    if np.any((grid == OBSTACLE_CLASS) & conn_pixels):
        logging.error("Connection mask overlaps with OBSTACLE. Connection rejected.")
        sys.exit(1)
        
    added = np.zeros_like(grid, dtype=bool)
    converted_count = 0
    h, w = grid.shape
    for y in range(h):
        for x in range(w):
            if conn_pixels[y, x]:
                if grid[y, x] == UNKNOWN_CLASS:
                    grid[y, x] = FREE_CLASS
                    added[y, x] = True
                    converted_count += 1
    return grid, added, converted_count

def apply_heuristic_gap_repair(grid, n_px, roi=None):
    if n_px <= 0:
        return grid, np.zeros_like(grid, dtype=bool), {"accepted": 0, "rejected_reasons": []}
        
    path_bin = (grid == FREE_CLASS).astype(np.uint8)
    
    # 1. Fixed kernel extraction (guarantees candidate blobs are identical across N)
    fixed_kernel = np.ones((7, 7), np.uint8)
    closed = cv2.morphologyEx(path_bin, cv2.MORPH_CLOSE, fixed_kernel)
    
    candidates = (closed == 1) & (path_bin == 0)
    num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(candidates.astype(np.uint8), connectivity=8)
    
    # Distance from path to measure true gap width inside blobs
    dist_to_path = cv2.distanceTransform(1 - path_bin, cv2.DIST_L1, 3)
    
    added_connections = np.zeros_like(grid, dtype=bool)
    accepted_count = 0
    rejected_reasons = []
    original_grid = grid.copy()
    
    for i in range(1, num_labels):
        blob_mask = (labels == i)
        
        # 1. ROI Check
        if roi:
            x0, y0, x1, y1 = roi
            coords = np.argwhere(blob_mask) # (y, x)
            outside = [c for c in coords if not (y0 <= c[0] <= y1 and x0 <= c[1] <= x1)]
            if outside:
                rejected_reasons.append(f"Blob {i}: Out of ROI bounds")
                continue
                
        # 2. Only change pure UNKNOWN
        if np.any(original_grid[blob_mask] != UNKNOWN_CLASS):
            rejected_reasons.append(f"Blob {i}: Overlaps non-UNKNOWN")
            continue
            
        # 3. No Obstacle/Target Touching
        dilated_blob = cv2.dilate(blob_mask.astype(np.uint8), np.ones((3,3), np.uint8))
        if np.any(original_grid[dilated_blob == 1] == OBSTACLE_CLASS):
            rejected_reasons.append(f"Blob {i}: Touches OBSTACLE")
            continue
        if np.any(original_grid[dilated_blob == 1] == TARGET_CLASS):
            rejected_reasons.append(f"Blob {i}: Touches TARGET")
            continue
            
        # 4. Measure Gap Width and respect N
        max_dist = np.max(dist_to_path[blob_mask])
        if max_dist > (n_px + 1) / 2.0:
            rejected_reasons.append(f"Blob {i}: Max distance ({max_dist}) exceeds equivalent N={n_px}")
            continue
            
        accepted_count += 1
        added_connections[blob_mask] = True
        grid[blob_mask] = FREE_CLASS
        
    return grid, added_connections, {"accepted": accepted_count, "accepted_pixels": int(np.sum(added_connections)), "rejected_reasons": rejected_reasons[:20]}

def main():
    parser = argparse.ArgumentParser(description="Offline route planner using GridPlanner from semantic masks.")
    parser.add_argument("--image", required=True, help="RGB PNG original image")
    parser.add_argument("--mask", required=True, help="Semantic mask PNG (same dimensions as RGB)")
    parser.add_argument("--start", nargs=2, type=int, required=True, metavar=('X', 'Y'), help="Manual start coordinates")
    parser.add_argument("--goal", nargs=2, type=int, metavar=('X', 'Y'), help="Manual goal coordinates (optional, inferred from mask if absent)")
    parser.add_argument("--connect_mask", help="Optional mask to forge connections over UNKNOWN areas")
    parser.add_argument("--hard-clearance", type=int, default=5, help="Strict obstacle inflation radius in px")
    parser.add_argument("--repair-road-gaps-px", type=int, default=0, help="Heuristically repair small UNKNOWN gaps up to N px wide")
    parser.add_argument("--repair-roi", nargs=4, type=int, metavar=('X0', 'Y0', 'X1', 'Y1'), help="Restrict gap repair to strict ROI")
    parser.add_argument("--alpha", type=float, default=0.0, help="Weight for centering penalty array")
    parser.add_argument("--tau", type=float, default=10.0, help="Exponential decay constant for centering")
    parser.add_argument("--unknown-penalty", type=float, default=8.0, help="Step cost multiplier for crossing UNKNOWN cells")
    parser.add_argument("--output-dir", required=True, help="Directory to save outputs")
    
    args = parser.parse_args()
    
    logging.basicConfig(level=logging.INFO, format="%(levelname)s: %(message)s")
    
    if not os.path.isdir(args.output_dir):
        os.makedirs(args.output_dir, exist_ok=True)
        
    rgb_buffer = np.fromfile(args.image, dtype=np.uint8)
    if rgb_buffer.size == 0:
        logging.error(f"Cannot read rgb image (path broken/empty): {args.image}")
        sys.exit(1)
    rgb_img = cv2.imdecode(rgb_buffer, cv2.IMREAD_COLOR)
    if rgb_img is None:
        logging.error(f"Failed to decode RGB image: {args.image}")
        sys.exit(1)

    mask_buffer = np.fromfile(args.mask, dtype=np.uint8)
    if mask_buffer.size == 0:
        logging.error(f"Cannot read mask image (path broken/empty): {args.mask}")
        sys.exit(1)
    mask_img = cv2.imdecode(mask_buffer, cv2.IMREAD_UNCHANGED)
    if mask_img is None:
        logging.error(f"Failed to decode Mask image: {args.mask}")
        sys.exit(1)
        
    if rgb_img.shape[:2] != mask_img.shape[:2]:
        logging.error("RGB and Mask dimension mismatch!")
        sys.exit(1)
        
    logging.info("RGB and Mask dimensions match. (Note: Same dimension doesn't rigidly prove they are the exact same frame).")
    
    grid = extract_mask_layer(mask_img)
    h, w = grid.shape
    
    # Evaluate start
    start_pt = tuple(args.start)
    if not validate_coordinates(grid, start_pt, strict_free=True):
        logging.error(f"Start coordinates {start_pt} are invalid or strictly blocked!")
        sys.exit(1)
        
    # Evaluate connection
    manual_assumption = False
    added_connections = np.zeros_like(grid, dtype=bool)
    converted_count = 0
    if args.connect_mask:
        c_buffer = np.fromfile(args.connect_mask, dtype=np.uint8)
        if c_buffer.size == 0:
            logging.error(f"Cannot read connection mask (path broken/empty): {args.connect_mask}")
            sys.exit(1)
        c_img = cv2.imdecode(c_buffer, cv2.IMREAD_UNCHANGED)
        if c_img is None:
            logging.error(f"Failed to decode connection mask: {args.connect_mask}")
            sys.exit(1)
        if c_img.shape[:2] != (h, w):
            logging.error("Connection mask dimension mismatch!")
            sys.exit(1)
        grid, added_connections, converted_count = apply_connection_mask(grid, c_img)
        manual_assumption = True
        logging.info(f"Connection mask applied successfully. Converted {converted_count} UNKNOWN pixels to FREE.")
        
    added_repairs = np.zeros_like(grid, dtype=bool)
    repair_details = {}
    if args.repair_road_gaps_px > 0:
        grid, added_repairs, repair_details = apply_heuristic_gap_repair(grid, args.repair_road_gaps_px, args.repair_roi)
        logging.info(f"Heuristic repair applied. Accepted {repair_details['accepted']} candidate blobs.")
        
    # Determine Goal
    if args.goal:
        goal_pt = tuple(args.goal)
        h, w = grid.shape
        if not (0 <= goal_pt[1] < h and 0 <= goal_pt[0] < w):
            logging.error(f"Manual goal {goal_pt} is out of bounds (w={w}, h={h}).")
            sys.exit(1)
        # Check if manual goal is in target mask
        if grid[goal_pt[1], goal_pt[0]] != TARGET_CLASS:
            logging.warning(f"Manual goal {goal_pt} is NOT within the TARGET_CLASS mask.")
    else:
        goal_pt = find_target_point(grid)
        if goal_pt is None:
            logging.error("No Goal provided and no TARGET_CLASS found in mask!")
            sys.exit(1)
            
    if not validate_coordinates(grid, goal_pt, strict_free=True):
        logging.error(f"Goal coordinates {goal_pt} are invalid or strictly blocked!")
        sys.exit(1)
        
    # To treat Target as free space for routing:
    planning_grid = grid.copy()
    planning_grid[planning_grid == TARGET_CLASS] = FREE_CLASS
    
    # Planner Call
    # Pixel scale = 1.0, Vehicle Width = 0
    planner = GridPlanner(cell_size=1.0, vehicle_width=0.0, safety_margin=args.hard_clearance, unknown_penalty=args.unknown_penalty)
    
    # Planner translation: 0=FREE, 1=OBSTACLE, 2=UNKNOWN (Inflation treats UNKNOWN as obstacle)
    # Our grid has 1=FREE, 2=OBSTACLE, 0=UNKNOWN. Need to map carefully!
    # GridPlanner expects FREE=0, OBSTACLE=1, UNKNOWN=2
    mapped_grid = np.zeros_like(planning_grid, dtype=np.uint8)
    mapped_grid[planning_grid == FREE_CLASS] = GridPlanner.FREE
    mapped_grid[planning_grid == OBSTACLE_CLASS] = GridPlanner.OBSTACLE
    mapped_grid[planning_grid == UNKNOWN_CLASS] = GridPlanner.UNKNOWN
    
    dist_map = None
    if args.alpha > 0.0:
        dangerous = ((mapped_grid == GridPlanner.OBSTACLE) | (mapped_grid == GridPlanner.UNKNOWN)).astype(np.uint8)
        dist_map = cv2.distanceTransform(1 - dangerous, cv2.DIST_L2, 5)
    
    json_resp, path_grid_pts, inflated_work_grid = planner.plan_path(
        mapped_grid, start_pt, goal_pt,
        dist_map=dist_map, alpha=args.alpha, tau=args.tau
    )
    
    # Analyze UNKNOWN traversal metrics
    unknowns_crossed = 0
    min_obs_dist = float('inf')
    avg_obs_dist_free = 0.0
    free_crossed = 0
    narrow_passage_pixels = 0
    
    if path_grid_pts:
        obs_mask = (mapped_grid == GridPlanner.OBSTACLE).astype(np.uint8)
        obs_dist_map = cv2.distanceTransform(1 - obs_mask, cv2.DIST_L2, 5)
        for pt in path_grid_pts:
            x, y = pt
            dist = obs_dist_map[y, x]
            
            if dist < 10.0:
                narrow_passage_pixels += 1
                
            if mapped_grid[y, x] == GridPlanner.UNKNOWN:
                unknowns_crossed += 1
                if dist < min_obs_dist:
                    min_obs_dist = dist
            elif mapped_grid[y, x] == GridPlanner.FREE:
                free_crossed += 1
                avg_obs_dist_free += dist
                if dist < min_obs_dist:
                    min_obs_dist = dist
                
        if free_crossed > 0:
            avg_obs_dist_free /= free_crossed
                    
        if unknowns_crossed > 0:
            logging.info(f"Crossed {unknowns_crossed} UNKNOWN pixels.")
        logging.info(f"Global min distance to OBSTACLE: {min_obs_dist:.2f}px")
        if free_crossed > 0:
            logging.info(f"FREE segment mean distance to rigid OBSTACLES: {avg_obs_dist_free:.2f}px")
        logging.info(f"Narrow passage traversal (<10px clearance): {narrow_passage_pixels} pixels.")
    
    # Generate Output Image
    out_img = rgb_img.copy()
    
    # Visualize Inflated Mask in semi-transparent Red
    inflated_territory = (inflated_work_grid == GridPlanner.OBSTACLE) & (mapped_grid == GridPlanner.FREE)
    out_img[inflated_territory] = out_img[inflated_territory] * 0.5 + np.array([0, 0, 255]) * 0.5
    
    # Draw connections
    if manual_assumption:
        out_img[added_connections] = [255, 0, 255] # Magenta
        cv2.putText(out_img, "MANUEL BAGLANTI - NOKTASAL TEST", (50, 100), cv2.FONT_HERSHEY_SIMPLEX, 1.5, (255, 0, 255), 4, cv2.LINE_AA)
        
    if args.repair_road_gaps_px > 0:
        out_img[added_repairs] = [255, 0, 255] # Purple for heuristic repairs
        cv2.putText(out_img, "PIKSEL TABANLI MASKE ONARIMI - FIZIKSEL GECIS DOGRULANMADI", (50, 220), cv2.FONT_HERSHEY_SIMPLEX, 1.2, (255, 0, 255), 3, cv2.LINE_AA)
        
    status = json_resp.get("status")
    
    # Print clearance text
    diag = json_resp.get("diagnostics", {})
    conf_c = diag.get("configured_clearance", int(args.hard_clearance))
    app_c = diag.get("applied_inflation", int(args.hard_clearance))
    b_val = diag.get("computed_bottleneck_B", 0.0)
    
    title_text = f"CONF CLEARANCE: {conf_c} PX | APPLIED: {app_c} PX (B={b_val}) | STATUS: {status}"
    cv2.putText(out_img, title_text, (50, 160), cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 255, 255), 4, cv2.LINE_AA)
    
    waypoints = json_resp.get("waypoints", [])
    
    if status == "SUCCESS" and path_grid_pts:
        path_grid_pts = np.array(path_grid_pts)
        cv2.polylines(out_img, [path_grid_pts.reshape(-1, 1, 2)], isClosed=False, color=(0, 255, 0), thickness=2)
        
    # Draw Start / Goal
    cv2.circle(out_img, start_pt, 5, (255, 0, 0), -1) # Blue Start
    cv2.circle(out_img, goal_pt, 5, (0, 0, 255), -1) # Red Goal
    
    out_img_path = os.path.join(args.output_dir, "route_output.png")
    ok, encoded = cv2.imencode(".png", out_img)
    if not ok:
        logging.error("Failed to encode output image to PNG format.")
        sys.exit(1)
        
    try:
        encoded.tofile(out_img_path)
    except Exception as e:
        logging.error(f"Failed to save output image: {e}")
        sys.exit(1)
        
    # Verify PNG write
    if not os.path.exists(out_img_path) or os.path.getsize(out_img_path) == 0:
        logging.error("Output PNG does not exist or is 0 bytes.")
        sys.exit(1)
        
    verify_buf = np.fromfile(out_img_path, dtype=np.uint8)
    verify_img = cv2.imdecode(verify_buf, cv2.IMREAD_COLOR)
    if verify_img is None:
        logging.error("Output PNG was saved but could not be decoded. File might be corrupt.")
        sys.exit(1)
        
    img_size = os.path.getsize(out_img_path)
    logging.info(f"Verified PNG output. Path: {out_img_path}, Size: {img_size} bytes")
    
    # Generate Output JSON
    output_dict = {
        "status": status,
        "coordinate_frame": "pixel",
        "start_source": "manual",
        "mask_source": "unity_reference",
        "manual_assumption_used": True if manual_assumption else False,
        "converted_unknown_pixels": converted_count,
        "start": {"x": start_pt[0], "y": start_pt[1]},
        "goal": {"x": goal_pt[0], "y": goal_pt[1]},
        "waypoints": waypoints,
        "message": "NO_PATH generated" if status != "SUCCESS" else "Route planned successfully",
        "unknowns_crossed": unknowns_crossed if 'unknowns_crossed' in locals() else 0,
        "global_min_dist_to_obstacle": float(min_obs_dist) if ('min_obs_dist' in locals() and min_obs_dist != float('inf')) else 0.0,
        "avg_obs_dist_free": float(avg_obs_dist_free) if 'avg_obs_dist_free' in locals() else 0.0,
        "narrow_passage_pixels": narrow_passage_pixels if 'narrow_passage_pixels' in locals() else 0,
        "warning": "This result does not validate physical vehicle fit."
    }
    
    if args.repair_road_gaps_px > 0:
        output_dict["mask_repair_used"] = True
        output_dict["repair_source"] = "heuristic"
        output_dict["repair_details"] = repair_details
    
    out_json_path = os.path.join(args.output_dir, "route_output.json")
    with open(out_json_path, "w") as f:
        json.dump(output_dict, f, indent=4)
        
    logging.info(f"Done. Status: {status}. Output saved to {args.output_dir}")

if __name__ == "__main__":
    main()
