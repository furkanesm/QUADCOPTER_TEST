import argparse
import cv2
import numpy as np
import json
import os
import math
from ultralytics import YOLO

def heuristic(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])

def astar(grid, start, goal):
    if grid[start[1], start[0]] == 1 or grid[goal[1], goal[0]] == 1:
        return None

    open_set = {start}
    came_from = {}
    g_score = {start: 0}
    f_score = {start: heuristic(start, goal)}
    
    directions = [
        (0, -1, 1.0), (0, 1, 1.0), (-1, 0, 1.0), (1, 0, 1.0),
        (-1, -1, 1.414), (1, -1, 1.414), (-1, 1, 1.414), (1, 1, 1.414)
    ]
    h, w = grid.shape
    
    while open_set:
        current = min(open_set, key=lambda x: f_score.get(x, float('inf')))
        if current == goal:
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.append(start)
            path.reverse()
            return path
            
        open_set.remove(current)
        for dx, dy, cost in directions:
            neighbor = (current[0] + dx, current[1] + dy)
            if not (0 <= neighbor[0] < w and 0 <= neighbor[1] < h):
                continue
            if grid[neighbor[1], neighbor[0]] == 1:
                continue
            if dx != 0 and dy != 0:
                if grid[current[1], current[0] + dx] == 1 or grid[current[1] + dy, current[0]] == 1:
                    continue
            
            tentative_g_score = g_score.get(current, float('inf')) + cost
            if tentative_g_score < g_score.get(neighbor, float('inf')):
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g_score
                f_score[neighbor] = tentative_g_score + heuristic(neighbor, goal)
                if neighbor not in open_set:
                    open_set.add(neighbor)
    return None

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--image", required=True)
    parser.add_argument("--model", type=str, default="best.pt")
    
    parser.add_argument("--pixel-mode", action="store_true", help="Metre donusumu/homography devre disi")
    parser.add_argument("--pixel-safety-margin", type=int, default=20, help="Guvenlik payi piksel")
    parser.add_argument("--manual-obstacles", type=str, default="", help="x1,y1,x2,y2;...")
    
    parser.add_argument("--roi", type=str, default="", help="x1,y1,x2,y2,x3,y3,x4,y4 (4 kose) sadece metre modunda gerekli")
    parser.add_argument("--arena-size", type=str, default="20,20")
    parser.add_argument("--grid-res", type=float, default=0.2)
    parser.add_argument("--vehicle-width", type=float, default=0.6)
    parser.add_argument("--safety-margin", type=float, default=0.2)
    
    parser.add_argument("--class-start", type=str, default="start")
    parser.add_argument("--class-goal", type=str, default="hedef")
    parser.add_argument("--class-obstacle", type=str, default="engel")
    
    parser.add_argument("--start-pixel", type=str, default="")
    parser.add_argument("--output-dir", type=str, default=".")
    
    args = parser.parse_args()
    
    # 0. Load Image
    img = cv2.imread(args.image)
    if img is None:
        print("HATA: Image okunamadi.")
        return
    img_h, img_w, _ = img.shape
    
    PIX_MODE = args.pixel_mode
    GRID_DS_FACTOR = 4 if PIX_MODE else 1 # downsample to make A* run faster in 640x640 images
    grid_w = int(img_w / GRID_DS_FACTOR) if PIX_MODE else 0
    grid_h = int(img_h / GRID_DS_FACTOR) if PIX_MODE else 0

    inflation_cells = 0
    H = None

    if not PIX_MODE:
        roi_pts_flat = [float(x) for x in args.roi.split(',')]
        src_pts = np.array([
            [roi_pts_flat[0], roi_pts_flat[1]], [roi_pts_flat[2], roi_pts_flat[3]],
            [roi_pts_flat[4], roi_pts_flat[5]], [roi_pts_flat[6], roi_pts_flat[7]]
        ], dtype=np.float32)
        arena_w, arena_h = [float(x) for x in args.arena_size.split(',')]
        dst_pts = np.array([ [0, 0], [arena_w, 0], [arena_w, arena_h], [0, arena_h] ], dtype=np.float32)
        H = cv2.getPerspectiveTransform(src_pts, dst_pts)
        
        grid_w = int(math.ceil(arena_w / args.grid_res))
        grid_h = int(math.ceil(arena_h / args.grid_res))
        
        inflation_radius = (args.vehicle_width / 2.0) + args.safety_margin
        inflation_cells = int(math.ceil(inflation_radius / args.grid_res))
        
        for i, pt in enumerate(src_pts):
            cv2.circle(img, (int(pt[0]), int(pt[1])), 5, (255, 0, 0), -1)
            cv2.putText(img, str(i+1), (int(pt[0])+10, int(pt[1])+10), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 0, 0), 3)
        cv2.polylines(img, [src_pts.astype(np.int32)], True, (255, 0, 0), 2)
    else:
        inflation_cells = int(math.ceil(args.pixel_safety_margin / float(GRID_DS_FACTOR)))

    grid = np.zeros((grid_h, grid_w), dtype=np.uint8)
    
    # 1. Model inference
    model = YOLO(args.model)
    print(f"Model ID eşlemeleri (model.names): {model.names}")
    results = model.predict(source=img, conf=0.1, verbose=False)
    
    start_grid = None
    goal_grid = None
    start_source = "AUTO"
    obstacle_sources = set()
    
    # Manual start parsing
    if args.start_pixel:
        sx, sy = [int(v) for v in args.start_pixel.split(',')]
        cv2.circle(img, (sx, sy), 8, (0, 255, 255), -1)
        cv2.putText(img, "MANUAL START", (sx+10, sy-10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        start_source = "MANUAL"
        
        if PIX_MODE:
            start_grid = (int(sx/GRID_DS_FACTOR), int(sy/GRID_DS_FACTOR))
        else:
            pt_m = cv2.perspectiveTransform(np.array([[[float(sx), float(sy)]]], dtype=np.float32), H)
            start_grid = (int(pt_m[0,0,0] / args.grid_res), int(pt_m[0,0,1] / args.grid_res))
    
    classes = model.names
    print("----- TESPiTLER -----")
    for r in results:
        for box in r.boxes:
            cls_name = classes[int(box.cls[0])]
            conf = float(box.conf[0])
            x1, y1, x2, y2 = map(float, box.xyxy[0])
            print(f"Sinif: {cls_name}, ID: {int(box.cls[0])}, Conf: {conf:.2f}, Box: ({int(x1)},{int(y1)}) - ({int(x2)},{int(y2)})")
            
            if args.class_obstacle.lower() in cls_name.lower():
                cv2.rectangle(img, (int(x1), int(y1)), (int(x2), int(y2)), (0, 0, 255), 2)
                cv2.putText(img, f"{cls_name} {conf:.2f}", (int(x1), int(y1)-5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
                obstacle_sources.add("AUTO")
                
                if PIX_MODE:
                    gx1 = max(0, int(x1/GRID_DS_FACTOR))
                    gy1 = max(0, int(y1/GRID_DS_FACTOR))
                    gx2 = min(grid_w-1, int(x2/GRID_DS_FACTOR))
                    gy2 = min(grid_h-1, int(y2/GRID_DS_FACTOR))
                    grid[gy1:gy2, gx1:gx2] = 1
                else:
                    bbox_pts = np.array([ [[x1, y1]], [[x2, y1]], [[x2, y2]], [[x1, y2]] ], dtype=np.float32)
                    bbox_m = cv2.perspectiveTransform(bbox_pts, H)
                    bbox_cells = (bbox_m / args.grid_res).astype(np.int32)
                    cv2.fillPoly(grid, [bbox_cells.reshape((-1, 1, 2))], 1)
                
            elif args.class_start.lower() in cls_name.lower() and start_source == "AUTO":
                cx, cy = (x1+x2)/2.0, (y1+y2)/2.0
                bottom_cx, bottom_cy = cx, y2
                cv2.rectangle(img, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
                cv2.putText(img, f"Start({cls_name})", (int(x1), int(y1)-5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                
                if PIX_MODE:
                    start_grid = (int(bottom_cx/GRID_DS_FACTOR), int(bottom_cy/GRID_DS_FACTOR))
                else:
                    pt_m = cv2.perspectiveTransform(np.array([[[bottom_cx, bottom_cy]]], dtype=np.float32), H)
                    start_grid = (int(pt_m[0,0,0] / args.grid_res), int(pt_m[0,0,1] / args.grid_res))
                
            elif args.class_goal.lower() in cls_name.lower():
                cx, cy = (x1+x2)/2.0, (y1+y2)/2.0
                bottom_cx, bottom_cy = cx, y2
                cv2.rectangle(img, (int(x1), int(y1)), (int(x2), int(y2)), (255, 0, 0), 2)
                cv2.putText(img, f"Goal({cls_name})", (int(x1), int(y1)-5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
                
                if PIX_MODE:
                    goal_grid = (int(bottom_cx/GRID_DS_FACTOR), int(bottom_cy/GRID_DS_FACTOR))
                else:
                    pt_m = cv2.perspectiveTransform(np.array([[[bottom_cx, bottom_cy]]], dtype=np.float32), H)
                    goal_grid = (int(pt_m[0,0,0] / args.grid_res), int(pt_m[0,0,1] / args.grid_res))
    print("---------------------")

    # Manual barriers injection
    if args.manual_obstacles:
        obstacle_sources.add("MANUAL")
        for box in args.manual_obstacles.split(';'):
            if not box: continue
            bx1,by1,bx2,by2 = [int(v) for v in box.split(',')]
            cv2.rectangle(img, (bx1, by1), (bx2, by2), (0, 165, 255), 2)
            cv2.putText(img, "MANUAL_OBS", (bx1, by1-5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 165, 255), 2)
            
            gx1 = max(0, int(bx1/GRID_DS_FACTOR))
            gy1 = max(0, int(by1/GRID_DS_FACTOR))
            gx2 = min(grid_w-1, int(bx2/GRID_DS_FACTOR))
            gy2 = min(grid_h-1, int(by2/GRID_DS_FACTOR))
            grid[gy1:gy2, gx1:gx2] = 1

    # Apply global inflation
    if inflation_cells > 0:
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (inflation_cells*2 + 1, inflation_cells*2 + 1))
        # inflate grid borders automatically if NOT pixel mode
        if not PIX_MODE:
            cv2.rectangle(grid, (0,0), (grid_w-1, grid_h-1), 1, thickness=inflation_cells*2)
        grid = cv2.dilate(grid, kernel, iterations=1)
        
        # in pixel mode, boundaries are also impassable and should affect inflation
        if PIX_MODE:
            bl = np.zeros((grid_h, grid_w), dtype=np.uint8)
            cv2.rectangle(bl, (0,0), (grid_w-1, grid_h-1), 1, thickness=inflation_cells*2)
            grid = cv2.bitwise_or(grid, bl)
    
    start_str = "MANUAL" if start_source == "MANUAL" else "AUTO"
    obs_str = ",".join(list(obstacle_sources)) if obstacle_sources else "NONE"
    
    response = {
        "coordinate_frame": "image_pixel" if PIX_MODE else "arena_local",
        "origin_corner": "Top-Left" if PIX_MODE else 1, 
        "unit": "pixels" if PIX_MODE else "meters", 
        "x_dir": "right", 
        "y_dir": "down",
        "status": "SUCCESS",
        "start_source": start_str,
        "obstacle_sources": obs_str,
        "waypoints": [],
        "warnings": [
            "This is testing detection and obstacle mapping sequence on a pure pixel scale. Grid expansion is NOT mapped to physical vehicle width.",
            "Area boundaries are impassable."
        ] if PIX_MODE else ["Homography used with arbitrary ROI boundaries."]
    }
    
    if start_grid is None:
        response["status"] = "START_NOT_FOUND" 
    elif goal_grid is None:
        response["status"] = "GOAL_NOT_FOUND"
    else:
        if not (0 <= start_grid[0] < grid_w and 0 <= start_grid[1] < grid_h) or grid[start_grid[1], start_grid[0]] == 1:
            response["status"] = "START_BLOCKED"
        elif not (0 <= goal_grid[0] < grid_w and 0 <= goal_grid[1] < grid_h) or grid[goal_grid[1], goal_grid[0]] == 1:
            response["status"] = "GOAL_BLOCKED"
        else:
            path_idx = astar(grid, start_grid, goal_grid)
            if path_idx is None:
                response["status"] = "NO_PATH"
            else:
                for px, py in path_idx:
                    if PIX_MODE:
                        mx = px * GRID_DS_FACTOR
                        my = py * GRID_DS_FACTOR
                        response["waypoints"].append({"x": int(mx), "y": int(my)})
                    else:
                        mx = px * args.grid_res + (args.grid_res / 2.0)
                        my = py * args.grid_res + (args.grid_res / 2.0)
                        response["waypoints"].append({"x": round(mx, 3), "y": round(my, 3)})

    # Drawing
    if PIX_MODE and response["status"] == "SUCCESS":
        # Draw path on the raw image
        for i in range(len(response["waypoints"]) - 1):
            w1 = response["waypoints"][i]
            w2 = response["waypoints"][i+1]
            cv2.line(img, (int(w1["x"]), int(w1["y"])), (int(w2["x"]), int(w2["y"])), (255, 0, 255), 2)
            
    # Visualize grid logic
    viz_grid = np.zeros((grid_h, grid_w, 3), dtype=np.uint8)
    viz_grid[grid == 0] = [255, 255, 255] 
    viz_grid[grid == 1] = [0, 0, 0]
    
    if start_grid and 0 <= start_grid[0] < grid_w and 0 <= start_grid[1] < grid_h:
        cv2.circle(viz_grid, start_grid, 2, (0, 255, 0), -1)
    if goal_grid and 0 <= goal_grid[0] < grid_w and 0 <= goal_grid[1] < grid_h:
        cv2.circle(viz_grid, goal_grid, 2, (255, 0, 0), -1)
        
    if response["status"] == "SUCCESS":
        for i in range(len(response["waypoints"]) - 1):
            w1 = response["waypoints"][i]
            w2 = response["waypoints"][i+1]
            if PIX_MODE:
                p1 = (int(w1["x"]/GRID_DS_FACTOR), int(w1["y"]/GRID_DS_FACTOR))
                p2 = (int(w2["x"]/GRID_DS_FACTOR), int(w2["y"]/GRID_DS_FACTOR))
            else:
                p1 = (int(w1["x"]/args.grid_res), int(w1["y"]/args.grid_res))
                p2 = (int(w2["x"]/args.grid_res), int(w2["y"]/args.grid_res))
            cv2.line(viz_grid, p1, p2, (0, 0, 255), 1)

    # If NO_PATH, we can print it on the image
    if response["status"] == "NO_PATH":
        cv2.putText(img, "HATA: NO_PATH", (50,50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,0,255), 3)

    os.makedirs(args.output_dir, exist_ok=True)
    cv2.imwrite(os.path.join(args.output_dir, "output_image.jpg"), img)
    cv2.imwrite(os.path.join(args.output_dir, "output_grid.jpg"), viz_grid)
    with open(os.path.join(args.output_dir, "path_results.json"), "w") as f:
        json.dump(response, f, indent=4)
        
    print(f"Bitti. Durum: {response['status']}")

if __name__ == "__main__":
    main()
