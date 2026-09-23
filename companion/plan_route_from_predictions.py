import os
import json
import argparse
import sys
import heapq
import numpy as np
import cv2
from pathlib import Path

def read_image(path_str, flags):
    p = Path(path_str)
    if not p.is_file():
        print(f"Error: File not found at absolute path: {p.absolute()}")
        sys.exit(1)
    data = p.read_bytes()
    if not data:
        print(f"Error: File is empty: {p.absolute()}")
        sys.exit(1)
    np_arr = np.frombuffer(data, dtype=np.uint8)
    img = cv2.imdecode(np_arr, flags)
    if img is None:
        print(f"Error: Failed to decode image: {p.absolute()}")
        sys.exit(1)
    return img

def write_image(path_str, img):
    p = Path(path_str)
    success, encoded_img = cv2.imencode('.png', img)
    if not success:
        print("Error: Failed to encode image to PNG format.")
        sys.exit(1)
    p.write_bytes(encoded_img.tobytes())
    verify_img = read_image(path_str, cv2.IMREAD_UNCHANGED)
    if verify_img.shape[:2] != img.shape[:2]:
        print(f"Error: Verify read size mismatch for written PNG at {p.absolute()}")
        sys.exit(1)

def main():
    import hashlib
    script_path = os.path.abspath(__file__)
    try:
        script_hash = hashlib.sha256(open(script_path, 'rb').read()).hexdigest()
    except Exception as e:
        script_hash = "Could not compute hash"
        
    parser = argparse.ArgumentParser(description="Offline route planner")
    parser.add_argument("--start_x", type=int, default=-1, help="Start X")
    parser.add_argument("--start_y", type=int, default=-1, help="Start Y")
    parser.add_argument("--interactive", action='store_true', help="Use Tkinter to click start point")
    parser.add_argument("--connect_gaps", action='store_true', help="Enable gap connection mapping")
    parser.add_argument("--max_gap", type=float, default=60.0, help="Maximum gap size to traverse between roads")
    parser.add_argument("--corridor_width", type=float, default=20.0, help="Width of the permitted gap area")
    parser.add_argument("--safety_margin", type=float, default=15.0, help="Obstacle safety margin in px for the entire route")
    parser.add_argument("--restricted_zone", type=str, default="", help="JSON list of [x,y,w,h] to restrict gaps")

    args = parser.parse_args()

    base_dir = os.path.dirname(os.path.abspath(__file__))
    res_dir = os.path.join(base_dir, "offline_results")
    out_dir = os.path.join(base_dir, "offline_planning_results")
    os.makedirs(out_dir, exist_ok=True)

    results_json_path = os.path.join(res_dir, "results.json")
    if not os.path.exists(results_json_path):
        print(f"Error: Could not find {results_json_path}")
        sys.exit(1)

    with open(results_json_path, "r", encoding="utf-8") as f:
        res = json.load(f)

    rgb_path = res["image"]["path"]
    mask_path = os.path.join(res_dir, "road_mask.png")

    rgb = read_image(rgb_path, cv2.IMREAD_COLOR)
    mask = read_image(mask_path, cv2.IMREAD_GRAYSCALE)
    print(f"INFO: Python Script Absolute Path: {script_path}")
    print(f"INFO: Script SHA256: {script_hash}")
        
    h, w = mask.shape
    if rgb.shape[:2] != (h, w):
        print("Error: Resolution mismatch between RGB and mask")
        sys.exit(1)

    hedef_boxes = [d for d in res["detections"] if d["class_name"] == "Hedef"]
    if len(hedef_boxes) == 0:
        print("Error: No 'Hedef' found in JSON.")
        sys.exit(1)
    elif len(hedef_boxes) > 1:
        print("Error: Multiple 'Hedef' found. Cannot pick one randomly, explicit target needed.")
        sys.exit(1)
        
    hb = hedef_boxes[0]["bbox"]
    goal_x = int((hb[0] + hb[2]) / 2)
    goal_y = int((hb[1] + hb[3]) / 2)

    obs_boxes = [d for d in res["detections"] if d["class_name"] == "engel"]
    obstacle_map = np.ones((h, w), dtype=np.uint8) * 255
    for b in obs_boxes:
        x1, y1, x2, y2 = map(int, b["bbox"])
        cv2.rectangle(obstacle_map, (x1, y1), (x2, y2), 0, -1)

    dist_to_obs = cv2.distanceTransform(obstacle_map, cv2.DIST_L2, 5)
    dist_to_road_edge_internal = cv2.distanceTransform(mask, cv2.DIST_L2, 5)

    restricted_boxes = []
    if args.restricted_zone:
        try: restricted_boxes = json.loads(args.restricted_zone)
        except: pass

    dirs = [(0,1), (1,0), (0,-1), (-1,0), (1,1), (-1,1), (1,-1), (-1,-1)]

    # --- 1. DETERMINE START POINT ---
    start_x, start_y = args.start_x, args.start_y
    auto_detected = False
    entrance_tip, exit_tip = None, None
    selection_reason = ""
    
    if start_x < 0 or start_y < 0:
        if args.interactive:
            import tkinter as tk
            from PIL import Image, ImageTk
            clone = rgb.copy()
            clone_rgb = cv2.cvtColor(clone, cv2.COLOR_BGR2RGB)
            root = tk.Tk()
            scale = min((root.winfo_screenwidth() * 0.9) / clone_rgb.shape[1], (root.winfo_screenheight() * 0.8) / clone_rgb.shape[0])
            scale = min(1.0, scale)
            disp_w, disp_h = int(clone_rgb.shape[1] * scale), int(clone_rgb.shape[0] * scale)
            disp_img_pil = Image.fromarray(clone_rgb).resize((disp_w, disp_h), Image.Resampling.LANCZOS)
            img_tk = ImageTk.PhotoImage(disp_img_pil)
            canvas = tk.Canvas(root, width=disp_w, height=disp_h, bg='gray')
            canvas.pack()
            canvas.create_image(0, 0, anchor=tk.NW, image=img_tk)
            state = {'sel_x': -1, 'sel_y': -1, 'circle_id': None}
            def on_mouse_click(event):
                cx, cy = event.x, event.y
                if 0 <= cx < disp_w and 0 <= cy < disp_h:
                    state['sel_x'], state['sel_y'] = int(cx / scale), int(cy / scale)
                    if state['circle_id']: canvas.delete(state['circle_id'])
                    state['circle_id'] = canvas.create_oval(cx-5, cy-5, cx+5, cy+5, fill="blue")
            canvas.bind("<Button-1>", on_mouse_click)
            tk.Button(root, text="Hesapla", command=root.quit).pack()
            root.protocol("WM_DELETE_WINDOW", root.quit)
            root.mainloop()
            start_x, start_y = state['sel_x'], state['sel_y']
            root.destroy()
            if start_x < 0 or start_y < 0: sys.exit(1)
            selection_reason = "User interactive click"
        else:
            num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(mask, connectivity=8)
            if num_labels < 2: sys.exit(1)
            largest_label = 1 + np.argmax(stats[1:, cv2.CC_STAT_AREA])
            road_mask = (labels == largest_label).astype(np.uint8) * 255
            min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(dist_to_road_edge_internal, mask=road_mask)
            P0 = max_loc
            
            def get_farthest_point(start_p):
                q = [start_p]
                visited = {start_p: 0}
                farthest, max_d = start_p, 0
                while q:
                    cx, cy = q.pop(0)
                    d = visited[(cx, cy)]
                    if d > max_d: max_d, farthest = d, (cx, cy)
                    for dx, dy in dirs:
                        nx, ny = cx+dx, cy+dy
                        if 0 <= nx < w and 0 <= ny < h and road_mask[ny, nx] > 0 and (nx, ny) not in visited:
                            visited[(nx, ny)] = d + 1
                            q.append((nx, ny))
                return farthest
                
            P1 = get_farthest_point(P0)
            P2 = get_farthest_point(P1)
            d1 = np.linalg.norm(np.array(P1) - np.array([goal_x, goal_y]))
            d2 = np.linalg.norm(np.array(P2) - np.array([goal_x, goal_y]))
            
            if abs(d1 - d2) < 50:
                print("Error: AMBIGUOUS_START.")
                sys.exit(1)
                
            entrance_tip, exit_tip = (P1, P2) if d1 > d2 else (P2, P1)
            
            from heapq import heappop, heappush
            queue = [(0, entrance_tip)]
            c_came_from = {}
            c_g_score = {entrance_tip: 0}
            found_c = False
            
            while queue:
                _, curr = heappop(queue)
                if curr == exit_tip:
                    found_c = True; break
                cx, cy = curr
                for dx, dy in dirs:
                    nx, ny = cx+dx, cy+dy
                    if 0 <= nx < w and 0 <= ny < h and road_mask[ny, nx] > 0:
                        cost = 1.0 + 500.0 / (dist_to_road_edge_internal[ny, nx] + 1e-1)
                        dist_step = 1.414 if dx!=0 and dy!=0 else 1.0
                        tentative_g = c_g_score[curr] + cost * dist_step
                        if tentative_g < c_g_score.get((nx, ny), float('inf')):
                            c_came_from[(nx, ny)] = curr
                            c_g_score[(nx, ny)] = tentative_g
                            heappush(queue, (tentative_g, (nx, ny)))
                            
            if not found_c: sys.exit(1)
            path = []
            curr = exit_tip
            while curr != entrance_tip:
                path.append(curr)
                curr = c_came_from[curr]
            path.append(entrance_tip)
            path.reverse()
            
            final_start = None
            for px, py in path:
                if dist_to_obs[py, px] >= args.safety_margin:
                    final_start = (px, py)
                    break
            
            if final_start is None: sys.exit(1)
            start_x, start_y = final_start
            auto_detected = True
            selection_reason = f"First point ensuring dist_to_obs ({dist_to_obs[start_y, start_x]:.1f}px) >= safety ({args.safety_margin}px) along centerline."

    # --- 2. ACCESSIBLE ROAD MASK ---
    def is_safe_pass(cx, cy, nx, ny, dx, dy):
        if dist_to_obs[ny, nx] < args.safety_margin: return False
        if dx != 0 and dy != 0:
            if dist_to_obs[cy, cx+dx] < args.safety_margin or dist_to_obs[cy+dy, cx] < args.safety_margin: return False
        return True

    accessible_road_mask = np.zeros((h, w), dtype=np.uint8)
    q = [(start_x, start_y)]
    accessible_road_mask[start_y, start_x] = 255
    while q:
        cx, cy = q.pop(0)
        for dx, dy in dirs:
            nx, ny = cx+dx, cy+dy
            if 0 <= nx < w and 0 <= ny < h:
                if mask[ny, nx] >= 127 and accessible_road_mask[ny, nx] == 0:
                    if is_safe_pass(cx, cy, nx, ny, dx, dy):
                        accessible_road_mask[ny, nx] = 255
                        q.append((nx, ny))

    # --- 3. EXPLICIT GAP CORRIDOR FINDING ---
    valid_offroad_mask = np.zeros((h, w), dtype=np.uint8)
    connection_logs = []
    
    if args.connect_gaps:
        box_hit = False
        goal_range_y = slice(int(hb[1]), int(hb[3])+1)
        goal_range_x = slice(int(hb[0]), int(hb[2])+1)
        if np.max(accessible_road_mask[goal_range_y, goal_range_x]) > 0:
            box_hit = True

        if not box_hit:
            roi_slack = int(args.max_gap) + int(args.corridor_width)
            rx1 = max(0, goal_x - roi_slack)
            ry1 = max(0, goal_y - roi_slack)
            rx2 = min(w, goal_x + roi_slack)
            ry2 = min(h, goal_y + roi_slack)

            # Local A* from Target H searching for accessible_road_mask
            from heapq import heappop, heappush
            local_q = [(0, (goal_x, goal_y))]
            loc_came_from = {}
            loc_g_score = {(goal_x, goal_y): 0}
            gap_path_found = False
            best_meet_pt = None
            
            while local_q:
                curr_cost, curr = heappop(local_q)
                cx, cy = curr
                
                if accessible_road_mask[cy, cx] > 0:
                    gap_path_found = True
                    best_meet_pt = curr
                    break
                    
                for dx, dy in dirs:
                    nx, ny = cx + dx, cy + dy
                    if rx1 <= nx < rx2 and ry1 <= ny < ry2:
                        if not is_safe_pass(cx, cy, nx, ny, dx, dy):
                            continue
                        dist_step = 1.414 if (dx != 0 and dy != 0) else 1.0
                        tentative_g = curr_cost + dist_step
                        if tentative_g <= args.max_gap:
                            if tentative_g < loc_g_score.get((nx, ny), float('inf')):
                                loc_came_from[(nx, ny)] = curr
                                loc_g_score[(nx, ny)] = tentative_g
                                heappush(local_q, (tentative_g, (nx, ny)))
                                
            if gap_path_found:
                gap_len = loc_g_score[best_meet_pt]
                rp = []
                c = best_meet_pt
                min_obs_on_path = float('inf')
                while c != (goal_x, goal_y):
                    rp.append(c)
                    cd = dist_to_obs[c[1], c[0]]
                    if cd < min_obs_on_path: min_obs_on_path = cd
                    c = loc_came_from[c]
                rp.append((goal_x, goal_y))
                
                pts = np.array(rp, np.int32).reshape((-1,1,2))
                cv2.polylines(valid_offroad_mask, [pts], False, 255, int(args.corridor_width))
                
                for rb in restricted_boxes:
                    rx, ry, rw, rh = rb
                    cv2.rectangle(valid_offroad_mask, (rx, ry), (rx+rw, ry+rh), 0, -1)
                
                area = cv2.countNonZero(valid_offroad_mask)
                connection_logs.append({
                    "type": "Local_ROI_AStar_H_Connection",
                    "start_pixel_on_accessible_road": list(map(int, best_meet_pt)),
                    "end_pixel_at_target": [goal_x, goal_y],
                    "physical_offroad_length_cost": float(gap_len),
                    "opened_corridor_area": int(area),
                    "min_obs_distance_on_path": float(min_obs_on_path),
                    "accepted": True,
                    "reason": f"A* found obstacle-avoiding safe path of true length {gap_len:.1f}px (<= max_gap {args.max_gap})"
                })
            else:
                connection_logs.append({
                    "type": "Local_ROI_AStar_H_Connection",
                    "accepted": False,
                    "reason": f"Local A* failed within ROI. No obstacle-free connection path length <= {args.max_gap}px found."
                })

    # --- 4. GLOBAL A* ---
    global_traversable = cv2.bitwise_or(accessible_road_mask, valid_offroad_mask)

    def heuristic(p1, p2):
        return np.linalg.norm(np.array(p1) - np.array(p2))

    queue = []
    heapq.heappush(queue, (0, 0, (start_x, start_y)))
    came_from = {}
    g_score = {(start_x, start_y): 0}
    reached = False
    reached_hedef_box = False
    reached_hedef_center = False
    best_node = None
    
    while queue:
        _, current_g, current = heapq.heappop(queue)
        cx, cy = current
        
        in_box = (hb[0] <= cx <= hb[2] and hb[1] <= cy <= hb[3])
        if cx == goal_x and cy == goal_y:
            reached = reached_hedef_center = reached_hedef_box = True
            best_node = current
            break
        elif in_box and not reached_hedef_box:
            reached_hedef_box = True
            reached = True 
            best_node = current
            break
            
        for dx, dy in dirs:
            nx, ny = cx + dx, cy + dy
            if 0 <= nx < w and 0 <= ny < h:
                if global_traversable[ny, nx] == 0: continue
                if not is_safe_pass(cx, cy, nx, ny, dx, dy): continue
                
                dist_step = 1.414 if (dx != 0 and dy != 0) else 1.0
                
                if accessible_road_mask[ny, nx] > 0:
                    edge_penalty = 50.0 / (dist_to_road_edge_internal[ny, nx] + 1e-1)
                    cost = edge_penalty
                else:
                    cost = 500.0 # High penalty for being offroad 
                    
                tentative_g = current_g + dist_step + (cost * dist_step)
                if tentative_g < g_score.get((nx, ny), float('inf')):
                    came_from[(nx, ny)] = current
                    g_score[(nx, ny)] = tentative_g
                    f_score = tentative_g + heuristic((nx, ny), (goal_x, goal_y))
                    heapq.heappush(queue, (f_score, tentative_g, (nx, ny)))

    route = []
    furthest_connected = (start_x, start_y)
    if not reached:
        min_dist_to_goal = float('inf')
        for node in g_score.keys():
            d = heuristic(node, (goal_x, goal_y))
            if d < min_dist_to_goal:
                min_dist_to_goal = d
                furthest_connected = node
                
    if reached:
        curr = best_node
        while curr in came_from:
            route.append(curr)
            curr = came_from[curr]
        route.append((start_x, start_y))
        route.reverse()

    # --- 5. OVERLAYS & JSON ---
    overlay = rgb.copy()
    cv2.rectangle(overlay, (int(hb[0]), int(hb[1])), (int(hb[2]), int(hb[3])), (0, 255, 0), 3)
    cv2.circle(overlay, (goal_x, goal_y), 6, (0, 255, 0), -1)
    
    if args.connect_gaps:
        try:
            cv2.rectangle(overlay, (rx1, ry1), (rx2, ry2), (0, 165, 255), 1, cv2.LINE_AA)
            cv2.putText(overlay, "Local A* ROI", (rx1+5, ry1+15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 165, 255), 1)
        except: pass
        overlay_mask = valid_offroad_mask > 0
        overlay[overlay_mask] = (overlay[overlay_mask] * 0.5 + np.array([255, 0, 255]) * 0.5).astype(np.uint8)

    min_obs_dist_on_route = float('inf')
    if route:
        for i in range(len(route) - 1):
            p1, p2 = route[i], route[i+1]
            do_p1 = dist_to_obs[p1[1], p1[0]]
            if do_p1 < min_obs_dist_on_route: min_obs_dist_on_route = do_p1
            is_gap = valid_offroad_mask[p1[1], p1[0]] > 0 or valid_offroad_mask[p2[1], p2[0]] > 0
            color = (255, 0, 255) if is_gap else (0, 0, 255) 
            cv2.line(overlay, p1, p2, color, 3)
            
        do_last = dist_to_obs[route[-1][1], route[-1][0]]
        if do_last < min_obs_dist_on_route: min_obs_dist_on_route = do_last
            
    if not reached:
        cv2.circle(overlay, furthest_connected, 8, (0, 165, 255), -1)
        cv2.putText(overlay, "BLOCKED", (furthest_connected[0]+10, furthest_connected[1]-10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 165, 255), 2)
        cv2.line(overlay, furthest_connected, (goal_x, goal_y), (255, 255, 0), 2, cv2.LINE_AA)
            
    if auto_detected and entrance_tip:
        cv2.circle(overlay, list(entrance_tip), 8, (255, 255, 0), -1)
        cv2.line(overlay, list(entrance_tip), (start_x, start_y), (255, 255, 0), 2, cv2.LINE_AA)
        
    cv2.circle(overlay, (start_x, start_y), 8, (255, 0, 0), -1)

    fail_reason = "" if reached else "No contiguous path found within constraints."
    out_img = os.path.join(out_dir, "route_overlay.png")
    write_image(out_img, overlay)

    res_data = {
        "status": "SUCCESS" if reached else "FAILED",
        "fail_reason": fail_reason,
        "termination_condition": {
            "reached_hedef_box": reached_hedef_box,
            "reached_hedef_center": reached_hedef_center,
            "final_coordinate": list(map(int, best_node)) if best_node else None,
            "furthest_connected_point_if_blocked": list(map(int, furthest_connected))
        },
        "connection_logs": connection_logs,
        "parameters": {
            "connect_gaps": args.connect_gaps,
            "max_gap": args.max_gap,
            "corridor_width": args.corridor_width,
            "safety_margin": args.safety_margin,
            "restricted_zone": args.restricted_zone
        },
        "start_source": "auto_road_entrance" if auto_detected else "user_interactive" if args.interactive else "cli_args",
        "start": [start_x, start_y],
        "auto_detection": {
            "entrance_tip": list(map(int, entrance_tip)) if entrance_tip else None,
            "exit_tip": list(map(int, exit_tip)) if exit_tip else None,
            "selection_reason": selection_reason
        } if auto_detected else None,
        "goal": [goal_x, goal_y],
        "route_length": len(route),
        "min_obs_dist": float(min_obs_dist_on_route) if min_obs_dist_on_route != float('inf') else None
    }
    
    out_json = os.path.join(out_dir, "route_results.json")
    with open(out_json, "w", encoding="utf-8") as f:
        json.dump(res_data, f, indent=4, ensure_ascii=False)

    print(f"Status: {'SUCCESS' if reached else 'FAILED'}")
    if fail_reason: print(f"Reason: {fail_reason}")
    print(f"Files saved to {out_dir}")

if __name__ == "__main__":
    main()
