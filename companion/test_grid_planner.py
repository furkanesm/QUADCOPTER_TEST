import numpy as np
import cv2
import json
import os
from grid_path_planner import GridPlanner

def generate_visualization(grid, start, goal, work_grid, path_grid, filename, status):
    # Free = White (255), Obstacle = Black (0), Unknown = Gray (128)
    img = np.zeros((grid.shape[0], grid.shape[1], 3), dtype=np.uint8)
    img[grid == GridPlanner.FREE] = [255, 255, 255]
    img[grid == GridPlanner.OBSTACLE] = [0, 0, 0]
    img[grid == GridPlanner.UNKNOWN] = [128, 128, 128]
    
    # Overlay inflation as red-ish overlay where work_grid is 1 but original is FREE
    inflated_mask = (work_grid == 1) & (grid == GridPlanner.FREE)
    img[inflated_mask] = [200, 200, 255] # BGR light red
    
    if start and not work_grid[start[1], start[0]] == 1:
        img[start[1], start[0]] = [0, 255, 0] # Green start
    elif start:
        img[start[1], start[0]] = [0, 100, 0] # Dark green (blocked)
        
    if goal and not work_grid[goal[1], goal[0]] == 1:
        img[goal[1], goal[0]] = [255, 0, 0] # Blue goal
    elif goal:
        img[goal[1], goal[0]] = [100, 0, 0] # Dark blue (blocked)
        
    img = cv2.resize(img, (400, 400), interpolation=cv2.INTER_NEAREST)
    
    ratio = 400 / grid.shape[0]
    
    if path_grid:
        for i in range(len(path_grid)-1):
            p1 = (int(path_grid[i][0]*ratio + ratio/2), int(path_grid[i][1]*ratio + ratio/2))
            p2 = (int(path_grid[i+1][0]*ratio + ratio/2), int(path_grid[i+1][1]*ratio + ratio/2))
            cv2.line(img, p1, p2, (255, 0, 255), 2)
            
    cv2.putText(img, status, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
    cv2.imwrite(filename, img)

def run_scenario(name, grid, start, goal, planner):
    print(f"\n--- Senaryo: {name} ---")
    resp, path_grid, work_grid = planner.plan_path(grid, start, goal)
    
    print(json.dumps(resp, indent=2))
    
    if work_grid is not None:
        filename = f"scenario_{name.replace(' ', '_').lower()}.png"
        generate_visualization(grid, start, goal, work_grid, path_grid, filename, resp["status"])

def main():
    # 20x20 grid mapping, meters_per_cell = 0.5m. Veh width = 1.0m (safety 0). radius=0.5m -> 1 cell inflation
    planner = GridPlanner(meters_per_cell=0.5, vehicle_width_m=1.0, safety_margin_m=0.0)
    
    # 1. Open Corridor with turns
    g1 = np.zeros((20, 20), dtype=np.uint8)
    g1[5:15, 5] = GridPlanner.OBSTACLE
    g1[5, 5:15] = GridPlanner.OBSTACLE
    run_scenario("Open Corridor", g1, (2, 2), (10, 10), planner)
    
    # 2. Closed Corridor (NO_PATH)
    g2 = np.zeros((20, 20), dtype=np.uint8)
    g2[0:20, 10] = GridPlanner.OBSTACLE
    run_scenario("Closed Corridor", g2, (2, 2), (18, 15), planner)
    
    # 3. Too narrow corridor (unpassable due to inflation)
    # Both walls set so that passage is exactly 1 cell wide strictly (x=9 is 0, but x=8,10 is 1)
    g3 = np.zeros((20, 20), dtype=np.uint8)
    g3[5:15, 8] = GridPlanner.OBSTACLE
    g3[5:15, 10] = GridPlanner.OBSTACLE
    run_scenario("Narrow Corridor", g3, (9, 2), (9, 18), planner)
    
    # 4. Start/Goal on obstacle
    g4 = np.zeros((20, 20), dtype=np.uint8)
    g4[10, 10] = GridPlanner.OBSTACLE
    run_scenario("Goal Blocked", g4, (2, 2), (10, 10), planner)
    
    # 5. Unknown region shortcut prevented
    g5 = np.zeros((20, 20), dtype=np.uint8)
    g5[5:15, 10] = GridPlanner.UNKNOWN # Treat as obstacle
    g5[0:5, 10] = GridPlanner.OBSTACLE
    run_scenario("Unknown Shortcut", g5, (2, 10), (18, 10), planner)

if __name__ == "__main__":
    main()
