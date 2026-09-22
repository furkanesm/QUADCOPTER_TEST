import numpy as np
import math
import cv2

class GridPlanner:
    FREE = 0
    OBSTACLE = 1
    UNKNOWN = 2
    
    def __init__(self, cell_size=1.0, vehicle_width=0.0, safety_margin=0.0, strict_boundary_walls=False):
        self.res = cell_size
        self.veh_w = vehicle_width
        self.safety_m = safety_margin
        self.strict_boundary_walls = strict_boundary_walls
        
        if self.res <= 0:
            raise ValueError("cell_size is mandatory and must be > 0.")
            
        radius = (self.veh_w / 2.0) + self.safety_m
        self.inflation_cells = int(math.ceil(radius / self.res))

    def _heuristic(self, a, b):
        return math.hypot(a[0] - b[0], a[1] - b[1])

    def plan_path(self, grid, start, goal):
        """
        grid: 2D numpy array containing FREE(0), OBSTACLE(1), UNKNOWN(2).
        start: (x, y) grid coordinates
        goal: (x, y) grid coordinates
        
        Returns tuple: (json_response, path_grid_pts, inflated_work_grid)
        """
        h, w = grid.shape
        
        def is_out(pt):
            return not (0 <= pt[0] < w and 0 <= pt[1] < h)
            
        if is_out(start):
            return self._build_resp("START_OUT_OF_BOUNDS")
        if is_out(goal):
            return self._build_resp("GOAL_OUT_OF_BOUNDS")
            
        # Inflate grid. Treating Unknown(2) as Obstacle(1) for inflation limits
        work_grid = np.zeros((h, w), dtype=np.uint8)
        work_grid[(grid == self.OBSTACLE) | (grid == self.UNKNOWN)] = 1
        
        if self.inflation_cells > 0:
            kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (self.inflation_cells * 2 + 1, self.inflation_cells * 2 + 1))
            # strict_boundary_walls True ise dış alan katı duvar (1), aksi halde açık hava (0) doldurulur
            pad_val = 1 if self.strict_boundary_walls else 0
            padded = cv2.copyMakeBorder(work_grid, self.inflation_cells, self.inflation_cells, 
                                        self.inflation_cells, self.inflation_cells, 
                                        cv2.BORDER_CONSTANT, value=pad_val)
            padded = cv2.dilate(padded, kernel, iterations=1)
            # Crop back to original dimensions
            work_grid = padded[self.inflation_cells : h + self.inflation_cells, 
                               self.inflation_cells : w + self.inflation_cells]
            
        if work_grid[start[1], start[0]] == 1:
            return self._build_resp("START_BLOCKED", work_grid=work_grid)
        if work_grid[goal[1], goal[0]] == 1:
            return self._build_resp("GOAL_BLOCKED", work_grid=work_grid)
            
        # A* search
        open_set = {start}
        came_from = {}
        g_score = {start: 0}
        f_score = {start: self._heuristic(start, goal)}
        
        directions = [
            (0, -1, 1.0), (0, 1, 1.0), (-1, 0, 1.0), (1, 0, 1.0),
            (-1, -1, 1.414), (1, -1, 1.414), (-1, 1, 1.414), (1, 1, 1.414)
        ]
        
        path = None
        while open_set:
            current = min(open_set, key=lambda x: f_score.get(x, float('inf')))
            if current == goal:
                path = []
                while current in came_from:
                    path.append(current)
                    current = came_from[current]
                path.append(start)
                path.reverse()
                break
                
            open_set.remove(current)
            for dx, dy, cost in directions:
                nx, ny = current[0] + dx, current[1] + dy
                neighbor = (nx, ny)
                if is_out(neighbor) or work_grid[ny, nx] == 1:
                    continue
                    
                if dx != 0 and dy != 0:
                    # Diagonal cut check: ensure both adjacent straights are FREE
                    if work_grid[current[1], current[0] + dx] == 1 or work_grid[current[1] + dy, current[0]] == 1:
                        continue
                
                tentative_g_score = g_score.get(current, float('inf')) + cost
                if tentative_g_score < g_score.get(neighbor, float('inf')):
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = tentative_g_score + self._heuristic(neighbor, goal)
                    if neighbor not in open_set:
                        open_set.add(neighbor)
                        
        if path is None:
            return self._build_resp("NO_PATH", work_grid=work_grid)
            
        # Convert path to spatial coordinates based on cell_size
        waypoints = []
        for x, y in path:
            mx = x * self.res + (self.res / 2.0)
            my = y * self.res + (self.res / 2.0)
            waypoints.append({"x": round(mx, 3), "y": round(my, 3)})
            
        return self._build_resp("SUCCESS", path_grid=path, waypoints=waypoints, work_grid=work_grid)
        
    def _build_resp(self, status, path_grid=None, waypoints=None, work_grid=None):
        resp = {
            "status": status,
            "coordinate_frame": "grid_local",
            "scale": self.res,
            "assumptions": [
                "Circular footprint inflation",
                "Start/Goal explicitly provided in grid coords",
                "UNKNOWN and OBSTACLE are non-traversable",
                "Bounds act as strict walls"
            ]
        }
        if waypoints is not None:
            resp["waypoints"] = waypoints
            
        return resp, path_grid, work_grid
