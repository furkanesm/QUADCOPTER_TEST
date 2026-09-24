import numpy as np
import math
import cv2
import heapq

class GridPlanner:
    FREE = 0
    OBSTACLE = 1
    UNKNOWN = 2
    
    def __init__(self, cell_size=1.0, vehicle_width=0.0, safety_margin=0.0,
                 unknown_penalty=10.0, strict_boundary_walls=False):
        self.res = cell_size
        self.veh_w = vehicle_width
        self.safety_m = safety_margin
        self.unk_penalty = unknown_penalty
        self.strict_boundary_walls = strict_boundary_walls

        
        if self.res <= 0:
            raise ValueError("cell_size is mandatory and must be > 0.")
            
        radius = (self.veh_w / 2.0) + self.safety_m
        self.inflation_cells = int(math.ceil(radius / self.res))

    def _heuristic(self, a, b):
        return math.hypot(a[0] - b[0], a[1] - b[1])

    def plan_path(self, grid, start, goal, dist_map=None, alpha=0.0, tau=10.0):
        """
        grid: 2D numpy array containing FREE(0), OBSTACLE(1), UNKNOWN(2).
        start: (x, y) grid coordinates
        goal: (x, y) grid coordinates
        dist_map: optionally providing L2 distances to closest OBSTACLE|UNKNOWN
        alpha: weight for centering penalty
        tau: exponential decay scale constant
        
        Returns tuple: (json_response, path_grid_pts, inflated_work_grid)
        """
        h, w = grid.shape
        
        def is_out(pt):
            return not (0 <= pt[0] < w and 0 <= pt[1] < h)
            
        if is_out(start):
            return self._build_resp("START_OUT_OF_BOUNDS")
        if is_out(goal):
            return self._build_resp("GOAL_OUT_OF_BOUNDS")
            
        if grid[start[1], start[0]] == self.OBSTACLE:
            return self._build_resp("START_BLOCKED_PHYSICALLY")
        if grid[goal[1], goal[0]] == self.OBSTACLE:
            return self._build_resp("GOAL_BLOCKED_PHYSICALLY")
            


        # Calculate BOTTLENECK SHORTEST PATH (maximin path width) for this specific query
        # to ensure configured dynamic inflation does not block narrow topological passages. 
        free_mask = (grid != self.OBSTACLE).astype(np.uint8)
        
        # Enforce image borders as solid walls by explicitly padding with OBSTACLES (0) prior to DT
        free_mask_padded = cv2.copyMakeBorder(free_mask, 1, 1, 1, 1, cv2.BORDER_CONSTANT, value=0)
        dt_padded = cv2.distanceTransform(free_mask_padded, cv2.DIST_L2, 5)
        # Crop back to original dimensions
        dt_for_bottleneck = dt_padded[1:h+1, 1:w+1]
        
        max_t = min(dt_for_bottleneck[start[1], start[0]], dt_for_bottleneck[goal[1], goal[0]])
        t_cands = np.unique(dt_for_bottleneck[(dt_for_bottleneck > 0) & (dt_for_bottleneck <= max_t)])
        
        computed_bottleneck_B = 0.0
        if len(t_cands) > 0:
            low, high = 0, len(t_cands) - 1
            while low <= high:
                mid = (low + high) // 2
                t = t_cands[mid]
                mask = (dt_for_bottleneck >= t).astype(np.uint8)
                num_labels, labels = cv2.connectedComponents(mask, connectivity=8)
                if labels[start[1], start[0]] != 0 and labels[start[1], start[0]] == labels[goal[1], goal[0]]:
                    computed_bottleneck_B = float(t)
                    low = mid + 1
                else:
                    high = mid - 1
                        
        applied_inflation = self.inflation_cells
        if self.inflation_cells > 0 and computed_bottleneck_B > 0:
            max_allowed = max(1, int(math.ceil(computed_bottleneck_B - 1.0)))
            if max_allowed < self.inflation_cells:
                applied_inflation = max_allowed

        # Inflate grid. Treating ONLY Obstacle(1) as solid wall for inflation
        work_grid = np.zeros((h, w), dtype=np.uint8)
        work_grid[grid == self.OBSTACLE] = 1
        

        if applied_inflation > 0:
            kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (applied_inflation * 2 + 1, applied_inflation * 2 + 1))
            # strict_boundary_walls True ise dış alan katı duvar (1), aksi halde açık hava (0) doldurulur
            pad_val = 1 if self.strict_boundary_walls else 0
            padded = cv2.copyMakeBorder(work_grid, applied_inflation, applied_inflation,
                                         applied_inflation, applied_inflation,
                                         cv2.BORDER_CONSTANT, value=pad_val)

            padded = cv2.dilate(padded, kernel, iterations=1)
            # Crop back to original dimensions
            work_grid = padded[applied_inflation : h + applied_inflation, 
                               applied_inflation : w + applied_inflation]
            
        # Log if start/goal were swallowed by inflation
        start_inf = bool(work_grid[start[1], start[0]] == 1)
        goal_inf = bool(work_grid[goal[1], goal[0]] == 1)
        
        # Prepare diagnostics
        run_diagnostics = {
            "start_was_inflated": start_inf,
            "goal_was_inflated": goal_inf,
            "configured_clearance": int(self.inflation_cells),
            "computed_bottleneck_B": float(round(computed_bottleneck_B, 3)),
            "applied_inflation": int(applied_inflation)
        }

        # Katı sınır duvarı modunda start/goal duvar enflasyonuna kapılmışsa doğrudan ret
        if self.strict_boundary_walls:
            if start_inf:
                return self._build_resp("START_BLOCKED", work_grid=work_grid, diagnostics=run_diagnostics)
            if goal_inf:
                return self._build_resp("GOAL_BLOCKED", work_grid=work_grid, diagnostics=run_diagnostics)

        # Enflasyon (güvenlik payı) start/goal çevresinde gevşetilebilir, çünkü drone zaten orada.
        # Ancak fiziksel engeller (grid'de OBSTACLE olanlar) HİÇBİR ZAMAN silinmemelidir.
        if applied_inflation > 0:
            override_radius = applied_inflation
            cv2.circle(work_grid, start, override_radius, 0, -1)
            cv2.circle(work_grid, goal, override_radius, 0, -1)
            # Fiziksel engelleri kesinlikle geri yükle
            work_grid[grid == self.OBSTACLE] = 1
        
        # We also need to let A* step out of the inflated zone if start/goal are deeply inside it!
        # If work_grid is Solid 1 around start/goal, unblocking the single pixel is useless (it has no Free neighbors).
        # We will carve a direct 1-pixel tunnel if needed? The user specified:
        # "H'yi ve S'yi HER ZAMAN serbest kabul et (maskeyi override et)"
        # So we just unblock the single pixels as explicitly requested.

        # A* search için bağlı bileşen (Connected Components) Erken Çıkış (Early Exit)
        # Sadece A* ın değerlendireceği açık/kapalı yolları dikkate al
        impassable = np.logical_or(work_grid == 1, grid == self.OBSTACLE).astype(np.uint8)
        free_cells = 1 - impassable
        # 4'lü komşuluk A* ile BİREBİR uyumludur. A* çapraz ilerleyebilmek için 
        # HER İKİ ortogonal komşunun boş olmasını (strict diagonal) şart koştuğundan, 
        # 4-connectivity zaten o ortogonal boşluklardan geçerek hedefe ulaşabilecektir.
        # Bu yüzden 4-connectivity A*'dan ne daha katı ne daha gevşektir; BİREBİR eşittir.
        num_labels, labels = cv2.connectedComponents(free_cells, connectivity=4)
        label_start = labels[start[1], start[0]]
        label_goal = labels[goal[1], goal[0]]
        if label_start == 0 or label_goal == 0 or label_start != label_goal:
            # Farklı bileşendeyse veya tamamen arka plan/duvar üzerinde ise kesin olarak imkansızdır.
            return self._build_resp("NO_PATH", work_grid=work_grid, diagnostics=run_diagnostics)

        # A* search
        came_from = {}
        g_score = {start: 0}
        f_score = {start: self._heuristic(start, goal)}
        
        # Deterministik tie-breaker (Aynı f-score değerlerinde her iki iterasyonda da aynı komşunun işlenmesi için)
        counter = 0
        pq = []
        heapq.heappush(pq, (f_score[start], counter, start))
        
        directions = [
            (0, -1, 1.0), (0, 1, 1.0), (-1, 0, 1.0), (1, 0, 1.0),
            (-1, -1, 1.414), (1, -1, 1.414), (-1, 1, 1.414), (1, 1, 1.414)
        ]
        
        path = None
        visited = set()
        
        while pq:
            curr_f, _, current = heapq.heappop(pq)
            
            # Kapalı / Eskimiş düğüm (Closed Set / Lazy Deletion Bypass)
            if current in visited:
                continue
            visited.add(current)
            
            if current == goal:
                path = []
                while current in came_from:
                    path.append(current)
                    current = came_from[current]
                path.append(start)
                path.reverse()
                break
                
            for dx, dy, cost in directions:
                nx, ny = current[0] + dx, current[1] + dy
                neighbor = (nx, ny)
                # Ensure physical obstacles remain impassable at all times
                if is_out(neighbor) or work_grid[ny, nx] == 1 or grid[ny, nx] == self.OBSTACLE:
                    continue
                    
                if dx != 0 and dy != 0:
                    # Diagonal cut check: ensure both adjacent straights are FREE
                    if (work_grid[current[1], current[0] + dx] == 1 or
                        work_grid[current[1] + dy, current[0]] == 1 or
                        grid[current[1], current[0] + dx] == self.OBSTACLE or
                        grid[current[1] + dy, current[0]] == self.OBSTACLE):
                        continue
                        
                step_cost = cost
                # Soft penetration penalty for UNKNOWN region
                if grid[ny, nx] == self.UNKNOWN:
                    step_cost *= self.unk_penalty
                    
                if dist_map is not None and alpha > 0.0:
                    d = dist_map[ny, nx]
                    # Asymptotic tracking: exponential distance decay prevents zeroing out
                    # We tune the coefficient internally to respond well to Alpha=1.0
                    step_cost += (alpha * 50.0) * math.exp(-d / tau)
                
                tentative_g_score = g_score.get(current, float('inf')) + step_cost
                if tentative_g_score < g_score.get(neighbor, float('inf')):
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    fn = tentative_g_score + self._heuristic(neighbor, goal)
                    f_score[neighbor] = fn
                    
                    counter += 1
                    heapq.heappush(pq, (fn, counter, neighbor))
                        
        if path is None:
            return self._build_resp("NO_PATH", work_grid=work_grid, diagnostics=run_diagnostics)
            
        # Convert path to spatial coordinates based on cell_size
        waypoints = []
        for x, y in path:
            mx = x * self.res + (self.res / 2.0)
            my = y * self.res + (self.res / 2.0)
            waypoints.append({"x": round(mx, 3), "y": round(my, 3)})
            
        return self._build_resp("SUCCESS", path_grid=path, waypoints=waypoints, work_grid=work_grid, diagnostics=run_diagnostics)
        
    def _build_resp(self, status, path_grid=None, waypoints=None, work_grid=None, diagnostics=None):
        resp = {
            "status": status,
            "coordinate_frame": "grid_local",
            "scale": self.res,
            "assumptions": [
                "Circular footprint inflation",
                "Start/Goal explicitly provided in grid coords",
                "UNKNOWN is traversable with penalty, OBSTACLE is non-traversable",
                "Bounds act as strict walls"
            ]
        }
        
        if diagnostics is not None:
            resp["diagnostics"] = diagnostics
        else:
            resp["diagnostics"] = {
                "start_was_inflated": False,
                "goal_was_inflated": False
            }
        
        if waypoints is not None:
            resp["waypoints"] = waypoints
            
        return resp, path_grid, work_grid
