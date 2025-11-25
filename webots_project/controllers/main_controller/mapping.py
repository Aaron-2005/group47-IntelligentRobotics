from controller import Display, Lidar, InertialUnit, PositionSensor
import math
import numpy as np


class Mapping:
    def __init__(self, robot):
        self.robot = robot
        self.TIME_STEP = int(robot.getBasicTimeStep())
        # robot odometry calculation convertables
        self.WHEEL_RADIUS = 0.033
        self.WHEEL_BASE = 0.160

        # map area 
        self.MAP_SIZE_M = 20.0
        self.RESOLUTION = 10  
        self.MAP_W = int(self.MAP_SIZE_M * self.RESOLUTION)
        self.MAP_H = int(self.MAP_SIZE_M * self.RESOLUTION)
        
        # log odds for each cell: 0=unknown, positive =occ, negative =free
        self.LOG_ODDS_MIN = -8.0
        self.LOG_ODDS_MAX = 8.0
        self.L_FREE = -1.0  
        self.L_OCC = 0.5   
        self.OCC_INERTIA = 2.0
        self.DECAY = 0.9997
        self.OCC_CONFIRM = 15 

        # devices/sensors
        self.display = robot.getDevice("display")
        self.lidar = robot.getDevice("LDS-01")
        self.imu = robot.getDevice("inertial unit")
        self.left_enc = robot.getDevice("left wheel sensor")
        self.right_enc = robot.getDevice("right wheel sensor")

        self.lidar.enable(self.TIME_STEP)
        self.imu.enable(self.TIME_STEP)
        self.left_enc.enable(self.TIME_STEP)
        self.right_enc.enable(self.TIME_STEP)
        # Lidar config
        self.lidar_res = self.lidar.getHorizontalResolution()
        self.lidar_fov = self.lidar.getFov()
        self.lidar_min = self.lidar.getMinRange()
        self.lidar_max = self.lidar.getMaxRange()
        self.NO_HIT_THRESH = 0.98 * self.lidar_max

        self.grid = np.zeros((self.MAP_W, self.MAP_H), dtype=np.float32)
        self.map_data = self.grid
        # keep track of occ cells to confirm before plotting
        self.occ_hits = np.zeros((self.MAP_W, self.MAP_H), dtype=np.uint16)

        # intial localization state: position, orientation, and encoder values
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        self.last_l = self.left_enc.getValue()
        self.last_r = self.right_enc.getValue()
        
        # Scan matching parameters
        self.USE_SCAN_MATCHING = True 
        self.scan_match_counter = 0
        self.SCAN_MATCH_INTERVAL = 10  # Run every 10 updates
        self.prev_ranges = None
        
        # Stuck detection 
        self.stuck_counter = 0
        self.STUCK_THRESHOLD = 5 
        
        # Precompute beam angles for efficiency
        self.beam_angles = np.linspace(
            -self.lidar_fov / 2.0,
            self.lidar_fov / 2.0,
            self.lidar_res,
            dtype=np.float32
        )

    # keep log odds withen range (-8 to 8)
    def range(self, v, lo, hi):
        return lo if v < lo else hi if v > hi else v

    def wrap_angle(self, a):
        return math.atan2(math.sin(a), math.cos(a))
        
    # Convert world coordinates to map coordinates
    def world_to_map(self, x, y):
        cx, cy = self.MAP_W // 2, self.MAP_H // 2 # center map
        mx = int(round(y * self.RESOLUTION)) + cx
        my = int(round(-x * self.RESOLUTION)) + cy
        return mx, my
    
    # marking free space from bot to obstacle detected
    def trace_ray(self, x0, y0, x1, y1):
        points = []
        steps = int(max(abs(x1 - x0), abs(y1 - y0))) + 1
        if steps <= 0:
            return points
        dx = (x1 - x0) / steps
        dy = (y1 - y0) / steps
        x, y = x0, y0
        for _ in range(steps):
            points.append((int(round(x)), int(round(y))))
            x += dx
            y += dy
        return points
    
    # convert log odds to probability (0 =free, 1= occupied) 
    def logodds_to_prob(self, value):
        return 1.0 / (1.0 + math.exp(-value))
    
    # SCAN MATCHING:convert lidar ranges to obstacle pints in world cords
    def get_scan_points(self, ranges, x, y, th):
        points = []
        step = max(1, self.lidar_res // 40)  
        
        for i in range(0, self.lidar_res, step):
            r = float(ranges[i])
            if r > 0.05 and r < self.lidar_max * 0.95:
                angle = th + self.beam_angles[i]
                px = x + r * math.cos(angle)
                py = y + r * math.sin(angle)
                points.append((px, py))
        
        return np.array(points) if points else np.array([])

    # SCAN MATCHING...
    def simple_scan_match(self, ranges, x_odom, y_odom, th_odom):
        #check if enough information (map) exists, if not, rely on odometry
        known_cells = np.sum(np.abs(self.grid) > 0.3)
        if known_cells < 200:
            return x_odom, y_odom, th_odom
        
        # Get scan points
        scan_points = self.get_scan_points(ranges, x_odom, y_odom, th_odom)
        if len(scan_points) < 15:
            return x_odom, y_odom, th_odom
        
        # only correct small errors
        best_score = -1.0
        best_dx, best_dy, best_dth = 0.0, 0.0, 0.0
        
        # Search map ±5cm position, ±3 degrees rotation for best match
        for dx in [-0.05, 0.0, 0.05]:
            for dy in [-0.05, 0.0, 0.05]:
                for dth in [-0.05, 0.0, 0.05]:  
                    # score each correction and use the highest score 
                    x_test = x_odom + dx
                    y_test = y_odom + dy
                    th_test = th_odom + dth
                    
                    score = 0.0
                    valid = 0
                    
                    for px, py in scan_points:
                        px_rel = px - x_odom
                        py_rel = py - y_odom
                        cos_dth = math.cos(dth)
                        sin_dth = math.sin(dth)
                        px_rot = px_rel * cos_dth - py_rel * sin_dth + x_test
                        py_rot = px_rel * sin_dth + py_rel * cos_dth + y_test
                        
                        # Check map
                        mx, my = self.world_to_map(px_rot, py_rot)
                        if 0 <= mx < self.MAP_W and 0 <= my < self.MAP_H:
                            occ = self.logodds_to_prob(self.grid[mx, my])
                            score += occ
                            valid += 1
                    
                    if valid > 0:
                        score /= valid
                        if score > best_score:
                            best_score = score
                            best_dx, best_dy, best_dth = dx, dy, dth
        
        # apply if match is good, otherwise keep relying on odometry
        if best_score > 0.5:
            return x_odom + best_dx, y_odom + best_dy, th_odom + best_dth
        
        return x_odom, y_odom, th_odom

    # STUCK DETECTION
    def detect_stuck(self, ranges, d, dth):
        if ranges is None or abs(d) < 0.001:
            return False
        
        # surrounded by very close obstacles check
        very_close = np.sum((ranges > 0.05) & (ranges < 0.12))
        
        if very_close > self.lidar_res * 0.15:
            if self.prev_ranges is not None and abs(d) > 0.003:
                valid_mask = (ranges > 0.05) & (ranges < self.lidar_max * 0.9) & \
                            (self.prev_ranges > 0.05) & (self.prev_ranges < self.lidar_max * 0.9)
                
                if np.sum(valid_mask) > 20:
                    range_change = np.mean(np.abs(ranges[valid_mask] - self.prev_ranges[valid_mask]))
                    
                    if range_change < 0.015:
                        return True
        
        return False

    # pose update with SLAM
    def update_pose(self, ranges=None):
        xz, xc, xv = self.imu.getRollPitchYaw()
        if math.isnan(xv):
            return False

        yaw = -xv

        # compute odometry based on sensors
        l = self.left_enc.getValue()
        r = self.right_enc.getValue()
        dl = (l - self.last_l) * self.WHEEL_RADIUS
        dr = (r - self.last_r) * self.WHEEL_RADIUS
        self.last_l, self.last_r = l, r

        d = 0.5 * (dl + dr)
        dth = (dr - dl) / self.WHEEL_BASE

        # Detect if robot is stuck
        is_stuck = self.detect_stuck(ranges, d, dth)
        
        if is_stuck:
            self.stuck_counter += 1
        else:
            self.stuck_counter = max(0, self.stuck_counter - 1)
        
        # Apply slippage correction
        if self.stuck_counter > self.STUCK_THRESHOLD:
            d = 0.0
            dth = 0.0

        # Dead reckoning (localization) update
        self.th = yaw
        x_odom = self.x + d * math.cos(self.th + 0.5 * dth)
        y_odom = self.y + d * math.sin(self.th + 0.5 * dth)
        th_odom = self.th
        
        # Scan matching correction
        if self.USE_SCAN_MATCHING and ranges is not None:
            self.scan_match_counter += 1
            
            
            if self.scan_match_counter >= self.SCAN_MATCH_INTERVAL:
                x_corrected, y_corrected, th_corrected = self.simple_scan_match(
                    ranges, x_odom, y_odom, th_odom
                )
                
                # Calculate correction
                dx = x_corrected - x_odom
                dy = y_corrected - y_odom
                dth_corr = self.wrap_angle(th_corrected - th_odom)
                correction_dist = math.sqrt(dx**2 + dy**2)
                
                if correction_dist < 0.10:  # Only accept corrections under 10cm
                    # Apply correction
                    blend = 0.85  
                    self.x = x_odom + (1 - blend) * dx
                    self.y = y_odom + (1 - blend) * dy
                    self.th = self.wrap_angle(th_odom + (1 - blend) * dth_corr)
                else:
                    # Reject correction, use odometry
                    self.x = x_odom
                    self.y = y_odom
                    self.th = th_odom
                
                self.scan_match_counter = 0
            else:
                self.x = x_odom
                self.y = y_odom
                self.th = th_odom
        else:
            self.x = x_odom
            self.y = y_odom
            self.th = th_odom
       
        # MAPPING
        # Store ranges
        if ranges is not None:
            self.prev_ranges = ranges.copy()

        if not (math.isfinite(self.x) and math.isfinite(self.y) and math.isfinite(self.th)):
            self.x = self.y = 0.0
            self.th = yaw
            self.stuck_counter = 0
            return False

        return True

    def get_pose(self):
        return self.x, self.y, self.th

    # drawing on map
    def draw_map(self):
        self.display.setColor(0x000000)
        self.display.fillRectangle(0, 0, self.MAP_W, self.MAP_H)

        self.display.setColor(0x0000FF)
        for x in range(0, self.MAP_W):
            for y in range(0, self.MAP_H):
                if self.grid[x, y] > 0.4:
                    self.display.drawPixel(x, y)

        rx, ry = self.world_to_map(self.x, self.y)
        if 0 <= rx < self.MAP_W and 0 <= ry < self.MAP_H:
            if self.stuck_counter > self.STUCK_THRESHOLD:
                self.display.setColor(0xFFFF00)
            else:
                self.display.setColor(0xFF0000)
            self.display.fillRectangle(rx - 1, ry - 1, 2, 2)

    # main 
    def update(self):
        ranges = np.array(self.lidar.getRangeImage(), dtype=np.float32)
        ranges = np.clip(ranges, self.lidar_min, self.lidar_max)
        
        if not self.update_pose(ranges):
            return

        robot_mx, robot_my = self.world_to_map(self.x, self.y)
        
        # Clear around robot only if not already strongly occupied
        for dx in range(-1, 2):
            for dy in range(-1, 2):
                clear_x = robot_mx + dx
                clear_y = robot_my + dy
                if 0 <= clear_x < self.MAP_W and 0 <= clear_y < self.MAP_H:
                    if self.grid[clear_x, clear_y] < 4.0:  
                        self.grid[clear_x, clear_y] = min(self.grid[clear_x, clear_y], -2.0)  
                        self.occ_hits[clear_x, clear_y] = 0

        # Process lidar beams
        beam_step = 3  # Process every 3rd beam 
        for i in range(0, self.lidar_res, beam_step):
            r_val = float(ranges[i])
            if not np.isfinite(r_val) or r_val <= 0.05:
                continue

            beam_angle = (i / (self.lidar_res - 1)) * self.lidar_fov - (self.lidar_fov / 2.0)
            world_angle = self.th + beam_angle
            hit = (r_val < self.NO_HIT_THRESH)

            if hit:
                wx = self.x + r_val * math.cos(world_angle)
                wy = self.y + r_val * math.sin(world_angle)
                mx, my = self.world_to_map(wx, wy)
                
                if mx < 0 or my < 0 or mx >= self.MAP_W or my >= self.MAP_H:
                    continue
                
                # Mark free space along the ray
                ray_cells = self.trace_ray(robot_mx, robot_my, mx, my)
                for j, (cell_x, cell_y) in enumerate(ray_cells):
                    if 0 <= cell_x < self.MAP_W and 0 <= cell_y < self.MAP_H:
                        if j < len(ray_cells) - 1:
                    
                            if self.grid[cell_x, cell_y] < 4.0:  
                                self.grid[cell_x, cell_y] = self.range(
                                    self.grid[cell_x, cell_y] + self.L_FREE,
                                    self.LOG_ODDS_MIN,
                                    self.LOG_ODDS_MAX
                                )
                        else:
                            # Obstacle hit point
                            if self.occ_hits[cell_x, cell_y] < 65535:
                                self.occ_hits[cell_x, cell_y] += 1
                            
                            # Only mark as obstacle after MORE confirmations
                            if self.occ_hits[cell_x, cell_y] >= self.OCC_CONFIRM:
                                self.grid[cell_x, cell_y] = self.range(
                                    self.grid[cell_x, cell_y] + self.L_OCC,
                                    self.LOG_ODDS_MIN,
                                    self.LOG_ODDS_MAX
                                )

        # guard railing for mapping to minimize noise information on map over time
        weak_noise = (self.grid > 0.1) & (self.grid < 2.0) 
        self.grid[weak_noise] *= 0.98  
        
        medium_uncertain = (self.grid >= 2.0) & (self.grid < 4.0)  
        self.grid[medium_uncertain] *= 0.995  
        
       
        
        self.draw_map()
        self.map_data = self.grid





