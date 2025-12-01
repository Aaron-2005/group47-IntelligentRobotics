from controller import Display, Lidar, InertialUnit, PositionSensor
import math
import numpy as np


class Mapping:
    def __init__(self, robot):
        self.robot = robot
        self.TIME_STEP = int(robot.getBasicTimeStep())

        # related to nav? to be delet
        self.WHEEL_RADIUS = 0.033
        self.WHEEL_BASE = 0.160

      
        self.MAP_SIZE_M = 20.0
        self.RESOLUTION = 10  # cells per meter (0.1 m)
        self.MAP_W = int(self.MAP_SIZE_M * self.RESOLUTION)
        self.MAP_H = int(self.MAP_SIZE_M * self.RESOLUTION)

       
        self.LOG_ODDS_MIN = -8.0
        self.LOG_ODDS_MAX = 8.0
        self.L_FREE = -0.7
        self.L_OCC = 0.6
        self.OCC_INERTIA = 2.0
        self.DECAY = 0.9997
        self.OCC_CONFIRM = 3

        # devices
        self.display = robot.getDevice("display")
        self.lidar = robot.getDevice("LDS-01")
        self.imu = robot.getDevice("inertial unit")
        self.left_enc = robot.getDevice("left wheel sensor")
        self.right_enc = robot.getDevice("right wheel sensor")

       
        self.lidar.enable(self.TIME_STEP)
        self.imu.enable(self.TIME_STEP)
        self.left_enc.enable(self.TIME_STEP)
        self.right_enc.enable(self.TIME_STEP)

        # LiDAR P
        self.lidar_res = self.lidar.getHorizontalResolution()
        self.lidar_fov = self.lidar.getFov()
        self.lidar_min = self.lidar.getMinRange()
        self.lidar_max = self.lidar.getMaxRange()
        self.NO_HIT_THRESH = 0.98 * self.lidar_max

        # Map Display
        self.grid = np.zeros((self.MAP_W, self.MAP_H), dtype=np.float32)
        self.map_data = self.grid
        self.occ_hits = np.zeros((self.MAP_W, self.MAP_H), dtype=np.uint16)

        # intial position
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        self.last_l = self.left_enc.getValue()
        self.last_r = self.right_enc.getValue()
        

    # help fun "to be modif"=======
    def clamp(self, v, lo, hi):
        return lo if v < lo else hi if v > hi else v

    def wrap_angle(self, a):
        return math.atan2(math.sin(a), math.cos(a))

    def world_to_map(self, x, y):
        cx, cy = self.MAP_W // 2, self.MAP_H // 2
        mx = int(round(x * self.RESOLUTION)) + cx
        my = int(round(y * self.RESOLUTION)) + cy
        return mx, my

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

    # draw on map ======
    def draw_map(self):
        self.display.setColor(0x000000)  
        self.display.fillRectangle(0, 0, self.MAP_W, self.MAP_H)

        self.display.setColor(0x0000FF) 
        step = 1
        for x in range(0, self.MAP_W, step):
            for y in range(0, self.MAP_H, step):
                if self.grid[x, y] > 0.4:
                    self.display.drawPixel(x, y)

        # robot marker  "to be modif"
        rx, ry = self.world_to_map(self.x, self.y)
        if 0 <= rx < self.MAP_W and 0 <= ry < self.MAP_H:
            self.display.setColor(0xFF0000)
            self.display.fillRectangle(rx - 2, ry - 2, 4, 4)

    def update(self):
        roll, pitch, yaw = self.imu.getRollPitchYaw()
        if math.isnan(yaw):
            print("IMU not ready == skip")
            return
        self.th = -yaw

        # odometry "to be modif" ======
        l = self.left_enc.getValue()
        r = self.right_enc.getValue()
        dl = (l - self.last_l) * self.WHEEL_RADIUS
        dr = (r - self.last_r) * self.WHEEL_RADIUS
        self.last_l, self.last_r = l, r
        d = 0.5 * (dl + dr)
        dth = (dr - dl) / self.WHEEL_BASE

        self.x += d * math.cos(self.th + 0.5 * dth)
        self.y += d * math.sin(self.th + 0.5 * dth)

        if not math.isfinite(self.x) or not math.isfinite(self.y) or not math.isfinite(self.th):
            self.x = self.y = self.th = 0.0
            return

        ranges = np.array(self.lidar.getRangeImage(), dtype=np.float32)
        ranges = np.clip(ranges, self.lidar_min, self.lidar_max)
        rx, ry = self.world_to_map(self.x, self.y)

        for i in range(self.lidar_res):
            r_val = float(ranges[i])
            if not np.isfinite(r_val) or r_val <= 0.05:
                continue

            beam_angle = (i / (self.lidar_res - 1)) * self.lidar_fov - (self.lidar_fov / 2.0)
            world_angle = self.th + beam_angle
            hit = (r_val < self.NO_HIT_THRESH)
            r_use = r_val if hit else self.lidar_max

            wx = self.x + r_use * math.cos(world_angle)
            wy = self.y + r_use * math.sin(world_angle)
            mx, my = self.world_to_map(wx, wy)
            if mx < 0 or my < 0 or mx >= self.MAP_W or my >= self.MAP_H:
                continue

            if hit:
                self.occ_hits[mx, my] = min(65535, self.occ_hits[mx, my] + 1)
                if self.occ_hits[mx, my] >= self.OCC_CONFIRM:
                    self.grid[mx, my] = self.clamp(self.grid[mx, my] + self.L_OCC,
                                                   self.LOG_ODDS_MIN, self.LOG_ODDS_MAX)

        self.grid *= self.DECAY
        self.draw_map()
        self.map_data = self.grid
from controller import Display, Lidar, InertialUnit, PositionSensor
import math
import numpy as np


class Mapping:
    def __init__(self, robot):
        self.robot = robot
        self.TIME_STEP = int(robot.getBasicTimeStep())

        
        self.WHEEL_RADIUS = 0.033
        self.WHEEL_BASE = 0.160

        # map 
        self.MAP_SIZE_M = 20.0
        self.RESOLUTION = 10  # cells per meter (0.1 m)
        self.MAP_W = int(self.MAP_SIZE_M * self.RESOLUTION)
        self.MAP_H = int(self.MAP_SIZE_M * self.RESOLUTION)

        self.LOG_ODDS_MIN = -8.0
        self.LOG_ODDS_MAX = 8.0
        self.L_FREE = -0.7
        self.L_OCC = 0.6
        self.OCC_INERTIA = 2.0
        self.DECAY = 0.9997
        self.OCC_CONFIRM = 3

        # devices
        self.display = robot.getDevice("display")
        self.lidar = robot.getDevice("LDS-01")
        self.imu = robot.getDevice("inertial unit")
        self.left_enc = robot.getDevice("left wheel sensor")
        self.right_enc = robot.getDevice("right wheel sensor")

        self.lidar.enable(self.TIME_STEP)
        self.imu.enable(self.TIME_STEP)
        self.left_enc.enable(self.TIME_STEP)
        self.right_enc.enable(self.TIME_STEP)

        self.lidar_res = self.lidar.getHorizontalResolution()
        self.lidar_fov = self.lidar.getFov()
        self.lidar_min = self.lidar.getMinRange()
        self.lidar_max = self.lidar.getMaxRange()
        self.NO_HIT_THRESH = 0.98 * self.lidar_max

        self.grid = np.zeros((self.MAP_W, self.MAP_H), dtype=np.float32)
        self.map_data = self.grid
        self.occ_hits = np.zeros((self.MAP_W, self.MAP_H), dtype=np.uint16)

        # pose for local
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        self.last_l = self.left_enc.getValue()
        self.last_r = self.right_enc.getValue()

    def clamp(self, v, lo, hi):
        return lo if v < lo else hi if v > hi else v

    def wrap_angle(self, a):
        return math.atan2(math.sin(a), math.cos(a))

    def world_to_map(self, x, y):
        cx, cy = self.MAP_W // 2, self.MAP_H // 2
        mx = int(round(x * self.RESOLUTION)) + cx
        my = int(round(y * self.RESOLUTION)) + cy
        return mx, my

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

    # pose update for local
    def update_pose(self):
        roll, pitch, yaw = self.imu.getRollPitchYaw()
        if math.isnan(yaw):
            print("IMU not ready -> skip localization")
            return False

        self.th = -yaw

        # odometry
        l = self.left_enc.getValue()
        r = self.right_enc.getValue()
        dl = (l - self.last_l) * self.WHEEL_RADIUS
        dr = (r - self.last_r) * self.WHEEL_RADIUS
        self.last_l, self.last_r = l, r

        d = 0.5 * (dl + dr)
        dth = (dr - dl) / self.WHEEL_BASE  # this is odometry heading change

        self.x += d * math.cos(self.th + 0.5 * dth)
        self.y += d * math.sin(self.th + 0.5 * dth)

        if not (math.isfinite(self.x) and math.isfinite(self.y) and math.isfinite(self.th)):
            print("Pose became invalid, resetting.")
            self.x = self.y = self.th = 0.0
            return False

        return True

    def get_pose(self):
        return self.x, self.y, self.th

    # drawing on map
    def draw_map(self):
        self.display.setColor(0x000000)
        self.display.fillRectangle(0, 0, self.MAP_W, self.MAP_H)

        self.display.setColor(0x0000FF)
        step = 1
        for x in range(0, self.MAP_W, step):
            for y in range(0, self.MAP_H, step):
                if self.grid[x, y] > 0.4:
                    self.display.drawPixel(x, y)

        rx, ry = self.world_to_map(self.x, self.y)
        if 0 <= rx < self.MAP_W and 0 <= ry < self.MAP_H:
            self.display.setColor(0xFF0000)
            self.display.fillRectangle(rx - 2, ry - 2, 4, 4)

    # main update
    def update(self):
        if not self.update_pose():
            return

        ranges = np.array(self.lidar.getRangeImage(), dtype=np.float32)
        ranges = np.clip(ranges, self.lidar_min, self.lidar_max)

        for i in range(self.lidar_res):
            r_val = float(ranges[i])
            if not np.isfinite(r_val) or r_val <= 0.05:
                continue

            beam_angle = (i / (self.lidar_res - 1)) * self.lidar_fov - (self.lidar_fov / 2.0)
            world_angle = self.th + beam_angle
            hit = (r_val < self.NO_HIT_THRESH)
            r_use = r_val if hit else self.lidar_max

            wx = self.x + r_use * math.cos(world_angle)
            wy = self.y + r_use * math.sin(world_angle)
            mx, my = self.world_to_map(wx, wy)
            if mx < 0 or my < 0 or mx >= self.MAP_W or my >= self.MAP_H:
                continue

            if hit:
                self.occ_hits[mx, my] = min(65535, self.occ_hits[mx, my] + 1)
                if self.occ_hits[mx, my] >= self.OCC_CONFIRM:
                    self.grid[mx, my] = self.clamp(
                        self.grid[mx, my] + self.L_OCC,
                        self.LOG_ODDS_MIN,
                        self.LOG_ODDS_MAX
                    )

        self.grid *= self.DECAY
        self.draw_map()
        self.map_data = self.grid
