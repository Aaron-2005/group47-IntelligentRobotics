# navigation.py
# Handles robot movement and obstacle avoidance

import math

# Global defines
AXLE_LENGTH = 0.160
WHEEL_RADIUS = 0.033

class Navigation:
    def __init__(self, robot, timestep):
        self.robot = robot
        self.timestep = timestep
        
        # Get motor devices
        self.left_motor = robot.getDevice('left wheel motor')
        self.right_motor = robot.getDevice('right wheel motor')
        
        # Get Lidar
        self.lidar = robot.getDevice('LDS-01')
        self.lidar.enable(timestep)
        self.lidar_max = self.lidar.getMaxRange()
        self.lidar_width = self.lidar.getHorizontalResolution()
        
        # Set velocity
        self.left_motor.setPosition(float('inf'))
        self.right_motor.setPosition(float('inf'))
        self.left_motor.setVelocity(0.0)
        self.left_motor.setVelocity(0.0)
        
        # Get encoders
        self.left_sensor = robot.getDevice('left wheel sensor')
        self.right_sensor = robot.getDevice('right wheel sensor')
        self.left_sensor.enable(timestep)
        self.right_sensor.enable(timestep)
        
        # Store previous angles
        self.prev_left_angle = 0.0
        self.prev_right_angle = 0.0
        
        # Initialise robot pose
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        
        # Controller values
        self.k_rho = 5.0
        self.k_alpha = 10
        self.k_beta = -0.05
        
        # Motor limits
        self.max_speed = 6.28
        
        # Set goal and start position
        self.goal = (0.5, 0.5)
        self.start = (0.0, 0.0)
        
        # M-line equation
        xs, ys = self.start
        xg, yg = self.goal
        self.a = ys - yg
        self.b = xg - xs
        self.c = xs *yg - xg * ys
        self.mline_tolerance = 0.05 #Parameter for how far from line
        
        self.state = "GO_TO_GOAL"
        self.hit_point = None
        self.obs_threshold = 0.25
        
        print("init complete")
        
    # Update odometry values from encoder increments
    def update_odometry(self):   
        # Read current wheel rotation
        left_angle = self.left_sensor.getValue()
        right_angle = self.right_sensor.getValue()
        
        # Compute wheel displacements
        dl = (left_angle - self.prev_left_angle) * WHEEL_RADIUS
        dr = (right_angle - self.prev_right_angle) *WHEEL_RADIUS
        
        # Store current angles for next step
        self.prev_left_angle = left_angle
        self.prev_right_angle = right_angle
               
        # Compute linear and angular displacement
        dc = (dl + dr) / 2.0
        dtheta = (dr - dl) / AXLE_LENGTH
        
        # Update pose
        self.x += dc * math.cos(self.theta + dtheta / 2.0)
        self.y += dc * math.sin(self.theta + dtheta / 2.0)
        self.theta += dtheta
        
        # Normalise angle
        self.theta = (self.theta + math.pi) % (2 * math.pi) - math.pi
        
    # Compute wheel speeds 
    def goto_position(self, x_goal, y_goal, goal_theta=0):
        dx = x_goal - self.x
        dy = y_goal - self.y
        
        rho = math.sqrt(dx*dx + dy*dy)
        alpha = math.atan2(dy, dx) - self.theta
        beta = -self.theta - alpha + goal_theta
        
        # Normalise angles
        alpha = (alpha + math.pi) % (2*math.pi) - math.pi
        beta = (beta + math.pi) % (2*math.pi) - math.pi
        
        v = self.k_rho * rho
        w = self.k_alpha * alpha + self.k_beta * beta
        
        v_left = v - 0.5 * w
        v_right = v + 0.5 * w
        
        v_left = max(-self.max_speed, min(self.max_speed, v_left))
        v_right = max(-self.max_speed, min(self.max_speed, v_right))
        
        return v_left, v_right, rho 
    
    def on_mline(self):
        distance_to_line = abs(self.a*self.x + self.b*self.y + self.c)
        return distance_to_line < self.mline_tolerance
    
    def obstacle_detected(self):
        # Get lidar distance values
        ranges = self.lidar.getRangeImage()
        
        # Get central region
        one_third = int(self.lidar_width / 3)
        two_third = int(2 * self.lidar_width / 3)
        
        # Get lidar values from this region
        center_ranges = []
        for i in range(one_third, two_third):
            center_ranges.append(ranges[i])
            
        # Find the closest detected object in the central region
        closest_distance = min(center_ranges)
        
        # If it's closer than the threshold, its an obstacle
        if closest_distance < self.obs_threshold:
            return True
        else:
            return False
        
    def wall_follow(self):
        # Get lidar distance values from full 360
        ranges = self.lidar.getRangeImage()
        midpoint = int(self.lidar_width / 2)
        
        # Split ranges into halves
        left_range = []
        for i in range(0, midpoint):
            left_range.append(ranges[i])
        
        right_range = []
        for i in range(midpoint, self.lidar_width):
            right_range.append(ranges[i])
            
        # Find closest object on each side
        closest_left = min(left_range)
        closest_right = min(right_range)
        
        # Decide where to turn 
        if closest_left < closest_right:
            # Obstacle is closer on left so turn right 
            return 3.0, 1.5
        else:
            # Obstacle is closer on right so turn left
            return 1.5, 3.0
     
    def distance_to_goal(self):
        goal_x, goal_y = self.goal
        
        dx = goal_x - self.x
        dy = goal_y - self.y
        return math.sqrt(dx*dx + dy*dy)
    
    def move(self):    
        #Update pose
        self.update_odometry()
        
        if self.state == "GO_TO_GOAL":
            # If obstacle appears ahead, switch to wall follow state
            if self.obstacle_detected():
                self.state = "WALL_FOLLOW"
                self.hit_point = (self.x, self.y)
                print("Hit obstacle -> WALL_FOLLOW")
                v_left, v_right = self.wall_follow()
            else:
                # Continue towards goal
                v_left, v_right, rpho = self.goto_position(*self.goal)
        
        elif self.state == "WALL_FOLLOW":
            # Compute distance from hit point to goal
            if self.hit_point is not None:
                hit_x, hit_y = self.hit_point
                dx_hit = self.goal[0] - hit_x
                dy_hit = self.goal[1] - hit_y
                distance_hit_to_goal = math.sqrt(dx_hit*dx_hit + dy_hit*dy_hit)
            else:
                distance_hit_to_goal = float('inf')
                
            # Check is robot is on M-line and closer to the goal
            if self.on_mline() and self.distance_to_goal() < distance_hit_to_goal:
                print("Back on M-line -> GO_TO_GOAL")
                self.state = "GO_TO_GOAL"
                v_left, v_right, rpho = self.goto_position(*self.goal)
            else:
                # Continue to follow obstacle
                v_left, v_right = self.wall_follow()
        
        # Set speed
        self.left_motor.setVelocity(v_left)
        self.right_motor.setVelocity(v_right)
        
        print(f"[{self.state}] x={self.x:.3f}, y={self.y:.3f}, theta={self.theta:.3f}")