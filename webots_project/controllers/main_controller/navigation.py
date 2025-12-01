# navigation.py
# Handles robot movement and obstacle avoidance

import math
import numpy as np

# TurtleBot measurements
# From https://emanual.robotis.com/docs/en/platform/turtlebot3/features/
AXLE_LENGTH = 0.160
WHEEL_RADIUS = 0.033

class Navigation:
    def __init__(self, robot, timestep):
        self.robot = robot
        self.timestep = timestep
        
        # Set tolerance for how close to goal
        self.goal_tolerance = 0.20
        
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
        self.right_motor.setVelocity(0.0)
        
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
        self.goal = (1.0, 0)
        self.start = (0.0, 0.0)
        
        # M-line equation
        xs, ys = self.start
        xg, yg = self.goal
        self.a = ys - yg
        self.b = xg - xs
        self.c = xs *yg - xg * ys
        
        #Parameter for how far from line to be considered on the line
        self.mline_tolerance = 0.10
        
        # Normalise M-line coefficients for better distance calculation
        norm = math.sqrt(self.a**2 + self.b**2)
        if norm > 0:
            self.a /= norm
            self.b /= norm
            self.c /= norm
        
        self.state = "GO_TO_GOAL"
        # Where we first hit the obstacle
        self.hit_point = None 
        # How far to be considered obstacle
        self.obs_threshold = 0.25
        self.clearance_threshold = 0.35
        
        # Wall following parameters
        self.follow_side = None
        # Distance from wall
        self.target_distance = 0.15
        self.wall_follow_speed = 2.0
        
        print("init complete")
        
    # Update odometry values from encoder increments
    def update_odometry(self):   
        # Read current wheel rotation
        left_angle = self.left_sensor.getValue()
        right_angle = self.right_sensor.getValue()
        
        # Compute wheel displacements
        dl = (left_angle - self.prev_left_angle) * WHEEL_RADIUS
        dr = (right_angle - self.prev_right_angle) * WHEEL_RADIUS
        
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
    
    # Compute distance from the M-Line and return true if the distance
    # remains in the tolerance
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
    
    def path_clear(self):
        # Get lidar distances from full 360
        ranges = self.lidar.getRangeImage() 
        
        # Get central region in front of robot
        center_start = self.lidar_width // 3
        center_end = 2 * self.lidar_width // 3
        center_ranges = ranges[center_start:center_end]
        
        # Return true if closest obstacle ahead is further than clearance threshold
        return min(center_ranges) > self.clearance_threshold
       
    def wall_follow(self):
        # Get lidar distances from full 360
        ranges = self.lidar.getRangeImage()
        n = self.lidar_width
        
        # Split ranges into regions
        left_range = ranges[n // 4 : n // 2]
        right_range = ranges[n // 2 : 3 * n // 4]
        front_range = ranges[2 * n // 5 : 3 * n // 5]
        
        # Find closest object in each region
        if left_range:
            closest_left = min(left_range)
        else:
            closest_left = self.lidar_max
        if right_range:
            closest_right = min(right_range)
        else:
            closest_right = self.lidar_max
        if front_range:
            closest_front = min(front_range)
        else:
            closest_front = self.lidar_max
        
        # Decide which wall to follow
        if self.follow_side is None or self.hit_point == (self.x, self.y):
            if closest_left < closest_right:
                self.follow_side = "LEFT"
            else:
                self.follow_side = "RIGHT"
        
        # Set wall follow speed
        base_speed = self.wall_follow_speed
        
        # Calculate wheel speeds based on wall
        if self.follow_side == "RIGHT":
            # Find difference between desired wall distance
            # and current wall distance
            error = closest_right - self.target_distance
            turn_correction = error * 3.0
            
            # If there is an obstacle close, avoid it by
            # turning other way
            if closest_front < 0.2:
                v_left = base_speed
                v_right = -0.5 * base_speed
            else:
                v_left = base_speed + turn_correction
                v_right = base_speed - turn_correction
        else:
            # Follow left wall
            error = closest_left - self.target_distance
            turn_correction = error * 3.0
             
            if closest_front < 0.2:
                v_left = -0.5 * base_speed
                v_right = base_speed
            else:
                v_left = base_speed - turn_correction
                v_right = base_speed + turn_correction
         
        v_left = max(-self.max_speed, min(self.max_speed, v_left))
        v_right = max(-self.max_speed, min(self.max_speed, v_right))
         
        return v_left, v_right
     
    def distance_to_goal(self):
        goal_x, goal_y = self.goal
        
        dx = goal_x - self.x
        dy = goal_y - self.y
        return math.sqrt(dx*dx + dy*dy)
    
    def move(self):
        #Update pose
        self.update_odometry()
        
        # Check if we have reached the goal
        if self.distance_to_goal() < self.goal_tolerance:
            self.left_motor.setVelocity(0)
            self.right_motor.setVelocity(0)
            print("Goal reached")
            return
        
        if self.state == "GO_TO_GOAL":
            # If obstacle appears ahead, switch to wall follow state
            if self.obstacle_detected():
                self.state = "WALL_FOLLOW"
                self.hit_point = (self.x, self.y)
                self.follow_side = None
                print("Hit obstacle -> WALL_FOLLOW")
                v_left, v_right = self.wall_follow()
            else:
                # Continue towards goal
                v_left, v_right, rho = self.goto_position(*self.goal)
        
        elif self.state == "WALL_FOLLOW":
            # Compute distance from hit point to goal
            if self.hit_point is not None:
                hit_x, hit_y = self.hit_point
                dx_hit = self.goal[0] - hit_x
                dy_hit = self.goal[1] - hit_y
                distance_hit_to_goal = math.sqrt(dx_hit*dx_hit + dy_hit*dy_hit)
            else:
                distance_hit_to_goal = float('inf')
                
            # Switch back to GO_TO_GOAL if:
            # Path ahead is clear
            # On M-line
            # Closer to goal than hit point
            
            current_distance = self.distance_to_goal()
            
            if self.on_mline() and current_distance < distance_hit_to_goal and self.path_clear():
                print("Back on M-line -> GO_TO_GOAL")
                self.state = "GO_TO_GOAL"
                self.follow_side = None
                self.on_mline_count = 0
                v_left, v_right, rho = self.goto_position(*self.goal)
            else:
                # Continue to follow obstacle
                v_left, v_right = self.wall_follow()
        
        # Set speed
        self.left_motor.setVelocity(v_left)
        self.right_motor.setVelocity(v_right)
        
        print(f"[{self.state}] x={self.x:.3f}, y={self.y:.3f}, theta={self.theta:.3f}")