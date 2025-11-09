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
        #self.k_rho = 0.2
        self.k_rho = 5.0
        self.k_alpha = 10
        self.k_beta = -0.05
        
        # Motor limits
        self.max_speed = 6.28
        
        print("init complete")
        
    # Update odometry values from encoder increments
    def update_odometry(self):   
        # Read current wheel rotation
        left_angle = self.left_sensor.getValue()
        right_angle = self.right_sensor.getValue()
        
        dl = (left_angle - self.prev_left_angle) * WHEEL_RADIUS
        dr = (right_angle - self.prev_right_angle) *WHEEL_RADIUS
        
        self.prev_left_angle = left_angle
        self.prev_right_angle = right_angle
               
        dc = (dl + dr) / 2.0
        dtheta = (dr - dl) / AXLE_LENGTH
        
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
    
    def move(self):    
        #Update pose
        self.update_odometry()
        
        v_left, v_right, rpho = self.goto_position(0.2,0.2)
        
        self.left_motor.setVelocity(v_left)
        self.right_motor.setVelocity(v_right)
        
        print(f"Position -> x: {self.x:.3f}, y: {self.y:.3f}, theta: {self.theta:.3f}")