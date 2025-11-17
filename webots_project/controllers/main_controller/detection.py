# detection.py
# Handles survivor detection using thermal / vision sensors
from controller import Camera 
import numpy as np
import cv2
class Detection:
    def __init__(self, robot):
        self.robot = robot
        self.camera = self.robot.getDevice("rgb_camera")
        self.camera_motor = self.robot.getDevice("camera_motor")
        timestep = int(self.robot.getBasicTimeStep())
        self.camera.enable(timestep) #Updates cameras every interval(Creates a refresh loop)
        print("Detection module initialized")

    def detect(self):
       image = self.camera.getImage()
       self.camera_motor.setVelocity(1.0)
       self.camera_motor.setPosition(float('inf'))
       if image is None:
           print("No camera image yet")
           return []
       width = self.camera.getWidth()
       height = self.camera.getHeight()
       # Reorganises the massive 1 byte buffer to a structured 3d so that it can be understood
       image_array = np.frombuffer(image, np.uint8).reshape((height, width, 4))
        
       # Convert BGRA → BGR for OpenCV as OPENCV doesn't need the alpha(transparency)
       frame = cv2.cvtColor(image_array, cv2.COLOR_BGRA2BGR)
       hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
       lower_warm1 = np.array([0, 160, 120])
       upper_warm1 = np.array([20, 255, 255])
       lower_warm2 = np.array([170, 120, 70])
       upper_warm2 = np.array([180, 255, 255])
       mask_warm = cv2.inRange(hsv, lower_warm1, upper_warm1) | cv2.inRange(hsv, lower_warm2, upper_warm2)
       contours, _ = cv2.findContours(mask_warm, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
       red_detected = False
       if len(contours) > 0:
        c = max(contours, key=cv2.contourArea)
        if cv2.contourArea(c) > 200: 
            red_detected = True
        #robot_node = self.robot.getSelf()
        print("Human Found")
       cv2.imshow("TurtleBot RGB Camera", frame)
       cv2.waitKey(1)
       cv2.imshow("Warm Colors", mask_warm)  # show color mask
       cv2.waitKey(1) 
       return []
    