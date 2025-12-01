# detection.py
# Handles survivor detection using thermal / vision sensors
from controller import Camera 
import numpy as np
import cv2
import math
class Detection:
    def __init__(self, robot):
        self.nav = None
        self.robot = robot
        self.camera = self.robot.getDevice("rgb_camera")
        self.camera_motor = self.robot.getDevice("camera_motor")
        self.camera_sensor = self.camera_motor.getPositionSensor()
        timestep = int(self.robot.getBasicTimeStep())
        self.camera_sensor.enable(timestep)
        self.camera.enable(timestep) #Updates cameras every interval(Creates a refresh loop)
        self.start_angle = None
        self.scan_done = False
        self.detected_angles = []
        self.final_distances = []
        self.humans = {}
        self.next_human_id = 0
        self.passed_zero = False
        print("Detection module initialized")

    def reset_scan(self):
        self.scan_done = False
        self.start_angle = None
        self.humans = {}
        self.next_human_id = 0
        self.detected_angles = []
        self.final_distances = []
        
    def detect(self):
       print(self.scan_done)
       self.nav.pause()
       if self.start_angle is None:
           self.start_angle = self.camera_sensor.getValue()
       image = self.camera.getImage()
       if not self.scan_done: 
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
       valid_contours = [c for c in contours if cv2.contourArea(c) > 200]
       FOV = 1.047  
       IMAGE_WIDTH = 640
       PIXEL_ANGLE = FOV / IMAGE_WIDTH
       HUMAN_WIDTH = 0.2
       IMAGE_CENTER = 320
       CENTER_TOLERANCE = 40
       seen_ids = []
       camera_angle = self.camera_motor.getPositionSensor().getValue()
       angle = camera_angle % ( 2 * math.pi)
       for c in valid_contours:
           x, y, w, h = cv2.boundingRect(c)
           cx = x + w/2 # center x of contour 
           object_angle = w * PIXEL_ANGLE
           if w > 0:
               distance = HUMAN_WIDTH / (2 * math.tan(object_angle / 2)) #Formula to work out distance Found online
           matched_id = None 
           for hid, data in self.humans.items(): 
               if abs(cx - data["last_x"]) < 60: # ~60 px tolerance 
                   matched_id = hid 
           if matched_id is None and distance > 0.25: 
               matched_id = self.next_human_id 
               self.humans[matched_id] = {  #Dictionary of humans at the distances and angle we save them at
               "last_x": cx, 
               "distances": [],
               "angle_saved": False,
               } 
               self.next_human_id += 1
           human = self.humans[matched_id]
           human["last_x"] = cx
           if "min_distance" not in human:
               human["min_distance"] = distance
           else:
               human["min_distance"] = min(human["min_distance"], distance)  #Only save the lowest distance so check between us and our current self
           if not human["angle_saved"]:
               if abs(cx - IMAGE_CENTER) <= CENTER_TOLERANCE:  #Checks when the object is in the middle so that we can get the best theta and angle  
                   self.detected_angles.append(angle)
                   human["angle_saved"] = True
                   self.final_distances.append(human["min_distance"])
       cv2.imshow("TurtleBot RGB Camera", frame)
       cv2.waitKey(1)
       cv2.imshow("Warm Colors", mask_warm)  # show color mask
       cv2.waitKey(1) 
       if not self.scan_done:
           current = self.camera_sensor.getValue()
           if abs(current - self.start_angle) >= 2 * math.pi:   # Check for one full rotation 
               self.camera_motor.setVelocity(0)
               self.scan_done = True
               print("Scan finished.")
               coords = self.calculate_coordinates(self.detected_angles,self.final_distances)
               print(coords[0] )
               closest_human = list(coords[0])
               if coords:
                   self.nav.goal[0] = closest_human[0]
                   self.nav.goal[1] = closest_human[1] 
                   print("Goal inside main loop:", self.nav.goal)    
       else:
           self.nav.resume()
       return []
    def calculate_coordinates(self,anglelist,distancelist):
        coords = []
        print(distancelist)
        print(anglelist)
        paired = list(zip(anglelist, distancelist))
        paired.sort(key=lambda x: x[1])
        for angle, distance in paired:
            x = self.nav.x + distance * math.cos(self.nav.theta + angle)
            z = self.nav.y + distance * math.sin(self.nav.theta + angle)
            coords.append((x, z))
        return coords
        