# detection.py
# Handles survivor detection using thermal / vision sensors
from controller import Camera 
import numpy as np
import cv2
import math

coords = [] 

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
        self.past_coordinates = []            
        self.camera_height = 0.36         
        self.target_height = 0.1   
        self.camera_pitch = 0.15       
        self.h_fov = self.camera.getFov()  
        self.image_width = self.camera.getWidth()
        self.image_height = self.camera.getHeight()
        self.image_center_x = self.image_width / 2.0
        self.image_center_y = self.image_height / 2.0
        self.all_human_reached = False
# compute vertical FOV from aspect ratio:
        self.v_fov = 2.0 * math.atan(math.tan(self.h_fov / 2.0) * (self.image_height / self.image_width))
        self.pixel_angle_horizontal = self.h_fov / self.image_width
        self.pixel_angle_vertical   = self.v_fov / self.image_height
        print("Detection module initialized")

    def reset_scan(self):
        self.scan_done = False
        self.start_angle = None
        self.humans = {}
        self.next_human_id = 0
        self.detected_angles = []
        self.final_distances = []
        
    def detect(self):
       if self.start_angle is None:
           self.start_angle = self.camera_sensor.getValue()
       image = self.camera.getImage()
       if not self.scan_done: 
           self.camera_motor.setVelocity(0.5)
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
       seen_ids = []
       camera_angle = self.camera_motor.getPositionSensor().getValue()
       for c in valid_contours:
           x, y, w, h = cv2.boundingRect(c)
           cx = x + w/2 # center x of contour 
           cy = y + h/2
           #Horizontal
           pixel_offset_x = (cx - self.image_center_x)
           object_angle = pixel_offset_x * self.pixel_angle_horizontal          # angle of object relative to camera forward
           true_angle = (camera_angle + object_angle + math.pi) % (2 * math.pi) - math.pi
           # global angle to the object
           #Veritcal
           cy = y + h/2
           pixel_offset_y = cy - self.image_center_y
           vertical_angle_pixel = pixel_offset_y * self.pixel_angle_vertical
           vertical_baseline = self.camera_height - self.target_height
           if vertical_baseline <= 0:      # object appears above horizon
               continue 
           vertical_angle_total = self.camera_pitch + vertical_angle_pixel
           if vertical_angle_total <= 0:
               continue
           distance = vertical_baseline / math.tan(vertical_angle_total)
           if distance <= 0:      # object appears above horizon
               continue   
           matched_id = None 
           for hid, data in self.humans.items(): 
               if abs(cx - data["last_x"]) < 60: # ~60 px tolerance 
                   matched_id = hid 
           if matched_id is None and distance > 0.25: 
               matched_id = self.next_human_id 
               self.humans[matched_id] = {
                   "last_x": cx,
                   "angle_samples": [],
                   "distance_samples": [],
                   "angle_saved": False,
               }
               self.next_human_id += 1
           human = self.humans[matched_id]
           human["last_x"] = cx
           if not human["angle_saved"]:               
               if abs(cx - self.image_center_x) <= 40:  #Checks when the object is in the middle so that we can get the best theta and angle  
                   human["angle_samples"].append(true_angle)
                   human["distance_samples"].append(distance)
                   if len(human["angle_samples"]) >= 6:
                       avg_angle = sum(human["angle_samples"]) / len(human["angle_samples"])
                       avg_dist  = sum(human["distance_samples"]) / len(human["distance_samples"])
                       self.detected_angles.append(avg_angle)
                       self.final_distances.append(avg_dist)
                       human["angle_saved"] = True
       cv2.imshow("Warm Colors", mask_warm)  # show color mask
       cv2.waitKey(1) 
       if not self.scan_done:
           self.nav.pause()
           current = self.camera_sensor.getValue()
           if abs(current - self.start_angle) >= 2 * math.pi:   # Check for one full rotation 
               self.camera_motor.setVelocity(0)
               self.scan_done = True
               print("Scan finished.")
               print("distance:", self.final_distances, "angle:", self.detected_angles)
               global coords
               coords = self.calculate_coordinates(self.detected_angles,self.final_distances)
               print(coords)
               if coords:
                   closest_human = list(coords[0])
                   self.past_coordinates.append(coords[0])
                   self.nav.reset(new_goal=(closest_human[0], closest_human[1]))
               else:
                   self.all_human_reached = True
                   print("All humans reached")  
       else:       
           self.nav.resume()
           coverage = np.sum(mask_warm > 0) / (self.image_width * self.image_height)
           if coverage >= 0.15:
               self.nav.goalreached = True
       return []
    def calculate_coordinates(self, anglelist, distancelist):
        coords = []
        paired = list(zip(anglelist, distancelist))
        paired.sort(key=lambda x: x[1])  # nearest first
        for angle, distance in paired:
            x = self.nav.x + distance * math.cos(self.nav.theta + angle)
            z = self.nav.y + distance * math.sin(self.nav.theta + angle)
            too_close = False
            for past_x, past_z in self.past_coordinates:
                dx = x - past_x
                dz = z - past_z
                if math.sqrt(dx*dx + dz*dz) < 0.5:   
                    too_close = True
                    break
    
            if too_close:
                continue 
            coords.append((x, z))
    
        return coords
    
    
        