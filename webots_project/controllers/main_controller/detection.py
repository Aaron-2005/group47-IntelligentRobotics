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

        self.camera = self.robot.getDevice("rgb_camera")     # RGB camera device
        self.camera_motor = self.robot.getDevice("camera_motor")  # Camera rotation motor
        self.camera_sensor = self.camera_motor.getPositionSensor()  # Tracks camera angle

        timestep = int(self.robot.getBasicTimeStep())        
        self.camera_sensor.enable(timestep)                  
        self.camera.enable(timestep)                         

        self.start_angle = None              # The angle when scan starts
        self.scan_done = False               # Checks whether a full rotation is completed
        self.detected_angles = []            # Stored final angles of humans
        self.final_distances = []            # Stored final distances of humans

        self.humans = {}                     # Tracked humans + sample data
        self.next_human_id = 0              
        self.past_coordinates = []           # Previously visited human coords

        self.camera_height = 0.36            # The height of the camera
        self.target_height = 0.1             # Estimated height of warm object put as the center of the object
        self.camera_pitch = 0.15             # Downward tilt angle in radians

        self.h_fov = self.camera.getFov()    # Horizontal field of view
        self.image_width = self.camera.getWidth()     # Camera resolution width
        self.image_height = self.camera.getHeight()   # Camera resolution height

        self.image_center_x = self.image_width / 2    # Horizontal mid pixel
        self.image_center_y = self.image_height / 2   # Vertical mid pixel

        self.v_fov = 2.0 * math.atan(                 
            math.tan(self.h_fov / 2) * (self.image_height / self.image_width)
        )

        self.pixel_angle_horizontal = self.h_fov / self.image_width   
        self.pixel_angle_vertical = self.v_fov / self.image_height    

        self.all_human_reached = False     

        print("Detection module initialized")

    def reset_scan(self):
        # Restarts the detections reinitalises all the variable
        self.scan_done = False
        self.start_angle = None
        self.humans = {}
        self.next_human_id = 0
        self.detected_angles = []
        self.final_distances = []

    def capture_frame(self):
        # Reads the camera frame and converts it into an OpenCV BGR image
        image = self.camera.getImage()
        if image is None:
            print("No camera image yet")
            return None

        # Coverts the webots buffer and puts it as a NumPy Image
        arr = np.frombuffer(image, np.uint8).reshape(
            (self.image_height, self.image_width, 4)
        )
        frame = cv2.cvtColor(arr, cv2.COLOR_BGRA2BGR)
        return frame

    def detect_warm_targets(self, frame):
        # Converts the frame to HSV then extracts the contour which is the red colored regions
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Warm Color Threshold -> Between light red to dark red
        lower_warm1 = np.array([0, 160, 120])
        upper_warm1 = np.array([20, 255, 255])
        lower_warm2 = np.array([170, 120, 70])
        upper_warm2 = np.array([180, 255, 255])

        mask = (
            cv2.inRange(hsv, lower_warm1, upper_warm1)
            | cv2.inRange(hsv, lower_warm2, upper_warm2)
        )

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Filters out noise
        valid_contours = [c for c in contours if cv2.contourArea(c) > 200]

        cv2.imshow("Warm Colors", mask)
        cv2.waitKey(1)

        return valid_contours, mask

    def process_contours(self, contours):
        # Takes contours calculates angles, distances and sample captures when the target is in the center
        camera_angle = self.camera_motor.getPositionSensor().getValue()

        for c in contours:
            x, y, w, h = cv2.boundingRect(c)
            cx = x + w / 2
            cy = y + h / 2

            pixel_offset_x = cx - self.image_center_x
            object_angle = pixel_offset_x * self.pixel_angle_horizontal
            true_angle = (camera_angle + object_angle + math.pi) % (2 * math.pi) - math.pi

            pixel_offset_y = cy - self.image_center_y
            vertical_angle_pixel = pixel_offset_y * self.pixel_angle_vertical

            vertical_baseline = self.camera_height - self.target_height
            if vertical_baseline <= 0:
                continue

            vertical_angle_total = self.camera_pitch + vertical_angle_pixel
            if vertical_angle_total <= 0:
                continue

            distance = vertical_baseline / math.tan(vertical_angle_total)
            if distance <= 0:
                continue

            matched_id = None
            for hid, data in self.humans.items():
                if abs(cx - data["last_x"]) < 60:
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

            # Only stores humans once they are in the center
            if not human["angle_saved"] and abs(cx - self.image_center_x) <= 40:
                human["angle_samples"].append(true_angle)
                human["distance_samples"].append(distance)

                if len(human["angle_samples"]) >= 6:
                    avg_angle = sum(human["angle_samples"]) / len(
                        human["angle_samples"]
                    )
                    avg_dist = sum(human["distance_samples"]) / len(
                        human["distance_samples"]
                    )

                    self.detected_angles.append(avg_angle)
                    self.final_distances.append(avg_dist)
                    human["angle_saved"] = True

    def handle_scan_completion(self):
        # Checks whether the camera has gone 360 and if it does it stops the scan continues navigation and works out the goal coordinates
        if not self.scan_done:
            self.nav.pause()

            current_angle = self.camera_sensor.getValue()
            if self.start_angle is None:
                self.start_angle = current_angle

            if abs(current_angle - self.start_angle) >= 2 * math.pi:
                self.camera_motor.setVelocity(0)
                self.scan_done = True

                print("Scan finished.")
                print("distance:", self.final_distances, "angle:", self.detected_angles)

                global coords
                coords = self.calculate_coordinates(
                    self.detected_angles, self.final_distances
                )
                print(coords)

                if coords:
                    closest_x, closest_z = coords[0]
                    self.past_coordinates.append((closest_x, closest_z))
                    self.nav.reset(new_goal=(closest_x, closest_z))
                else:
                    self.all_human_reached = True
                    print("All humans reached")

        else:
            self.nav.resume()

    def calculate_coordinates(self, anglelist, distancelist):
        coords = []
        paired = list(zip(anglelist, distancelist))
        paired.sort(key=lambda x: x[1])

        for angle, distance in paired:
            x = self.nav.x + distance * math.cos(self.nav.theta + angle)
            z = self.nav.y + distance * math.sin(self.nav.theta + angle)

            too_close = False
            for px, pz in self.past_coordinates:
                if math.hypot(x - px, z - pz) < 0.5:
                    too_close = True
                    break

            if not too_close:
                coords.append((x, z))

        return coords

    def detect(self):
        if self.start_angle is None:
            self.start_angle = self.camera_sensor.getValue()
    
        if not self.scan_done:
            self.camera_motor.setVelocity(0.5)
            self.camera_motor.setPosition(float('inf'))
    
        frame = self.capture_frame()
        if frame is None:
            return []

        contours, mask = self.detect_warm_targets(frame)
        self.process_contours(contours)
        self.handle_scan_completion()
        if self.scan_done:
            coverage = np.sum(mask > 0) / (self.image_width * self.image_height)
            if coverage >= 0.15:
                self.nav.goalreached = True

        return []

        if self.scan_done:
            coverage = np.sum(mask > 0) / (self.image_width * self.image_height)
            if coverage >= 0.15:
                self.nav.goalreached = True

        return []
