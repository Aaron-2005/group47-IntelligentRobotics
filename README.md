README – Group 47 (Intelligent Robotics)
Autonomous Disaster-Response Robot – Webots Simulation
Team Members

Abdulmajeed Alanazi – Localisation & Mapping

Aaron Abraham – Survivor Detection

Sebastian Andre – Navigation & Obstacle Avoidance

Jialun Wu – Communication System & User Interface

What We Implemented Ourselves
Survivor Detection (Aaron)

HSV colour segmentation (red survivor boxes)

Contour extraction + filtering

Angle estimation from pixel offsets

Monocular distance estimation using camera geometry

360° scanning + survivor tracking

Global coordinate transformation + duplicate filtering

Navigation & Obstacle Avoidance (Sebastian)

Odometry (encoder + IMU fusion)

Proportional controller (goal-directed motion)

Bug2 algorithm with wall-following

Lidar-based obstacle detection

Localisation & Mapping (Abdulmajeed)

Occupancy grid (log-odds representation)

Lidar ray-tracing for free/occupied updates

Multi-hit confirmation filtering

Simple scan-matching SLAM to reduce drift

Communication & GUI (Jialun)

JSON-based communication between robot and GUI

External Tkinter GUI

Real-time plotting of robot pose & detected survivors

Pre-Programmed Packages Used

OpenCV (cv2)	-> Image masking, HSV conversion, contour detection
NumPy	-> Mathematical calculations, vector operations
Tkinter	-> GUI interface
Matplotlib	-> Plotting robot + survivor positions
JSON, threading, subprocess, os	-> File communication, async updates
Webots API (Robot, Camera, Lidar, InertialUnit, PositionSensor, Display)	-> Reading sensors and controlling the robot

All mapping, navigation, detection, and SLAM logic was programmed manually.
