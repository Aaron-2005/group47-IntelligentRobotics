# Group 47 – Intelligent Robotics  
## Autonomous Disaster Response Robot (Webots Simulation)

### Overview
This project implements an autonomous disaster response robot in Webots that:
- Navigates a cluttered environment
- Builds a map of its surroundings (SLAM)
- Detects survivors using colour based vision
- Sends real-time state and detection data to an external GUI

---

### Team Members

- **Abdulmajeed Alanazi** – Localisation & Mapping (SLAM)  
- **Aaron Abraham** – Survivor Detection  
- **Sebastian Andre** – Navigation & Obstacle Avoidance  
- **Jialun Wu** – Communication System & User Interface  

---

### What We Implemented Ourselves

#### Survivor Detection (Aaron)
- HSV colour segmentation to detect red survivor markers  
- Contour extraction and filtering  
- Angle estimation from pixel offsets  
- Monocular distance estimation using camera geometry  
- 360° camera scanning and survivor tracking  
- Global coordinate transformation and duplicate filtering  

#### Navigation & Obstacle Avoidance (Sebastian)
- Odometry using wheel encoders and IMU fusion  
- Proportional controller for goal-directed motion  
- Bug2 algorithm with wall-following behaviour  
- Lidar-based obstacle detection and state switching  

#### Localisation & Mapping (Abdulmajeed)
- Occupancy grid using log-odds representation  
- Lidar ray-tracing to update free and occupied cells  
- Multi-hit confirmation filtering for stable obstacle detection  
- Lightweight scan-matching SLAM to reduce odometry drift  

#### Communication & User Interface (Jialun)
- JSON-based communication between Webots controller and external GUI  
- External Tkinter-based GUI application  
- Real-time plotting of robot pose, detected survivors, and visited locations  

---

### Pre-Programmed Packages Used

| Package / API | Purpose |
| ------------- | ------- |
| **OpenCV (cv2)** | HSV conversion, image masking, contour detection |
| **NumPy** | Numerical operations, vector and matrix maths |
| **Tkinter** | Desktop GUI framework |
| **Matplotlib** | Plotting robot and survivor positions in the GUI |
| **json, threading, subprocess, os** | File I/O, process management, asynchronous updates |
| **Webots API** (`Robot`, `Camera`, `Lidar`, `InertialUnit`, `PositionSensor`, `Display`) | Access to simulator sensors and actuators |

