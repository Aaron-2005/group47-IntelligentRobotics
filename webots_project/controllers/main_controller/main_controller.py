# main_controller.py
# Group 47 - Intelligent Robotics

from controller import Robot
import navigation
import mapping
import detection
import communication

# Initialize Webots robot
robot = Robot()
timestep = int(robot.getBasicTimeStep())

# Initialize modules
nav = navigation.Navigation(robot, timestep)
map_module = mapping.Mapping(robot)
detector = detection.Detection(robot)
comm = communication.Communication(robot)

# Link modules
nav.detect = detector
detector.nav = nav

# ----- MAIN LOOP -----
while robot.step(timestep) != -1:

    # 1. Update map
    map_module.update()

    # 2. Detect survivors
    survivors = detector.detect()

    # 3. Navigation update
    nav.move()

    # 4. Robot state report
    robot_data = {
        "position": {
            "x": nav.x,
            "y": nav.y,
            "theta": nav.theta
        },
        "navigation_state": nav.state,
        "goal_position": nav.goal,
        "obstacle_detected": nav.obstacle_detected(),
        "battery": 85,
        "velocity": 0.5,
        "lidar_data": []
    }

    # 5. SEND DATA TO UI (Correct map variable)
    comm.send(robot_data, survivors, map_module.map_data)
