from controller import Robot
import navigation
import mapping
import detection
from gui_window import GUIWindow
from communication import Communication

robot = Robot()
timestep = int(robot.getBasicTimeStep())

gui = GUIWindow()
comm = Communication(gui)

nav = navigation.Navigation(robot, timestep)
map_module = mapping.Mapping(robot)
detector = detection.Detection(robot)

nav.detect = detector
detector.nav = nav

while robot.step(timestep) != -1:
    map_module.update()
    survivors = detector.detect()
    nav.move()

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

    comm.send(robot_data, survivors, map_module.map_data)
