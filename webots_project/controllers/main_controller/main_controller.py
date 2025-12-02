from controller import Robot
import navigation
import mapping
import detection
import communication
from gui_window import GUIWindow
import threading
import time

robot = Robot()
timestep = int(robot.getBasicTimeStep())

nav = navigation.Navigation(robot, timestep)
map_module = mapping.Mapping(robot)
detector = detection.Detection(robot)
comm = communication.Communication(robot)

nav.detect = detector
detector.nav = nav

gui = GUIWindow()

def robot_loop():
    while robot.step(timestep) != -1:
        map_module.update()
        survivors = detector.detect()
        nav.move()

        left_speed = nav.left_motor.getVelocity()
        right_speed = nav.right_motor.getVelocity()
        nav.left_speed = left_speed
        nav.right_speed = right_speed

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
            "left_speed": left_speed,
            "right_speed": right_speed,
            "lidar_data": []
        }

        comm.send(robot_data, survivors, map_module.map_data)
        gui.update_data(robot_data, survivors)
        time.sleep(0.02)

threading.Thread(target=robot_loop, daemon=True).start()
gui.run()
