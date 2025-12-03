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

gui = None

def start_gui():
    global gui
    gui = GUIWindow()
    gui.run()

gui_thread = threading.Thread(target=start_gui, daemon=True)
gui_thread.start()

time.sleep(1.0)

while robot.step(timestep) != -1:
    map_module.update()
    survivors = detector.detect()
    nav.move()

    try:
        left_speed = nav.left_motor.getVelocity()
        right_speed = nav.right_motor.getVelocity()
    except:
        left_speed = 0
        right_speed = 0

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

    try:
        comm.send(robot_data, survivors, map_module.map_data)
    except:
        pass

    if gui is not None:
        try:
            gui.update_data(robot_data, survivors, map_module.map_data)
        except:
            pass

        if gui.desired_goal is not None:
            try:
                nav.reset(new_goal=tuple(gui.desired_goal))
            except:
                pass
            gui.desired_goal = None

        if gui.paused:
            nav.pause()
        else:
            nav.resume()

        if gui.stopped:
            nav.pause()
            gui.stopped = False

    time.sleep(0.02)
