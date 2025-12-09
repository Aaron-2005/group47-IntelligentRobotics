from controller import Robot
import navigation
import mapping
import detection
import communication
import subprocess
import sys
import os
import time

robot = Robot()
timestep = int(robot.getBasicTimeStep())

try:
    gui_path = os.path.join(os.path.dirname(__file__), "robot_gui.py")
    if os.path.exists(gui_path):
        subprocess.Popen([sys.executable, gui_path], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
except Exception as e:
    print("Could not start GUI:", e)

nav = navigation.Navigation(robot, timestep)
map_module = mapping.Mapping(robot)
detector = detection.Detection(robot)
comm = communication.Communication(robot)

nav.detect = detector
detector.nav = nav

last_send = time.time()

while robot.step(timestep) != -1 and detector.all_human_reached == False:
    try:
        map_module.update()
    except Exception as e:
        print("map update error:", e)
    try:
        survivors = detector.detect()
    except Exception as e:
        print("detection error:", e)
        survivors = []
    try:
        nav.move()
    except Exception as e:
        print("nav move error:", e)
    try:
        left_speed = nav.left_motor.getVelocity() if hasattr(nav, "left_motor") else 0
        right_speed = nav.right_motor.getVelocity() if hasattr(nav, "right_motor") else 0
    except Exception:
        left_speed = 0
        right_speed = 0
    robot_data = {
        "position": {"x": nav.x, "y": nav.y, "theta": nav.theta},
       "navigation_state": nav.state,
       "goal_position": nav.goal,
       "obstacle_detected": nav.obstacle_detected() if hasattr(nav, "obstacle_detected") else False,
       "battery": 85,
       "velocity": nav.just_reset if hasattr(nav, "just_reset") else 0.5,
       "left_speed": left_speed,
       "right_speed": right_speed,
       "lidar_data": getattr(map_module, "lidar_raw", [])
    }
    try:
        past_coords = getattr(detector, "past_coordinates", []) if hasattr(detector, "past_coordinates") else []
        if time.time() - last_send > 0.2:
            comm.send(robot_data, survivors, getattr(map_module, "map_data", []), past_coords)
            last_send = time.time()
    except Exception as e:
        print("comm send error:", e)
