from controller import Robot
from navigation import Navigation
from detection import Detection
from communication import Communication
from robot_gui import RescueGUI 

import threading
import time

robot = Robot()
timestep = int(robot.getBasicTimeStep())

nav = Navigation(robot, timestep)
det = Detection(robot, timestep)
com = Communication("robot_data.json")

nav.detect = det

gui = RescueGUI()
threading.Thread(target=gui.run, daemon=True).start()

last_send = time.time()

while robot.step(timestep) != -1:
    det.scan()
    nav.move()

    data = {
        "state": nav.state,
        "battery": 85,
        "position": [nav.x, nav.y],
        "theta": nav.theta,
        "survivors": det.survivors,
        "map": []
    }

    now = time.time()
    if now - last_send > 0.5:
        com.send(data, det.survivors, [])
        last_send = now
