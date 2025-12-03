from controller import Robot
import navigation
import mapping
import detection
import communication
import threading
import time
import os

robot = Robot()
timestep = int(robot.getBasicTimeStep())
nav = navigation.Navigation(robot, timestep)
map_module = mapping.Mapping(robot)
detector = detection.Detection(robot)
comm = communication.Communication(robot)
nav.detect = detector
detector.nav = nav
while robot.step(timestep) != -1:
    map_module.update()
    survivors = detector.detect()
    nav.move()

    robot_data = {
        "timestamp": time.time(),
        "position": {
            "x": nav.x,
            "y": nav.y,
            "theta": nav.theta
        },
        "status": nav.state,
        "battery": 85,
        "velocity": nav.max_speed
    }

    comm.send(robot_data, survivors, map_module.map_data)
