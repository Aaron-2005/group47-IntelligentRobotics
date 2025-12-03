from controller import Robot
import navigation
import mapping
import detection
import communication
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

while robot.step(timestep) != -1:
    map_module.update()
    survivors = detector.detect()
    nav.move()

    robot_data = {
        "timestamp": time.time(),
        "position":{"x":nav.x,"y":nav.y,"theta":nav.theta},
        "navigation_state": nav.state,
        "goal_position": nav.goal,
        "obstacle_detected": nav.obstacle_detected(),
        "battery": 85,
        "velocity": nav.current_speed if hasattr(nav,"current_speed") else 0.5,
        "lidar": map_module.lidar.getRangeImage() if hasattr(map_module.lidar,"getRangeImage") else []
    }

    comm.send(robot_data, survivors, map_module.map_data)

    cmd = comm.receive_command()
    if cmd:
        action = cmd.get("action")
        if action=="pause":
            nav.pause()
        elif action=="resume":
            nav.resume()
        elif action=="set_goal":
            goal = cmd.get("goal")
            if goal and len(goal)==2:
                nav.reset(new_goal=goal)
        comm.clear_command()
