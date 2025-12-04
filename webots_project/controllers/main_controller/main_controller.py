import os
import sys
import time
import json
import threading
import webbrowser

from controller import Supervisor

from navigation import NavigationModule
from detection import DetectionModule
from mapping import MappingModule
from data_logger import DataLogger


class RescueRobotController:
    def __init__(self):
        self.robot = Supervisor()
        self.timestep = int(self.robot.getBasicTimeStep())

        self.navigation = NavigationModule(self.robot)
        self.detection = DetectionModule(self.robot)
        self.mapping = MappingModule(self.robot)

        self.data_logger = DataLogger()

        # 自动打开浏览器（延迟 1 秒）
        threading.Timer(1.0, lambda: webbrowser.open_new("http://localhost:8000")).start()

        self.start_time = time.time()

    def step(self):
        return self.robot.step(self.timestep)

    def main_loop(self):
        print("nav complete")
        print("Detection module initialized")

        while True:
            if self.step() == -1:
                break

            robot_pose = self.navigation.get_pose()
            if robot_pose is None:
                print("Pose became invalid, resetting.")
                self.navigation.reset_pose()
                continue

            detections = self.detection.detect()
            self.mapping.update(robot_pose, detections)

            state = self.navigation.update()

            self.data_logger.log({
                "state": state,
                "pose": robot_pose,
                "detections": detections,
                "map": self.mapping.get_map()
            })


if __name__ == "__main__":
    controller = RescueRobotController()
    controller.main_loop()
