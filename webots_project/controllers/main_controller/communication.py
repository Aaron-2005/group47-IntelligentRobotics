import json
import time
import math
import os


class Communication:
    def __init__(self, robot_instance=None):
        self.data_file = os.path.join(os.path.dirname(__file__), "robot_data.json")
        self.robot = robot_instance
        self.start_time = time.time()
        self.update_count = 0

    def send(self, robot_data, survivors, map_data):
        self.update_count += 1

        complete_data = {
            "timestamp": time.time(),
            "update_count": self.update_count,
            "robot": self._format_robot_data(robot_data),
            "survivors": self._format_survivor_data(survivors),
            "navigation": self._format_navigation_data(robot_data),
            "mapping": self._format_mapping_data(map_data),
            "system": self._format_system_data(),
            "sensors": self._format_sensor_data(robot_data)
        }

        self._save_to_file(complete_data)

    def _format_robot_data(self, robot_data):
        pos = robot_data["position"]
        return {
            "position": {
                "x": round(pos["x"], 3),
                "y": round(pos["y"], 3),
                "theta": round(pos["theta"], 3),
                "theta_degrees": round(pos["theta"] * 180 / math.pi, 1)
            },
            "battery": robot_data.get("battery", 85),
            "status": "active",
            "velocity": robot_data.get("velocity", 0.5),
            "data_source": "navigation"
        }

    def _format_navigation_data(self, robot_data):
        return {
            "current_state": robot_data["navigation_state"],
            "goal_position": robot_data["goal_position"],
            "obstacle_detected": robot_data["obstacle_detected"],
            "movement_status": "exploring",
            "algorithm": "Bug2_with_Obstacle_Avoidance"
        }

    def _format_survivor_data(self, survivors):
        return [
            {
                "id": idx,
                "x": s.get("x", 0),
                "y": s.get("y", 0),
                "confidence": s.get("confidence", 0.8),
                "status": "detected",
                "data_source": "camera"
            }
            for idx, s in enumerate(survivors)
        ]

    def _format_mapping_data(self, map_data):
        if hasattr(map_data, "shape"):
            return {
                "status": "active",
                "map_size": f"{map_data.shape[0]}x{map_data.shape[1]}",
                "resolution": "0.1m/pixel",
                "data_source": "slam"
            }
        return {
            "status": "initializing",
            "map_size": "200x200",
            "resolution": "0.1m/pixel",
            "data_source": "placeholder"
        }

    def _format_sensor_data(self, robot_data):
        lidar_data = robot_data.get("lidar_data", [])
        return {
            "lidar": {
                "points": len(lidar_data),
                "range": (
                    f"{min(lidar_data):.2f}-{max(lidar_data):.2f}m"
                    if lidar_data else "0-0m"
                ),
                "status": "active",
            },
            "camera": {"status": "streaming"},
            "imu": {"status": "active"},
        }

    def _format_system_data(self):
        return {
            "system_status": "operational",
            "data_quality": "sensor_data",
        }

    def _save_to_file(self, data):
        with open(self.data_file, "w", encoding="utf-8") as f:
            json.dump(data, f, indent=2, ensure_ascii=False)
