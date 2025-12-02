# communication.py

import json
import time
import math
import webbrowser
import os


class Communication:
    def __init__(self, robot_instance=None):
        self.robot = robot_instance
        self.data_file = "robot_data.json"
        self.start_time = time.time()
        self.update_count = 0

    def send(self, robot_data, survivors, map_data):
        current_time = time.time() - self.start_time
        self.update_count += 1

        complete_data = {
            "timestamp": time.time(),
            "update_count": self.update_count,
            "robot": self._format_robot_data(robot_data),
            "survivors": self._format_survivor_data(survivors),
            "navigation": self._format_navigation_data(robot_data),
            "mapping": self._format_mapping_data(map_data),
            "system": self._format_system_data(current_time),
            "sensors": self._format_sensor_data(robot_data)
        }

        self._save_to_file(complete_data)

    def _format_robot_data(self, robot_data):
        position = robot_data["position"]
        return {
            "position": {
                "x": round(position["x"], 3),
                "y": round(position["y"], 3),
                "theta": round(position["theta"], 3),
                "theta_degrees": round(position["theta"] * 180 / math.pi, 1)
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
        if survivors and len(survivors) > 0:
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
        return []

    def _format_mapping_data(self, map_data):
        if hasattr(map_data, 'shape'):
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
                "status": "active"
            },
            "camera": {"status": "streaming", "resolution": "640x480"},
            "imu": {"status": "active"}
        }

    def _format_system_data(self, current_time):
        return {
            "operational_time": int(current_time),
            "update_rate": "real_time",
            "system_status": "operational",
            "communication_mode": "file_json",
            "data_quality": "sensor_data"
        }

    def _save_to_file(self, data):
        try:
            with open(self.data_file, 'w', encoding='utf-8') as f:
                json.dump(data, f, indent=2, ensure_ascii=False)
        except Exception:
            pass

    def send_emergency_alert(self, alert_type, message):
        alert_data = {
            "type": "emergency_alert",
            "timestamp": time.time(),
            "alert": {
                "type": alert_type,
                "message": message,
                "priority": "high"
            }
        }
        self._save_to_file(alert_data)


def test_communication_system():
    comm = Communication()

    test_robot_data = {
        "position": {"x": 1.23, "y": 0.87, "theta": 0.78},
        "navigation_state": "GO_TO_GOAL",
        "goal_position": (1.0, 0),
        "obstacle_detected": False,
        "battery": 82,
        "lidar_data": [1.5, 1.6, 1.4, 2.0, 1.8],
        "velocity": 0.55
    }

    test_survivors = []
    test_map_data = {}

    for _ in range(3):
        comm.send(test_robot_data, test_survivors, test_map_data)
        time.sleep(1)


if __name__ == "__main__":

    base_path = os.path.dirname(os.path.abspath(__file__))
    file_url = "file:///" + os.path.join(base_path, "index.html").replace("\\", "/")
    webbrowser.open(file_url)


    test_communication_system()
