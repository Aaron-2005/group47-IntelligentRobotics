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

    def send(self, robot_data, survivors, map_data, past_coordinates=None):
        self.update_count += 1
        complete_data = {
            "timestamp": time.time(),
            "update_count": self.update_count,
            "robot": self._format_robot_data(robot_data),
            "survivors": self._format_survivor_data(survivors),
            "mapping": {
                "status": "active",
                "map_size": f"{map_data.shape[0] if hasattr(map_data,'shape') else 200}x{map_data.shape[1] if hasattr(map_data,'shape') else 200}"
            },
            "past_coordinates": past_coordinates or []
        }
        self._save_to_file(complete_data)

    def _format_robot_data(self, robot_data):
        pos = robot_data.get("position", {"x":0,"y":0,"theta":0})
        return {
            "position": {
                "x": round(pos["x"],3),
                "y": round(pos["y"],3),
                "theta": round(pos["theta"],3),
                "theta_degrees": round(pos.get("theta",0)*180/math.pi,1)
            },
            "battery": robot_data.get("battery",85),
            "status": robot_data.get("navigation_state", robot_data.get("status","active")),
            "velocity": robot_data.get("velocity",0.5),
            "left_speed": robot_data.get("left_speed",0),
            "right_speed": robot_data.get("right_speed",0)
        }

    def _format_survivor_data(self, survivors):
        return [
            {"id": idx, "x": s.get("x",0), "y": s.get("y",0), "confidence": s.get("confidence",0.8)}
            for idx,s in enumerate(survivors or [])
        ]

    def _save_to_file(self, data):
        try:
            with open(self.data_file,"w",encoding="utf-8") as f:
                json.dump(data,f,indent=2,ensure_ascii=False)
        except Exception as e:
            print("Error writing JSON:", e)
