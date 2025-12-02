from controller import Robot
import time
import json
import os

from navigation import Navigation
from detection import Detection
from mapping import Mapping
from communication import Communication


def main():
    robot = Robot()
    timestep = int(robot.getBasicTimeStep())

    # === MODULES INITIALISATION ===
    nav = Navigation(robot, timestep)
    mapping = Mapping(robot)
    detect = Detection(robot)
    comm = Communication(robot_instance=robot)

    # bind modules with each other
    detect.nav = nav  # detection --> navigation uses nav.reset()

    print("Main system initialised.")

    # ============================================
    # HELPERS FOR COMMUNICATION / UI
    # ============================================
    def write_data_to_json():
        """Write real-time data to robot_data.json for UI"""
        data = {
            "time": round(time.time() - comm.start_time, 2),

            # robot pose
            "pose": {
                "x": round(nav.x, 3),
                "y": round(nav.y, 3),
                "theta": round(nav.theta, 3),
            },

            # lidar map (for UI)
            "map_ready": True,
            "map_width": mapping.MAP_W,
            "map_height": mapping.MAP_H,
            "map_data": mapping.map_data.tolist(),

            # navigation state
            "navigation": {
                "state": nav.state,
                "goal": nav.goal,
            },

            # detection data
            "detected_survivors": detect.past_coordinates,
            "current_scan_done": detect.scan_done,
        }

        with open(comm.data_file, "w") as f:
            json.dump(data, f, indent=4)

    # ============================================
    # MAIN CONTROL LOOP
    # ============================================
    while robot.step(timestep) != -1:

        # --- UPDATE MODULES ---
        nav.update_odometry()
        mapping.update()
        detect.detect()

        # --- COMPUTE MOTION ---
        if nav.paused:  # paused during detection rotation
            nav.left_motor.setVelocity(0)
            nav.right_motor.setVelocity(0)
        else:
            vL, vR, dist = nav.goto_position(nav.goal[0], nav.goal[1])
            nav.left_motor.setVelocity(vL)
            nav.right_motor.setVelocity(vR)

        # --- WRITE TO UI ---
        write_data_to_json()

    # exit cleanly
    print("Simulation finished.")


if __name__ == "__main__":
    main()
