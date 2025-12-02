# gui_window.py
import tkinter as tk
from tkinter import ttk
import time


class GUIWindow:
    def __init__(self):
        self.root = tk.Tk()
        self.root.title("Rescue Robot - Live Monitor")
        self.root.geometry("450x300")

        # ---- Title ----
        title = tk.Label(self.root, text="Rescue Robot Status", font=("Segoe UI", 16, "bold"))
        title.pack(pady=10)

        # ---- Info Frame ----
        frame = tk.Frame(self.root)
        frame.pack(pady=5)

        # Position
        self.position_label = tk.Label(frame, text="Position: x=0.00  y=0.00", font=("Segoe UI", 12))
        self.position_label.pack(anchor="w")

        # Orientation
        self.orientation_label = tk.Label(frame, text="Orientation: 0°", font=("Segoe UI", 12))
        self.orientation_label.pack(anchor="w")

        # Motors
        self.motor_label = tk.Label(frame, text="Motors: L=0.00 R=0.00", font=("Segoe UI", 12))
        self.motor_label.pack(anchor="w")

        # Status
        self.status_label = tk.Label(frame, text="State: ---", font=("Segoe UI", 12))
        self.status_label.pack(anchor="w")

        # Survivor count
        self.survivor_label = tk.Label(frame, text="Survivors detected: 0", font=("Segoe UI", 12))
        self.survivor_label.pack(anchor="w")

        # ---- Data Storage ----
        self.latest_data = None

        # Start refreshing UI
        self.root.after(200, self.refresh_loop)

    def update_data(self, data, survivors):
        """Called by communication module each timestep"""
        self.latest_data = (data, survivors)

    def refresh_loop(self):
        """Refresh UI every 200 ms"""
        if self.latest_data:
            robot_data, survivors = self.latest_data

            x = robot_data["position"]["x"]
            y = robot_data["position"]["y"]
            theta_deg = round(robot_data["position"]["theta"] * 180 / 3.1416, 1)

            left = robot_data.get("left_speed", 0)
            right = robot_data.get("right_speed", 0)

            self.position_label.config(text=f"Position: x={x:.2f}  y={y:.2f}")
            self.orientation_label.config(text=f"Orientation: {theta_deg}°")
            self.motor_label.config(text=f"Motors: L={left:.2f}  R={right:.2f}")
            self.status_label.config(text=f"State: {robot_data['navigation_state']}")
            self.survivor_label.config(text=f"Survivors detected: {len(survivors)}")

        # Continue updating
        self.root.after(200, self.refresh_loop)

    def run(self):
        self.root.mainloop()
