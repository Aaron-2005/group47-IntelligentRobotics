# popup_ui.py
import tkinter as tk
from tkinter import ttk
import json
import os
import threading
import time

class RescueRobotPopup:
    def __init__(self, data_file="robot_data.json", update_interval=0.5):
        self.data_file = data_file
        self.update_interval = update_interval
        self.current_data = {}
        
        self.root = tk.Tk()
        self.root.title("Rescue Robot Monitor - Pop-up")
        self.root.geometry("500x400")
        
        self.create_widgets()
        self.running = True
        
        # Start polling in a separate thread
        threading.Thread(target=self.poll_data, daemon=True).start()
        
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)
        self.root.mainloop()
    
    def create_widgets(self):
        self.status_label = ttk.Label(self.root, text="System Status: --")
        self.status_label.pack(pady=5)
        
        self.update_label = ttk.Label(self.root, text="Last Update: --")
        self.update_label.pack(pady=5)
        
        self.robot_pos_label = ttk.Label(self.root, text="Robot Position: X=0, Y=0")
        self.robot_pos_label.pack(pady=5)
        
        self.battery_label = ttk.Label(self.root, text="Battery: --%")
        self.battery_label.pack(pady=5)
        
        self.nav_state_label = ttk.Label(self.root, text="Navigation State: --")
        self.nav_state_label.pack(pady=5)
    
    def poll_data(self):
        while self.running:
            if os.path.exists(self.data_file):
                try:
                    with open(self.data_file, "r", encoding="utf-8") as f:
                        data = json.load(f)
                        self.current_data = data
                        self.update_display()
                except Exception:
                    pass
            time.sleep(self.update_interval)
    
    def update_display(self):
        data = self.current_data
        self.status_label.config(text=f"System Status: {data.get('system', {}).get('system_status', '--')}")
        self.update_label.config(text=f"Last Update: {time.strftime('%H:%M:%S')}")
        robot = data.get("robot")
        if robot:
            pos = robot.get("position", {})
            self.robot_pos_label.config(text=f"Robot Position: X={pos.get('x',0):.2f}, Y={pos.get('y',0):.2f}")
            self.battery_label.config(text=f"Battery: {robot.get('battery',0)}%")
        nav = data.get("navigation")
        if nav:
            self.nav_state_label.config(text=f"Navigation State: {nav.get('current_state','--')}")
    
    def on_close(self):
        self.running = False
        self.root.destroy()


if __name__ == "__main__":
    RescueRobotPopup()
