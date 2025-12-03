import tkinter as tk
from tkinter import ttk
import threading
import json
import os
import time

class RobotGUI:
    def __init__(self):
        self.root = tk.Tk()
        self.root.title("Rescue Robot GUI")
        self.data_file = os.path.join(os.path.dirname(__file__), "robot_data.json")
        self.cmd_file = os.path.join(os.path.dirname(__file__), "gui_command.json")
        self.root.geometry("500x400")

        self.status_label = tk.Label(self.root, text="Status: N/A")
        self.status_label.pack(pady=5)
        self.battery_label = tk.Label(self.root, text="Battery: N/A")
        self.battery_label.pack(pady=5)
        self.goal_label = tk.Label(self.root, text="Goal: N/A")
        self.goal_label.pack(pady=5)

        self.pause_btn = tk.Button(self.root, text="Pause", command=self.send_pause)
        self.pause_btn.pack(side=tk.LEFT, padx=10, pady=10)
        self.resume_btn = tk.Button(self.root, text="Resume", command=self.send_resume)
        self.resume_btn.pack(side=tk.LEFT, padx=10, pady=10)

        tk.Label(self.root, text="Set New Goal X,Y").pack()
        frame = tk.Frame(self.root)
        frame.pack(pady=5)
        self.goal_x = tk.Entry(frame,width=5)
        self.goal_x.pack(side=tk.LEFT,padx=2)
        self.goal_y = tk.Entry(frame,width=5)
        self.goal_y.pack(side=tk.LEFT,padx=2)
        self.set_goal_btn = tk.Button(frame,text="Set Goal",command=self.send_new_goal)
        self.set_goal_btn.pack(side=tk.LEFT,padx=5)

        self.update_thread = threading.Thread(target=self.update_loop, daemon=True)
        self.update_thread.start()

    def send_pause(self):
        self._write_command({"action":"pause"})

    def send_resume(self):
        self._write_command({"action":"resume"})

    def send_new_goal(self):
        try:
            x = float(self.goal_x.get())
            y = float(self.goal_y.get())
            self._write_command({"action":"set_goal","goal":[x,y]})
        except:
            pass

    def _write_command(self,cmd):
        try:
            with open(self.cmd_file,"w",encoding="utf-8") as f:
                json.dump(cmd,f)
        except:
            pass

    def update_loop(self):
        while True:
            try:
                if os.path.exists(self.data_file):
                    with open(self.data_file,"r",encoding="utf-8") as f:
                        data = json.load(f)
                    robot = data.get("robot",{})
                    nav = data.get("navigation",{})
                    self.status_label.config(text=f"State: {nav.get('current_state','N/A')}")
                    self.battery_label.config(text=f"Battery: {robot.get('battery',0)}%")
                    goal = nav.get("goal_position",[0,0])
                    self.goal_label.config(text=f"Goal: ({goal[0]:.2f},{goal[1]:.2f})")
            except:
                pass
            time.sleep(0.5)

    def run(self):
        self.root.mainloop()

if __name__=="__main__":
    gui = RobotGUI()
    gui.run()
