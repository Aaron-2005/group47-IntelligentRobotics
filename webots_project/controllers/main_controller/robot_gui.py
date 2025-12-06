import tkinter as tk
from tkinter import ttk
import json
import os
import threading
import time
import matplotlib
matplotlib.use("TkAgg")
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import matplotlib.pyplot as plt

class RescueGUI:
    def __init__(self):
        self.root = tk.Tk()
        self.root.title("Rescue Robot Monitor")
        self.root.geometry("900x620")
        self.root.resizable(False, False)
        self.data_file = os.path.join(os.path.dirname(__file__), "robot_data.json")
        self.robot_data = {}
        self.survivors = []
        self.past_coords = []
        self.running = True
        self.create_widgets()
        threading.Thread(target=self.update_loop, daemon=True).start()

    def create_widgets(self):
        status_frame = ttk.LabelFrame(self.root,text="Robot Status",width=280,height=140)
        status_frame.place(x=10,y=10)
        self.status_text = tk.StringVar()
        self.status_label = tk.Label(status_frame,textvariable=self.status_text,justify="left",font=("Segoe UI",10))
        self.status_label.pack(anchor="w",padx=6,pady=6)
        control_frame = ttk.LabelFrame(self.root,text="Controls",width=280,height=120)
        control_frame.place(x=10,y=160)
        self.pause_btn = tk.Button(control_frame,text="Pause",width=8,command=self.on_pause)
        self.pause_btn.place(x=10,y=10)
        self.resume_btn = tk.Button(control_frame,text="Resume",width=8,command=self.on_resume)
        self.resume_btn.place(x=100,y=10)
        self.set_goal_x = tk.Entry(control_frame,width=6)
        self.set_goal_x.place(x=10,y=50)
        self.set_goal_y = tk.Entry(control_frame,width=6)
        self.set_goal_y.place(x=80,y=50)
        self.set_goal_btn = tk.Button(control_frame,text="Set Goal",command=self.on_set_goal)
        self.set_goal_btn.place(x=150,y=48)
        map_frame = ttk.LabelFrame(self.root,text="Environment Map",width=580,height=580)
        map_frame.place(x=310,y=10)
        self.fig,self.ax = plt.subplots(figsize=(5.8,5.8))
        self.ax.set_xlim(-2.7,2.7)
        self.ax.set_ylim(-2.7,2.7)
        self.ax.set_xlabel("X (m)")
        self.ax.set_ylabel("Y (m)")
        self.map_canvas = FigureCanvasTkAgg(self.fig,master=map_frame)
        self.map_canvas.get_tk_widget().pack()
        survivor_frame = ttk.LabelFrame(self.root,text="Detected Survivors",width=280,height=300)
        survivor_frame.place(x=10,y=300)
        self.survivor_listbox = tk.Listbox(survivor_frame,width=40,height=16)
        self.survivor_listbox.pack(padx=6,pady=6)
        self.exit_button = tk.Button(self.root,text="Exit",command=self.quit)
        self.exit_button.place(x=120,y=570)

    def on_pause(self):
        try:
            with open(self.data_file,"r",encoding="utf-8") as f:
                data=json.load(f)
            data["robot"]["status"]="paused_by_gui"
            with open(self.data_file,"w",encoding="utf-8") as f:
                json.dump(data,f,indent=2,ensure_ascii=False)
        except:
            pass

    def on_resume(self):
        try:
            with open(self.data_file,"r",encoding="utf-8") as f:
                data=json.load(f)
            data["robot"]["status"]="active"
            with open(self.data_file,"w",encoding="utf-8") as f:
                json.dump(data,f,indent=2,ensure_ascii=False)
        except:
            pass

    def on_set_goal(self):
        try:
            gx = float(self.set_goal_x.get())
            gy = float(self.set_goal_y.get())
            try:
                with open(self.data_file,"r",encoding="utf-8") as f:
                    data=json.load(f)
            except:
                data={}
            data["set_goal"] = {"x":gx,"y":gy}
            with open(self.data_file,"w",encoding="utf-8") as f:
                json.dump(data,f,indent=2,ensure_ascii=False)
        except:
            pass

    def update_loop(self):
        while self.running:
            try:
                if os.path.exists(self.data_file):
                    with open(self.data_file,"r",encoding="utf-8") as f:
                        data = json.load(f)
                        self.robot_data = data.get("robot",{})
                        self.survivors = data.get("survivors",[])
                        self.past_coords = data.get("past_coordinates",[])
                self.update_status()
                self.update_survivors()
                self.update_map()
            except Exception as e:
                print("GUI update error:",e)
            time.sleep(0.4)

    def update_status(self):
        pos = self.robot_data.get("position",{})
        status = self.robot_data.get("status","unknown")
        battery = self.robot_data.get("battery",0)
        left = self.robot_data.get("left_speed",0)
        right = self.robot_data.get("right_speed",0)
        text = f"State: {status}\nBattery: {battery}%\nPosition: ({pos.get('x',0):.2f},{pos.get('y',0):.2f})\nTheta: {self.robot_data.get('position',{}).get('theta_degrees',0):.1f}°\nLeft R: {left:.2f} Right R: {right:.2f}"
        self.status_text.set(text)

    def update_survivors(self):
        self.survivor_listbox.delete(0,tk.END)
        for s in self.survivors:
            line = f"ID {s.get('id',0)}: ({s.get('x',0):.2f},{s.get('y',0):.2f}) Conf:{s.get('confidence',0):.2f}"
            self.survivor_listbox.insert(tk.END,line)
        if self.past_coords:
            self.survivor_listbox.insert(tk.END,"---- Past coords ----")
            for i,(x,y) in enumerate(self.past_coords):
                self.survivor_listbox.insert(tk.END,f"P{i}: ({x:.2f},{y:.2f})")

    def update_map(self):
        self.ax.clear()
        self.ax.set_xlim(-2.7,2.7)
        self.ax.set_ylim(-2.7,2.7)
        self.ax.set_title("Robot Map (m)")
        rx = self.robot_data.get("position",{}).get("x",0)
        ry = self.robot_data.get("position",{}).get("y",0)
        self.ax.plot(rx,ry,"ro",markersize=6,label="Robot")
        theta = self.robot_data.get("position",{}).get("theta",0)
        self.ax.arrow(rx,ry,0.2*math.cos(theta),0.2*math.sin(theta),head_width=0.05,head_length=0.07)
        for s in self.survivors:
            self.ax.plot(s.get("x",0),s.get("y",0),"go",markersize=6)
        for (x,y) in self.past_coords:
            self.ax.plot(x,y,'bx',markersize=4)
        self.map_canvas.draw()

    def quit(self):
        self.running = False
        self.root.quit()

    def run(self):
        self.root.mainloop()

if __name__=="__main__":
    import math
    gui = RescueGUI()
    gui.run()
