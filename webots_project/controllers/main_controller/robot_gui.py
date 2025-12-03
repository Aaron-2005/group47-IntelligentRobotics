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
import numpy as np

class RescueGUI:
    def __init__(self):
        self.root = tk.Tk()
        self.root.title("Rescue Robot Monitor")
        self.root.geometry("900x600")
        self.root.resizable(False, False)

        self.data_file = os.path.join(os.path.dirname(__file__), "robot_data.json")
        self.robot_data = {}
        self.survivors = []
        self.running = True

        self.create_widgets()
        threading.Thread(target=self.update_loop, daemon=True).start()

    def create_widgets(self):
        status_frame = ttk.LabelFrame(self.root,text="Robot Status",width=300,height=120)
        status_frame.place(x=10,y=10)
        self.status_text = tk.StringVar()
        self.status_label = tk.Label(status_frame,textvariable=self.status_text,justify="left")
        self.status_label.pack(anchor="w",padx=10,pady=5)

        map_frame = ttk.LabelFrame(self.root,text="Environment Map",width=580,height=580)
        map_frame.place(x=310,y=10)
        self.fig,self.ax = plt.subplots(figsize=(5.8,5.8))
        self.ax.set_xlim(0,200)
        self.ax.set_ylim(0,200)
        self.map_canvas = FigureCanvasTkAgg(self.fig,master=map_frame)
        self.map_canvas.get_tk_widget().pack()

        survivor_frame = ttk.LabelFrame(self.root,text="Detected Survivors",width=280,height=470)
        survivor_frame.place(x=10,y=140)
        self.survivor_listbox = tk.Listbox(survivor_frame,width=38,height=27)
        self.survivor_listbox.pack(padx=5,pady=5)

        self.exit_button = tk.Button(self.root,text="Exit",command=self.quit)
        self.exit_button.place(x=120,y=570)

    def update_loop(self):
        while self.running:
            try:
                if os.path.exists(self.data_file):
                    with open(self.data_file,"r",encoding="utf-8") as f:
                        data = json.load(f)
                        self.robot_data = data.get("robot",{})
                        self.survivors = data.get("survivors",[])
                self.update_status()
                self.update_survivors()
                self.update_map()
            except Exception as e:
                print("GUI update error:",e)
            time.sleep(0.5)

    def update_status(self):
        pos = self.robot_data.get("position",{})
        status = self.robot_data.get("status","unknown")
        battery = self.robot_data.get("battery",0)
        text = f"State: {status}\nBattery: {battery}%\n"
        text += f"Position: ({pos.get('x',0):.2f},{pos.get('y',0):.2f})\n"
        text += f"Theta: {pos.get('theta_degrees',0):.1f}°"
        self.status_text.set(text)

    def update_survivors(self):
        self.survivor_listbox.delete(0,tk.END)
        for s in self.survivors:
            line = f"ID {s.get('id',0)}: ({s.get('x',0):.2f},{s.get('y',0):.2f}) Conf:{s.get('confidence',0):.2f}"
            self.survivor_listbox.insert(tk.END,line)

    def update_map(self):
        self.ax.clear()
        self.ax.set_xlim(0,200)
        self.ax.set_ylim(0,200)
        self.ax.set_title("Robot Map")
        self.ax.plot(self.robot_data.get("position",{}).get("x",0)*10,
                     self.robot_data.get("position",{}).get("y",0)*10,"ro",markersize=6,label="Robot")
        for s in self.survivors:
            self.ax.plot(s.get("x",0)*10,s.get("y",0)*10,"go",markersize=5)
        self.map_canvas.draw()

    def quit(self):
        self.running = False
        self.root.quit()

    def run(self):
        self.root.mainloop()

if __name__=="__main__":
    gui = RescueGUI()
    gui.run()
