import tkinter as tk
from tkinter import ttk
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import matplotlib.pyplot as plt
import numpy as np
import time

class GUIWindow:
    def __init__(self):
        self.root = tk.Tk()
        self.root.title("Rescue Robot - Interactive Monitor")
        self.root.geometry("1000x700")
        self.left_frame = tk.Frame(self.root)
        self.left_frame.pack(side="left", fill="y", padx=8, pady=8)
        self.right_frame = tk.Frame(self.root)
        self.right_frame.pack(side="right", fill="both", expand=True, padx=8, pady=8)
        self.status_title = tk.Label(self.left_frame, text="Robot Status", font=("Segoe UI", 14, "bold"))
        self.status_title.pack(anchor="w", pady=(2,6))
        self.pos_label = tk.Label(self.left_frame, text="Position: x=0.00  y=0.00", font=("Segoe UI", 11))
        self.pos_label.pack(anchor="w", pady=2)
        self.orient_label = tk.Label(self.left_frame, text="Orientation: 0.0°", font=("Segoe UI", 11))
        self.orient_label.pack(anchor="w", pady=2)
        self.battery_label = tk.Label(self.left_frame, text="Battery: --%", font=("Segoe UI", 11))
        self.battery_label.pack(anchor="w", pady=2)
        self.state_label = tk.Label(self.left_frame, text="State: ---", font=("Segoe UI", 11))
        self.state_label.pack(anchor="w", pady=2)
        self.survivor_count = tk.Label(self.left_frame, text="Survivors: 0", font=("Segoe UI", 11))
        self.survivor_count.pack(anchor="w", pady=2)
        self.left_frame.pack_propagate(False)
        self.control_title = tk.Label(self.left_frame, text="Controls", font=("Segoe UI", 14, "bold"))
        self.control_title.pack(anchor="w", pady=(12,6))
        btn_frame = tk.Frame(self.left_frame)
        btn_frame.pack(anchor="w", pady=4)
        self.pause_btn = ttk.Button(btn_frame, text="Pause", command=self._on_pause)
        self.pause_btn.grid(row=0, column=0, padx=4, pady=2)
        self.resume_btn = ttk.Button(btn_frame, text="Resume", command=self._on_resume)
        self.resume_btn.grid(row=0, column=1, padx=4, pady=2)
        self.stop_btn = ttk.Button(btn_frame, text="Stop", command=self._on_stop)
        self.stop_btn.grid(row=0, column=2, padx=4, pady=2)
        goal_frame = tk.Frame(self.left_frame)
        goal_frame.pack(anchor="w", pady=6)
        tk.Label(goal_frame, text="Set Goal (x,y):").grid(row=0, column=0, sticky="w")
        self.goal_x = tk.Entry(goal_frame, width=6)
        self.goal_x.grid(row=0, column=1, padx=4)
        self.goal_y = tk.Entry(goal_frame, width=6)
        self.goal_y.grid(row=0, column=2, padx=4)
        self.set_goal_btn = ttk.Button(goal_frame, text="Set", command=self._on_set_goal)
        self.set_goal_btn.grid(row=0, column=3, padx=4)
        self.speed_title = tk.Label(self.left_frame, text="Speed Scale", font=("Segoe UI", 12))
        self.speed_title.pack(anchor="w", pady=(8,2))
        self.speed_var = tk.DoubleVar(value=1.0)
        self.speed_slider = ttk.Scale(self.left_frame, from_=0.1, to=2.0, orient="horizontal", variable=self.speed_var)
        self.speed_slider.pack(fill="x", pady=4)
        self.log_title = tk.Label(self.left_frame, text="Event Log", font=("Segoe UI", 14, "bold"))
        self.log_title.pack(anchor="w", pady=(12,6))
        self.log_box = tk.Text(self.left_frame, width=34, height=12, state="disabled", wrap="word")
        self.log_box.pack(pady=2)
        self.clear_log_btn = ttk.Button(self.left_frame, text="Clear Log", command=self._on_clear_log)
        self.clear_log_btn.pack(anchor="e", pady=(6,0))
        self.survivor_title = tk.Label(self.right_frame, text="Map & Survivors", font=("Segoe UI", 14, "bold"))
        self.survivor_title.pack(anchor="w")
        self.fig, self.ax = plt.subplots(figsize=(6,6))
        plt.tight_layout()
        self.canvas = FigureCanvasTkAgg(self.fig, master=self.right_frame)
        self.canvas.get_tk_widget().pack(fill="both", expand=True)
        self.map_image = None
        self.robot_marker = None
        self.survivor_markers = []
        self.latest = None
        self.latest_survivors = []
        self.desired_goal = None
        self.paused = False
        self.stopped = False
        self.map_resolution = 0.1
        self.root.bind("<Configure>", lambda e: self._on_resize())
        self.canvas.mpl_connect('button_press_event', self._on_map_click)
        self._schedule_refresh()

    def _on_resize(self):
        pass

    def _on_map_click(self, event):
        if event.xdata is None or event.ydata is None:
            return
        mx = int(round(event.xdata))
        my = int(round(event.ydata))
        map_w, map_h = self._get_map_size()
        cx = map_w // 2
        cy = map_h // 2
        world_x = (mx - cx) * self.map_resolution
        world_y = -(my - cy) * self.map_resolution
        self.desired_goal = (world_x, world_y)
        self._log(f"Map click -> set desired goal: ({world_x:.2f}, {world_y:.2f})")

    def _get_map_size(self):
        if self.map_image is None:
            return 400, 400
        h, w = self.map_image.shape
        return w, h

    def _on_pause(self):
        self.paused = True
        self._log("Pause pressed")

    def _on_resume(self):
        self.paused = False
        self._log("Resume pressed")

    def _on_stop(self):
        self.stopped = True
        self._log("Stop pressed")

    def _on_set_goal(self):
        try:
            gx = float(self.goal_x.get())
            gy = float(self.goal_y.get())
            self.desired_goal = (gx, gy)
            self._log(f"Set goal input -> ({gx:.2f}, {gy:.2f})")
        except:
            self._log("Invalid goal input")

    def _on_clear_log(self):
        self.log_box.config(state="normal")
        self.log_box.delete("1.0", "end")
        self.log_box.config(state="disabled")

    def _log(self, text):
        ts = time.strftime("%H:%M:%S")
        self.log_box.config(state="normal")
        self.log_box.insert("end", f"[{ts}] {text}\n")
        self.log_box.see("end")
        self.log_box.config(state="disabled")

    def update_data(self, data, survivors, map_array=None):
        self.latest = data
        self.latest_survivors = survivors or []
        if isinstance(map_array, (list, tuple, np.ndarray)):
            try:
                arr = np.array(map_array)
                if arr.ndim == 2:
                    self.map_image = arr
            except:
                pass

    def _draw_map(self):
        self.ax.clear()
        if self.map_image is None:
            blank = np.zeros((400,400))
            self.ax.imshow(blank, cmap="gray", origin="lower")
        else:
            display_img = self.map_image
            if np.max(display_img) != np.min(display_img):
                self.ax.imshow(display_img, cmap="gray", origin="lower")
            else:
                self.ax.imshow(display_img, cmap="gray", origin="lower")
        if self.latest:
            rx = int(round(self.latest["position"]["x"] / self.map_resolution + (self._get_map_size()[0]//2)))
            ry = int(round(-(self.latest["position"]["y"] / self.map_resolution) + (self._get_map_size()[1]//2)))
            self.ax.scatter([rx], [ry], c="cyan", s=80, marker="^")
            theta = self.latest["position"]["theta"]
            dx = 10 * np.cos(theta)
            dy = 10 * np.sin(theta)
            self.ax.plot([rx, rx+dx], [ry, ry+dy], c="cyan")
        if self.latest_survivors:
            xs = []
            ys = []
            for s in self.latest_survivors:
                mx = int(round(s["x"] / self.map_resolution + (self._get_map_size()[0]//2)))
                my = int(round(-(s["y"] / self.map_resolution) + (self._get_map_size()[1]//2)))
                xs.append(mx)
                ys.append(my)
            self.ax.scatter(xs, ys, c="red", s=40)
        self.ax.set_xticks([])
        self.ax.set_yticks([])
        self.canvas.draw_idle()

    def _refresh_ui(self):
        if self.latest:
            x = self.latest["position"]["x"]
            y = self.latest["position"]["y"]
            theta = self.latest["position"]["theta"]
            theta_deg = round(theta * 180.0 / 3.1415926, 1)
            self.pos_label.config(text=f"Position: x={x:.2f}  y={y:.2f}")
            self.orient_label.config(text=f"Orientation: {theta_deg}°")
            self.battery_label.config(text=f"Battery: {self.latest.get('battery', '--')}%")
            self.state_label.config(text=f"State: {self.latest.get('navigation_state', '---')}")
            self.survivor_count.config(text=f"Survivors: {len(self.latest_survivors)}")
        self._draw_map()
        self.root.after(200, self._schedule_refresh)

    def _schedule_refresh(self):
        self.root.after(200, self._refresh_ui)

    def run(self):
        self.root.mainloop()
