import tkinter as tk

class GUIWindow:
    def __init__(self):
        self.root = tk.Tk()
        self.root.title("Rescue Robot Monitor")

        self.pos_label = tk.Label(self.root, text="Position: x=0  y=0", font=("Arial", 12))
        self.pos_label.pack(pady=5)

        self.yaw_label = tk.Label(self.root, text="Orientation (yaw): 0", font=("Arial", 12))
        self.yaw_label.pack(pady=5)

        self.motor_label = tk.Label(self.root, text="Motor: L=0  R=0", font=("Arial", 12))
        self.motor_label.pack(pady=5)

        self.root.update()

    def update(self, data):
        x = round(data["position"]["x"], 3)
        y = round(data["position"]["y"], 3)
        yaw = round(data["orientation"], 3)
        left = data["motor"]["left"]
        right = data["motor"]["right"]

        self.pos_label.config(text=f"Position: x={x}  y={y}")
        self.yaw_label.config(text=f"Orientation (yaw): {yaw}")
        self.motor_label.config(text=f"Motor: L={left}  R={right}")

        self.root.update()
