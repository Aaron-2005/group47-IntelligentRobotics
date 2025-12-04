import subprocess
import sys
import os

def start_gui():
    controller_dir = os.path.dirname(__file__)
    gui_path = os.path.join(controller_dir, "robot_gui.py")   
    print("Launching GUI:", gui_path)

    subprocess.Popen([sys.executable, gui_path])

if __name__ == "__main__":
    start_gui()
