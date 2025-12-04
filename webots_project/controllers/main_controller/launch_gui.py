import subprocess
import sys
import os

def start_gui():

    controller_dir = os.getcwd()
    gui_path = os.path.join(controller_dir, "gui_window.py")

    print("GUI path:", gui_path)  

    subprocess.Popen([sys.executable, gui_path])

if __name__ == "__main__":
    start_gui()
