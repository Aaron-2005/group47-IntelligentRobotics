import subprocess
import sys
import os

def start_gui():
    gui_path = os.path.abspath(os.path.join(os.path.dirname(__file__), "gui_window.py"))
    subprocess.Popen([sys.executable, gui_path])

if __name__ == "__main__":
    start_gui()
