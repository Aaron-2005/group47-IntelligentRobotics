from controller import Robot, GPS, InertialUnit, Compass
import math
from communication import Communication
from gui_window import GUIWindow

robot = Robot()
timestep = int(robot.getBasicTimeStep())

gps = robot.getDevice("gps")
gps.enable(timestep)

imu = robot.getDevice("inertial_unit")
imu.enable(timestep)

compass = robot.getDevice("compass")
compass.enable(timestep)

left_motor = robot.getDevice("left wheel motor")
right_motor = robot.getDevice("right wheel motor")
left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))
left_motor.setVelocity(0)
right_motor.setVelocity(0)

comm = Communication()
gui = GUIWindow()

while robot.step(timestep) != -1:

    position = gps.getValues()
    imu_data = imu.getRollPitchYaw()
    compass_data = compass.getValues()

    x = position[0]
    y = position[2]
    yaw = imu_data[2]

    left_speed = 2.0
    right_speed = 2.0
    left_motor.setVelocity(left_speed)
    right_motor.setVelocity(right_speed)

    data = {
        "position": {"x": x, "y": y},
        "orientation": yaw,
        "motor": {"left": left_speed, "right": right_speed}
    }

    comm.update_data(data)
    gui.update(data)
