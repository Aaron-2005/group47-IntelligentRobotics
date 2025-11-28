# main_controller.py
# Group 47 - Intelligent Robotics
# Main controller for disaster response robot
# This integrates mapping, navigation, detection, and communication modules

from controller import Robot
import navigation
import mapping
import detection
import communication

# Create the Robot instance
robot = Robot()

# Get the simulation time step (ms)
timestep = int(robot.getBasicTimeStep())
# Initialize modules
nav = navigation.Navigation(robot, timestep)
map_module = mapping.Mapping(robot)
detector = detection.Detection(robot)
comm = communication.Communication(robot)

# Main control loop
while robot.step(timestep) != -1:
    # Update map
    map_module.update()

    # Detect survivors
    survivors = detector.detect()

    # Navigate safely
    nav.move()

    # Send data back to rescue team
    #comm.send(map_module.map_data, survivors)
