"""box_controller controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot, Connector
import sys, math

# create the Robot instance.
robot = Robot()

# get the time step of the current world.
TIME_STEP = int(robot.getBasicTimeStep())
if TIME_STEP < 64:
    TIME_STEP = 20
else:
    TIME_STEP = 20

# ----------------- BOX CONNECTORS ---------------
con_box = robot.getDevice('con_box')
con_box.enablePresence(TIME_STEP)


    
# ------ Switch case -----
def manual_mode():
    return "Manual mode"

def remote_mode():
    return "Remote mode"

def automatic_mode():
    return "Automatic mode"

# Set the type of robot mode.
def box_connection(argument):
    switcher = {
        1: manual_mode,
        2: remote_mode,
        3: automatic_mode
    }
    
    # Get the function from switcher dictionary
    func = switcher.get(argument, lambda: "Invalid robot mode")
    # Execute the function
    mode = func()
    return mode

# Main loop:
# - perform simulation steps until Webots is stopping the controller
def main():
    while robot.step(TIME_STEP) != -1:
        # Connector for connecting to boxes
        # print("Box Connector presence: ", con_box.getPresence())
        if (con_box.getPresence() == 1):
            # print("Connector presence!")
            con_box.lock()
        # Read the sensors:
        # Enter here functions to read sensor data, like:
         # val = ds.getValue()
    
        # Process sensor data here.
    
        # Enter here functions to send actuator commands, like:
         # motor.setPosition(10.0)
        pass

# Method to start the program by running the main. Python override.
if __name__ == "__main__":
    main()