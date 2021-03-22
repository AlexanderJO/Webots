"""agv_controller controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot, DistanceSensor, Motor, Keyboard
import sys, math
from ethercatCommunication import Communication

#robotCtrlTest = RobotControllerTest()

#def __init__(self):
#if __name__ == "__main__":

# Communication
communication = Communication()
    
# create the Robot instance.
robot = Robot()

# get the time step of the current world.
TIME_STEP = int(robot.getBasicTimeStep())
if TIME_STEP < 64:
    TIME_STEP = 20
else:
    TIME_STEP = 20

# Distance sensor initialization
ds = []
ds_names = ['ds_fwd', 'ds_aft', 'ds_left', 'ds_right']
#for i in range(len(ds_names)):
#    ds.append(robot.getDevice(ds_names[i]))
#    ds[i].enable(TIME_STEP)

# LED light
# led_top = robot.getDevice('led_top')
# --> led.set(1) # Turns the led on

# COmmunication


# ----------------- AGV ---------------
# Create instances of the AGV wheel motors.
motor_rightFWD_AGV = robot.getDevice('motor_rightFWD_AGV')
motor_rightAFT_AGV = robot.getDevice('motor_rightAFT_AGV')
motor_leftFWD_AGV = robot.getDevice('motor_leftFWD_AGV')
motor_leftAFT_AGV = robot.getDevice('motor_leftAFT_AGV')

# Additional wheels aft
motor_rightAFT2_AGV = robot.getDevice('motor_rightAFT2_AGV')
motor_leftAFT2_AGV = robot.getDevice('motor_leftAFT2_AGV')

# Sets the initial position of the motor.
motor_rightFWD_AGV.setPosition(float('inf'))
motor_rightAFT_AGV.setPosition(float('inf'))
motor_leftFWD_AGV.setPosition(float('inf'))
motor_leftAFT_AGV.setPosition(float('inf'))

# Additional wheels aft
motor_rightAFT2_AGV.setPosition(float('inf'))
motor_leftAFT2_AGV.setPosition(float('inf'))

# Sets the initial velocity of the motor.
motor_rightFWD_AGV.setVelocity(0.0)
motor_rightAFT_AGV.setVelocity(0.0)
motor_leftFWD_AGV.setVelocity(0.0)
motor_leftAFT_AGV.setVelocity(0.0)

# Additional wheels aft
motor_rightAFT2_AGV.setVelocity(0.0)
motor_leftAFT2_AGV.setVelocity(0.0)

# Speed settings for AGV
max_velocity_agv = min(motor_rightFWD_AGV.getMaxVelocity(),motor_rightAFT_AGV.getMaxVelocity(),motor_leftFWD_AGV.getMaxVelocity(),motor_leftAFT_AGV.getMaxVelocity())
speed_agv = 0.5 * max_velocity_agv
SPEED_INCREMENT_AGV = 0.1

# ------------------- AXIS 1 --------------
# Get Axis 1 motor
motor_axis_1 = robot.getDevice('motor_axis_1')

# Sets the initial position of the motor.
motor_axis_1.setPosition(float('inf'))

# Sets the initial velocity of the motor.
motor_axis_1.setVelocity(0.0)

# Speed settings for Axis 1
max_velocity_axis_1 = motor_axis_1.getMaxVelocity()
speed_axis_1 = 2
SPEED_INCREMENT_AXIS_1 = 0.05

# ------------------- AXIS 2 --------------
# Get Axis 2 motor
motor_axis_2 = robot.getDevice('motor_axis_2')
axis_2_pos = 0

# Get position sensor for axis 2
ps_axis_2 = robot.getDevice('ps_axis_2')
ps_axis_2.enable(TIME_STEP)

# Speed settings for Axis 2
max_velocity_axis_2 = motor_axis_2.getMaxVelocity()
speed_axis_2 = 0.02
SPEED_INCREMENT_AXIS_2 = 0.02

# ------------------- AXIS 3 - Snake --------------
# Populate the snake robot
num_snake_joints = 7
snake_piece_length = 0.30         # Length in meter.
dist_snake_start_to_tip = 0.445  # Length in meter.
max_snake_length = 1.70 + dist_snake_start_to_tip    # Length in meter.

# Get Axis 3 motor - Snake part 1
#motor_axis_3_pt1 = robot.getDevice('motor_axis_3_pt1')
#axis_3_pos_pt1 = 0

# Get position sensor for axis 3 - Snake part 1
#ps_axis_3_pt1 = robot.getDevice('ps_axis_3_pt1')
#ps_axis_3_pt1.enable(TIME_STEP)

# Speed settings for Axis 3
speed_axis_3 = 0.005
SPEED_INCREMENT_AXIS_3 = 0.005

# ------------------- KEYBOARD --------------
# Enable the keyboard.
kb = Keyboard()
kb.enable(TIME_STEP)

# Keyboard values for AGV
FORWARD = '315'
BACKWARD = '317'
TURN_LEFT = '314'
TURN_RIGHT = '316'
SPEED_INCREASE_AGV = '43'
SPEED_DECREASE_AGV = '45'

# Keyboard values for Hasselhoff Hug - Axis 1
LEFT_AXIS_1 = '65'
RIGHT_AXIS_1 = '68'
SPEED_INCREASE_SNAKEBOX = str(Keyboard.CONTROL+Keyboard.SHIFT+43)
SPEED_DECREASE_SNAKEBOX = str(Keyboard.CONTROL+Keyboard.SHIFT+45)

# Keyboard values for Tower - Axis 2
UP_AXIS_2 = '87'
DOWN_AXIS_2 = '83'
SPEED_INCREASE_TOWER = str(Keyboard.CONTROL+43)
SPEED_DECREASE_TOWER = str(Keyboard.CONTROL+45)

# Keyboard values for Snake - Axis 3
EXTEND_AXIS_3 = '81'
RETRACT_AXIS_3 = '69'
SPEED_INCREASE_SNAKE = str(Keyboard.SHIFT+43)
SPEED_DECREASE_SNAKE = str(Keyboard.SHIFT+45)

# ------------------- METHODS --------------
def populate_snake(num_joints):
    global motor_axis_3_pt
    motor_axis_3_pt = [""] * num_joints
    for i in range(num_joints):
        motor_axis_3_pt[i] = "mtor_axis_3_pt" + str(i+1)
        print(motor_axis_3_pt[i])

# Method for driving AGV by controlling forward and aft wheel on left and right side
def drive_agv(left_fwd, left_aft, right_fwd, right_aft, speed):
    motor_rightFWD_AGV.setVelocity(right_fwd * speed)
    motor_rightAFT_AGV.setVelocity(right_aft * speed)
    motor_leftFWD_AGV.setVelocity(left_fwd * speed)
    motor_leftAFT_AGV.setVelocity(left_aft * speed)

# Method for rotating snake box - Axis 1
def drive_axis_1(left, right, speed):
    if (left):
        motor_axis_1.setVelocity(speed)
    elif (right):
        motor_axis_1.setVelocity(-speed)
    else:
        motor_axis_1.setVelocity(speed)

# Method to assign keys after keystrokes.
def key_assign(keystrokes):
    pass
    
# Switch case for robot control
def control_selection(argument):
    switcher = {
        1: "Manual",
        2: "Remote"
    }
    
    print(switcher.get(argument, "Invalid selection"))

def read_distance_sensor():
    # ------ DISTANCE SENSORS -----
    # read sensor outputs
    ds_values = []
    for i in range(len(ds_names)):
        pass
        # print(ds_names[i])
        # ds_values.append(ds[i].getValue())
    return ds_values

def read_keyboard():
    # ------ KEYBOARD -----
    # https://cyberbotics.com/doc/reference/keyboard
    # Register keystrokes
    keystrokes = [str(-1)] * 7
    for i in range(0, 7):
        keystrokes[i] = str(kb.getKey())
    # print(keystrokes)
    return keystrokes

def manual_mode():
    return "Manual mode"

def remote_mode():
    return "Remote mode"

def robot_mode(argument):
    switcher = {
        1: manual_mode,
        2: remote_mode
    }
    # Get the function from switcher dictionary
    func = switcher.get(argument, lambda: "Invalid robot mode")
    # Execute the function
    func()
    #print(func())

# ------ AGV -----
def increment_speed_agv(keystrokes):
    # Variables
    global speed_agv

    # Increment AGV speed
    if (SPEED_INCREASE_AGV in keystrokes):
        if (speed_agv < max_velocity_agv):
            speed_agv = round(speed_agv + SPEED_INCREMENT_AGV, 3)
            print("AGV speed increased to: ", speed_agv)
        else:
            speed_agv = max_velocity_agv
            sys.stderr.write("Maximum AGV speed reached.\n")
    elif (SPEED_DECREASE_AGV in keystrokes):
        if (speed_agv > 0):
            speed_agv = round(speed_agv - SPEED_INCREMENT_AGV, 3)
            print("AGV speed decreased to: ", speed_agv)
        else:
            speed_agv = 0
            sys.stderr.write("Minimum AGV speed reached.\n")

def move_agv(keystrokes):
    # Drive AGV
    if (FORWARD in keystrokes):  # Drive forwards
        if (TURN_LEFT in keystrokes):  # Turn left
            print("Turning left")
            drive_agv(left_fwd=0.2, left_aft=0.2, right_fwd=1, right_aft=1, speed=speed_agv)
        elif (TURN_RIGHT in keystrokes):  # Turn right
            print("Turning right")
            drive_agv(left_fwd=1, left_aft=1, right_fwd=0.2, right_aft=0.2, speed=speed_agv)
        else:
            drive_agv(left_fwd=1, left_aft=1, right_fwd=1, right_aft=1, speed=speed_agv)
    elif (BACKWARD in keystrokes):  # Drive backwards
        if (TURN_LEFT in keystrokes):  # Turn left
            print("Turning left")
            drive_agv(left_fwd=-0.2, left_aft=-0.2, right_fwd=-1, right_aft=-1, speed=speed_agv)
        elif (TURN_RIGHT in keystrokes):  # Turn right
            print("Turning right")
            drive_agv(left_fwd=-1, left_aft=-1, right_fwd=-0.2, right_aft=-0.2, speed=speed_agv)
        else:
            drive_agv(left_fwd=-1, left_aft=-1, right_fwd=-1, right_aft=-1, speed=speed_agv)
    elif (TURN_LEFT in keystrokes):  # Turn left
        print("Turning left")
        drive_agv(left_fwd=-1, left_aft=-1, right_fwd=1, right_aft=1, speed=speed_agv)
    elif (TURN_RIGHT in keystrokes):  # Turn right
        print("Turning right")
        drive_agv(left_fwd=1, left_aft=1, right_fwd=-1, right_aft=-1, speed=speed_agv)
    else:
        drive_agv(left_fwd=1, left_aft=1, right_fwd=1, right_aft=1, speed=0)

# ------ Snake box - 1 -----
def increment_speed_snakebox(keystrokes):
    # Variable
    global speed_axis_1

    # Increment Snake box - Axis 1 speed
    if (SPEED_INCREASE_SNAKEBOX in keystrokes):
        if (speed_axis_1 < max_velocity_axis_1):
            speed_axis_1 = round(speed_axis_1 + SPEED_INCREMENT_AXIS_1, 3)
            print("Snake box speed increased to: ", speed_axis_1)
        else:
            speed_axis_1 = max_velocity_axis_1
            sys.stderr.write("Maximum snake box speed reached.\n")
    elif (SPEED_DECREASE_SNAKEBOX in keystrokes):
        if (speed_axis_1 > 0):
            speed_axis_1 = round(speed_axis_1 - SPEED_INCREMENT_AXIS_1, 3)
            print("Snake box speed decreased to: ", speed_axis_1)
        else:
            speed_axis_1 = 0
            sys.stderr.write("Minimum snake box speed reached.\n")

def rotate_snakebox(keystrokes):
    # Variables
    global speed_axis_1

    # Rotate snake box - Axis 1
    if (LEFT_AXIS_1 in keystrokes):
        drive_axis_1(left=True, right=False, speed=speed_axis_1)
    elif (RIGHT_AXIS_1 in keystrokes):
        drive_axis_1(left=False, right=True, speed=speed_axis_1)
    else:
        drive_axis_1(left=False, right=False, speed=0)

# ------ Tower - Axis 2 -----
def increment_speed_tower(keystrokes):
    # Variables
    global speed_axis_2

    # Increment Tower speed
    if (SPEED_INCREASE_TOWER in keystrokes):
        speed_axis_2 = round(speed_axis_2 + SPEED_INCREMENT_AXIS_2, 3)
        print("Tower speed increased to: ", speed_axis_2)
    elif (SPEED_DECREASE_TOWER in keystrokes):
        speed_axis_2 = round(speed_axis_2 - SPEED_INCREMENT_AXIS_2, 3)
        print("Tower speed decreased to: ", speed_axis_2)

def change_tower_height(keystrokes):
    # Variables
    global axis_2_pos

    if (UP_AXIS_2 in keystrokes):
        axis_2_pos = round(ps_axis_2.getValue() + speed_axis_2, 2)
        print(axis_2_pos)
    elif (DOWN_AXIS_2 in keystrokes):
        axis_2_pos = round(ps_axis_2.getValue() - speed_axis_2, 2)
        print(axis_2_pos)

    if (axis_2_pos > motor_axis_2.getMaxPosition()):
        axis_2_pos = round(motor_axis_2.getMaxPosition(), 2)
        #print("Desired pos: ", axis_2_pos)
        #print("Maximum position: ", motor_axis_2.getMaxPosition())
        sys.stderr.write("Axis 2 has reached maximum height.\n")
    elif (axis_2_pos < motor_axis_2.getMinPosition()):
        axis_2_pos = round(motor_axis_2.getMinPosition(), 2)
        #print("Min. pos: ", motor_axis_2.getMaxPosition())
        sys.stderr.write("Axis 2 has reached minimum height.\n")
    else:
        motor_axis_2.setPosition(axis_2_pos)

# ------ Snake - Axis 3 -----
def increment_speed_snake(keystrokes):
    # Variable
    global speed_axis_3

    # Increment Snake - Axis 3 speed
    if (SPEED_INCREASE_SNAKE in keystrokes):
        if (speed_axis_3 < max_velocity_axis_3):
            speed_axis_3 = round(speed_axis_3 + SPEED_INCREMENT_AXIS_3, 3)
            print("Snake speed increased to: ", speed_axis_3)
        else:
            speed_axis_1 = max_velocity_axis_1
            sys.stderr.write("Maximum snake speed reached.\n")
    elif (SPEED_DECREASE_SNAKE in keystrokes):
        if (speed_axis_3 > 0):
            speed_axis_3 = round(speed_axis_3 - SPEED_INCREMENT_AXIS_3, 3)
            print("Snake speed decreased to: ", speed_axis_3)
        else:
            speed_axis_3 = 0
            sys.stderr.write("Minimum snake speed reached.\n")

def move_snake(keystrokes):
    # Variables
    global axis_3_pos_pt1

    # Move snake part 1
    if (EXTEND_AXIS_3 in keystrokes):
        axis_3_pos_pt1 = round(ps_axis_3_pt1.getValue() - SPEED_INCREMENT_AXIS_3, 3)
        print("Snake tip position: ", axis_3_pos_pt1)
    elif (RETRACT_AXIS_3 in keystrokes):
        axis_3_pos_pt1 = round(ps_axis_3_pt1.getValue() + SPEED_INCREMENT_AXIS_3, 3)
        print("Snake tip position: ", axis_3_pos_pt1)

    if (axis_3_pos_pt1 > motor_axis_3_pt1.getMaxPosition()):
        axis_3_pos_pt1 = round(motor_axis_3_pt1.getMaxPosition(), 3)
        print("Desired pos: ", axis_3_pos_pt1)
        print("Max. pos: ", motor_axis_3_pt1.getMaxPosition())
        sys.stderr.write("Axis 3 - Snake Part 1 - has reached maximum length.\n")
    elif (axis_3_pos_pt1 < motor_axis_3_pt1.getMinPosition()):
        axis_3_pos_pt1 = round(motor_axis_3_pt1.getMinPosition(), 3)
        print("Desired pos: ", axis_3_pos_pt1)
        print("Min. pos: ", motor_axis_3_pt1.getMaxPosition())
        sys.stderr.write("Axis 3 - Snake Part 1 - has reached minimum length.\n")
    else:
        motor_axis_3_pt1.setPosition(axis_3_pos_pt1)

def main():
    print("Hello World!")
    
    # Communication with TwinCAT/PLC over EtherCAT
    communication.set_ip("254.254.254.253")
    print("IP is: ", str((communication.get_ip())))
    
    while robot.step(TIME_STEP) != -1:
    
        # Robot mode selection
        mode_selection = 1
        robot_mode(mode_selection)
    
        # Read keyboard values
        keystrokes = read_keyboard()
    
        # ------ MOVE AGV -----
        # Increment AGV speed
        increment_speed_agv(keystrokes=keystrokes)
    
        # Move AGV
        move_agv(keystrokes=keystrokes)
    
        # ------ HASSELHOFF HUG / SNAKE BOX - AXIS 1 -----
        # Increment Axis 1 speed
        increment_speed_snakebox(keystrokes=keystrokes)
    
        # Rotate snake box - Axis 1
        rotate_snakebox(keystrokes=keystrokes)
    
        # ------ TOWER - AXIS 2 -----
        # Increment Tower speed
        increment_speed_tower(keystrokes=keystrokes)
    
        # Move tower height
        change_tower_height(keystrokes=keystrokes)
    
        # ------ SNAKE - AXIS 3 -----
        # Increment Snake speed
        increment_speed_snake(keystrokes=keystrokes)
    
        # Move snake part 1
        move_snake(keystrokes=keystrokes)

if __name__ == "__main__":
    print("This is my file to test Python's execution methods.")
    print("The variable __name__ tells me which context this file is running in.")
    print("The value of __name__ is:", repr(__name__))
    main()

# Main loop:
# - perform simulation steps until Webots is stopping the controller

        
    # ------ AXIS 2 -----
    #axis_2_dist = axis_2_dist + 0.5
    #axis_2.setVelocity(0.5)
    #if axis_2_dist < motor_axis_2.getMaxPosition():
    #    motor_axis_2.setPosition(axis_2_dist)
    #elif motor_axis_2.getPosition() < motor_axis_2.maxPosition():
    #    motor_axis_2.setPosition(axis_2_dist)
    #else:
    #    sys.stderr.write("Axis 2 has reached maximum height.\n")
        #pass
    #print("Axis 2 dist: ", axis_2_dist)