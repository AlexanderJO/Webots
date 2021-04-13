"""agv_controller controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot, DistanceSensor, Motor, Keyboard, GPS, Connector
import sys, math
from dataclasses import dataclass, field
from typing import List
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

# Communication

# Driving class
class Drive():
    def __init__(self):
        pass

# ----------------- BOX CONNECTORS ---------------
con_suction_cup = robot.getDevice('con_suction_cup')
con_suction_cup.enablePresence(TIME_STEP)

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

# ------------------- GPS Axis 1 - Axle --------------
# Get the GPS device
gps_axle = robot.getDevice('gps_axle_center')
gps_agv = robot.getDevice('gps_agv')
gps_axle.enable(TIME_STEP)
gps_agv.enable(TIME_STEP)

# ------------------- AXIS 1 - Snake box --------------
# Get Axis 1 motor
motor_axis_1 = robot.getDevice('motor_axis_1')

# Get position sensor for axis 2
ps_axis_1 = robot.getDevice('ps_axis_1')
ps_axis_1.enable(TIME_STEP)

# Sets the initial position of the motor.
motor_axis_1.setPosition(float('inf'))

# Sets the initial velocity of the motor.
motor_axis_1.setVelocity(0.0)

# Speed settings for Axis 1
max_velocity_axis_1 = motor_axis_1.getMaxVelocity()
speed_axis_1 = 2
SPEED_INCREMENT_AXIS_1 = 0.05

# ------------------- AXIS 2 - Tower --------------
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
    # Variables
    global motor_axis_3_pt, axis_3_pos_pt, ps_axis_3_pt, max_velocity_axis_3

    motor_axis_3_pt = [""] * num_joints
    axis_3_pos_pt = [0] * num_joints
    ps_axis_3_pt = [""] * num_joints

    max_velocity_axis_3 = 0

    for i in range(num_joints):
        # Get Axis 3 motor for each snake part
        motor_axis_3_pt[i] = robot.getDevice("motor_axis_3_pt" + str(i+1))
        axis_3_pos_pt[i] = 0

        # Get position sensor for axis 3 - Snake part 1
        ps_axis_3_pt[i] = robot.getDevice("ps_axis_3_pt" + str(i+1))
        ps_axis_3_pt[i].enable(TIME_STEP)

        # Get the maximum motor velocity.
        motor_velocity = motor_axis_3_pt[i].getMaxVelocity()

        # Set the maximum velocity.
        if motor_velocity > max_velocity_axis_3:
            max_velocity_axis_3 = motor_velocity

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

# Method to assign keys after keystrokes. - REDACTED
def key_assign(keystrokes):
    pass

# Read the distance sensor.
def read_distance_sensor():
    # ------ DISTANCE SENSORS -----
    # read sensor outputs
    ds_values = []
    for i in range(len(ds_names)):
        pass
        # print(ds_names[i])
        # ds_values.append(ds[i].getValue())
    return ds_values

# Takes in a single keyboard input from user.
def keyboard_input_int(prompt):
    try:
        text = input(prompt)
    except:
        raise Exception('Input not an integer.')

# Continuously reads the keyboard for user inputs.
# Reads up to 7 simultaneous/combined key presses.
def read_keyboard():
    # ------ KEYBOARD -----
    # https://cyberbotics.com/doc/reference/keyboard
    # Register keystrokes
    keystrokes = [str(-1)] * 7
    for i in range(0, 7):
        keystrokes[i] = str(kb.getKey())
    # print(keystrokes)
    return keystrokes

# ------ Switch case -----
def manual_mode():
    return "Manual mode"

def remote_mode():
    return "Remote mode"

def automatic_mode():
    return "Automatic mode"

# Set the type of robot mode.
def robot_mode(argument):
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
def increment_speed_snake_manual(keystrokes):
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

# Returns the angle in degrees (°) from two coordinate systems.
def get_heading(coordinates_inner, coordinates_outer):
    diff_x = coordinates_outer['gps_pos'][0] - coordinates_inner['gps_pos'][0]
    diff_y = coordinates_outer['gps_pos'][1] - coordinates_inner['gps_pos'][1]
    diff_z = coordinates_outer['gps_pos'][2] - coordinates_inner['gps_pos'][2]

    # Calculates the angle in degrees (°).
    angle = math.atan2(diff_x, diff_z) * (360/(2*math.pi))
    if angle < 0:
        angle = angle + 360

    # Returns the angle in degrees (°).
    return angle

# Return GPS position in x, y and z format as a dictionary.
def get_gps_pos(name):
    # Read GPS position
    gps_info = dict()
    gps_info['gps_pos'] = name.getValues()
    return gps_info

# Returns the angular position in degrees (°) for a position sensor (name).
def get_angular_position(name):
    angle_rad = name.getValue()

    # Calculates the angle in degrees (°).
    angle_deg = angle_rad * (360 / (2 * math.pi))
    if angle_deg < 0:
        angle = angle_deg + 360

    # Returns the angle in degrees (°).
    return angle_deg

def move_snake(keystrokes):
    # Variables
    global motor_axis_3_pt, axis_3_pos_pt, ps_axis_3_pt

    rem_length = round(axis_3_pos_pt[0], 3) % snake_piece_length
    count = round(abs((round(axis_3_pos_pt[0], 4) - rem_length + snake_piece_length) / snake_piece_length), 0)
    # print("Overshooting length: ", rem_length, " at ", count, " pieces")




    for i in range(len(motor_axis_3_pt)):
        # if count == i:
        #     pos = i + 1
        #
        #     # Move snake part
        #     if (EXTEND_AXIS_3 in keystrokes):
        #         axis_3_pos_pt[i] = round(ps_axis_3_pt[i].getValue() - speed_axis_3, 3)
        #     elif (RETRACT_AXIS_3 in keystrokes):
        #         axis_3_pos_pt[i] = round(ps_axis_3_pt[i].getValue() + speed_axis_3, 3)
        #
        #     if (axis_3_pos_pt[i] > motor_axis_3_pt[i].getMaxPosition()):
        #         print("Desired pos: ", axis_3_pos_pt[i])
        #         axis_3_pos_pt[i] = round(motor_axis_3_pt[i].getMaxPosition(), 3)
        #         # print("Max. pos: ", motor_axis_3_pt[i].getMaxPosition())
        #         sys.stderr.write("Axis 3 - Snake Part " + str(pos) + " has reached maximum length.")
        #     elif (axis_3_pos_pt[i] < motor_axis_3_pt[i].getMinPosition()):
        #         print("Desired pos: ", axis_3_pos_pt[i])
        #         axis_3_pos_pt[i] = round(motor_axis_3_pt[i].getMinPosition(), 3)
        #         # print("Min. pos: ", motor_axis_3_pt[i].getMaxPosition())
        #         sys.stderr.write("Axis 3 - Snake Part " + str(pos) + " - has reached minimum length.")
        #     else:
        #         motor_axis_3_pt[i].setPosition(axis_3_pos_pt[i])
        #
        #     print("Snake tip ", (i + 1), " position: ", axis_3_pos_pt[i])

        if count >= i:
            pos = i + 1

            # Move snake part
            if (EXTEND_AXIS_3 in keystrokes):
                axis_3_pos_pt[i] = round(ps_axis_3_pt[i].getValue() - speed_axis_3, 3)
                print("Snake tip ", (i + 1), " position: ", axis_3_pos_pt[i])
            elif (RETRACT_AXIS_3 in keystrokes):
                axis_3_pos_pt[i] = round(ps_axis_3_pt[i].getValue() + speed_axis_3, 3)
                print("Snake tip ", (i + 1), " position: ", axis_3_pos_pt[i])

            if (axis_3_pos_pt[i] > motor_axis_3_pt[i].getMaxPosition()):
                print("Desired pos: ", axis_3_pos_pt[i])
                axis_3_pos_pt[i] = round(motor_axis_3_pt[i].getMaxPosition(), 3)
                sys.stderr.write("Axis 3 - Snake Part " + str(pos) + " has reached maximum length.")
            elif (axis_3_pos_pt[i] < motor_axis_3_pt[i].getMinPosition()):
                print("Desired pos: ", axis_3_pos_pt[i])
                axis_3_pos_pt[i] = round(motor_axis_3_pt[i].getMinPosition(), 3)
                sys.stderr.write("Axis 3 - Snake Part " + str(pos) + " - has reached minimum length.")
            else:
                motor_axis_3_pt[i].setPosition(axis_3_pos_pt[i])
        pass

def move_snake_manual(keystrokes):
    # Variables
    # global motor_axis_3_pt, axis_3_pos_pt, ps_axis_3_pt

    # # Move snake part 1
    # if (EXTEND_AXIS_3 in keystrokes):
    #     axis_3_pos_pt[0] = round(ps_axis_3_pt[0].getValue() - speed_axis_3, 3)
    #     print("Snake tip position: ", axis_3_pos_pt[0])
    # elif (RETRACT_AXIS_3 in keystrokes):
    #     axis_3_pos_pt[0] = round(ps_axis_3_pt[0].getValue() + speed_axis_3, 3)
    #     print("Snake tip position: ", axis_3_pos_pt[0])
    #
    # if (axis_3_pos_pt[0] > motor_axis_3_pt[0].getMaxPosition()):
    #     axis_3_pos_pt[0] = round(motor_axis_3_pt[0].getMaxPosition(), 3)
    #     print("Desired pos: ", axis_3_pos_pt[0])
    #     print("Max. pos: ", motor_axis_3_pt[0].getMaxPosition())
    #     sys.stderr.write("Axis 3 - Snake Part 1 - has reached maximum length.\n")
    # elif (axis_3_pos_pt[0] < motor_axis_3_pt[0].getMinPosition()):
    #     axis_3_pos_pt[0] = round(motor_axis_3_pt[0].getMinPosition(), 3)
    #     print("Desired pos: ", axis_3_pos_pt[0])
    #     print("Min. pos: ", motor_axis_3_pt[0].getMaxPosition())
    #     sys.stderr.write("Axis 3 - Snake Part 1 - has reached minimum length.\n")
    # else:
    #     motor_axis_3_pt[0].setPosition(axis_3_pos_pt[0])

    move_snake(keystrokes)

def manual_control_keyboard(keystrokes):

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
    increment_speed_snake_manual(keystrokes=keystrokes)

    # Move snake part 1
    move_snake_manual(keystrokes=keystrokes)

def snake_tip_kinematics(agv_heading, snake_angle, snake_tip_pos):
    pass

def get_snake_tip_globale_coord(agv_heading, snake_angle, agv_gps, snake_tip_pos):
    angle = agv_heading + snake_angle

    pass

def circle_coordinates(length, angle):
    coordinates = [0] * 2
    coordinates[0] = length * math.sin(math.radians(angle))
    coordinates[1] = length * math.cos(math.radians(angle))

    return coordinates

def main():
    # Variables
    # agv_heading = 0
    # snake_box_angle = 0
    # snake_angle = 0
    
    # Create the snake and setup the linear motors
    populate_snake(num_snake_joints)
    
    
    # Communication with TwinCAT/PLC over EtherCAT
    #communication.set_ip("254.254.254.253")
    #print("IP is: ", str((communication.get_ip())))

    # Robot mode selection
    mode_selection = 1
    mode = robot_mode(mode_selection)

    # Main loop:
    # - perform simulation steps until Webots is stopping the controller
    while robot.step(TIME_STEP) != -1:

        # Connector for connecting to boxes
        # print("Connector presence: ", con_suction_cup.getPresence())
        if (con_suction_cup.getPresence() == 1):
            # print("Connector presence")
            con_suction_cup.lock()

        # Get GPS position
        gps_axle_info = get_gps_pos(name=gps_axle)
        #print("Axle GPS:", "X: ", round(gps_axle_info['gps_pos'][0], 6), "Y: ", round(gps_axle_info['gps_pos'][1], 6), "Z: ", round(gps_axle_info['gps_pos'][2], 6))
        
        gps_agv_info = get_gps_pos(name=gps_agv)
        #print("AGV GPS:", "X: ", round(gps_agv_info['gps_pos'][0], 6), "Y: ", round(gps_agv_info['gps_pos'][1], 6), "Z: ", round(gps_agv_info['gps_pos'][2], 6))
        
        # Get heading of AGV
        agv_heading = get_heading(coordinates_inner=gps_axle_info, coordinates_outer=gps_agv_info)
        print("Heading of AGV: ", agv_heading, "°")

        # Get snake box angle
        angle_axis_1 = get_angular_position(name=ps_axis_1)
        print("Snake box angle: ", angle_axis_1)

        snake_length = axis_3_pos_pt[0]
        print("Snake length: ", snake_length)

        # angle_snake =
        snake_coordinates = circle_coordinates(length=snake_length, angle=angle_axis_1)
        print("Snake coordinates: ", snake_coordinates)












        # Runs either manual by keyboard input, or in automatic or remote mode.
        if mode == 'Manual mode':
            # Read keyboard values
            keystrokes = read_keyboard()
            manual_control_keyboard(keystrokes=keystrokes)
        elif mode == "Automatic mode":
            pass
        elif mode == "Remote mode":
            pass
        else:
            # Print error
            sys.stderr.write("No or incorrect mode selected.\n")

            # The program will exit
            sys.exit("The program will now be terminated.")
            pass

# Method to start the program by running the main. Python override.
if __name__ == "__main__":
    #print("This is my file to test Python's execution methods.")
    #print("The variable __name__ tells me which context this file is running in.")
    #print("The value of __name__ is:", repr(__name__))
    main()