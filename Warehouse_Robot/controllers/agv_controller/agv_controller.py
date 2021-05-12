"""agv_controller controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot, DistanceSensor, Motor, Keyboard, GPS, Connector, Camera
import sys, math, struct
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
# class Drive():
#     def __init__(self):
#         pass

class AGV(Robot):
    # get the time step of the current world.
    TIME_STEP = int(robot.getBasicTimeStep())
    if TIME_STEP < 32:
        TIME_STEP = 32
    else:
        TIME_STEP = 32

    # ----------------- BOX CONNECTORS ---------------
    con_suction_cup = robot.getDevice('con_suction_cup')
    con_suction_cup.enablePresence(TIME_STEP)

    # ----------------- COMMUNICATION ---------------
    # Instantiate receiver variable to hold receiver.
    receiver = None

    # Instantiate emitter variable to hold emitter.
    emitter = None

    # Message sent by emitter.
    emitted_message = ""
    previous_message = ""

    # ----------------- AGV ---------------
    # Status variables
    agv_approach_heading_reached = False
    agv_attack_heading_reached = False
    agv_heading_reached = False
    agv_positioned = [False for i in range(3)] # Positioned AGV in x, y and z position.
    agv_x_pos_reached = False
    agv_y_pos_reached = True
    agv_z_pos_reached = False
    agv_pos_reached = False
    agv_moving = False
    packet_picking_started = False
    packet_picking_ended = False
    packet_attached = False
    packet_placed = False
    deliver_on_agv = False
    go_idle = False
    packet_created = False
    gripper_pos_reached = False
    robot_ready = False
    go_init_pos = False

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
    max_velocity_agv = min(motor_rightFWD_AGV.getMaxVelocity(), motor_rightAFT_AGV.getMaxVelocity(),
                           motor_leftFWD_AGV.getMaxVelocity(), motor_leftAFT_AGV.getMaxVelocity())
    speed_agv = 0.5 * max_velocity_agv
    SPEED_INCREMENT_AGV = 0.1

    # ------------------- GPS Axis 1 - Axle --------------
    # Get the GPS device
    gps_axle = robot.getDevice('gps_axle_center')
    gps_agv = robot.getDevice('gps_agv')
    gps_axle.enable(TIME_STEP)
    gps_agv.enable(TIME_STEP)

    # ------------------- AXIS 1 - Snake box --------------
    # Status variables
    snakebox_pos_reached = False

    # Get Axis 1 motor
    motor_axis_1 = robot.getDevice('motor_axis_1')

    # Get position sensor for axis 1
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
    # Status variables
    tower_pos_reached = False

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
    # Status variables
    snake_pos_reached = False
    snake_extending = False
    snake_retracting = False

    # Populate the snake robot
    num_snake_joints = 7
    snake_piece_length = 0.30  # Length in meter.
    dist_snake_start_to_tip = 0.445  # Length in meter.
    max_snake_length = 1.70 + dist_snake_start_to_tip  # Length in meter.

    # Get Axis 3 motor - Snake part 1
    # motor_axis_3_pt1 = robot.getDevice('motor_axis_3_pt1')
    # axis_3_pos_pt1 = 0

    # Snake settings
    motor_axis_3_pt = None
    axis_3_pos_pt = None
    ps_axis_3_pt = None

    # Get position sensor for axis 3 - Snake part 1
    # ps_axis_3_pt1 = robot.getDevice('ps_axis_3_pt1')
    # ps_axis_3_pt1.enable(TIME_STEP)

    # Speed settings for Axis 3
    speed_axis_3 = 0.005
    SPEED_INCREMENT_AXIS_3 = 0.005
    max_velocity_axis_3 = 0

    # ------------------- AXIS 4 - Snake tip --------------
    # Status variables
    snaketip_pos_reached = False

    # Get Axis 4 motor
    motor_axis_4 = robot.getDevice('motor_axis_4')

    # Get position sensor for axis 4
    ps_axis_4 = robot.getDevice('ps_axis_4')
    ps_axis_4.enable(TIME_STEP)

    # Sets the initial position of the motor.
    motor_axis_4.setPosition(float('inf'))

    # Sets the initial velocity of the motor.
    motor_axis_4.setVelocity(0.0)

    # Speed settings for Axis 4
    max_velocity_axis_4 = motor_axis_4.getMaxVelocity()
    speed_axis_4 = 2
    SPEED_INCREMENT_AXIS_4 = 0.05

    # # ------------------- KEYBOARD --------------
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
    SPEED_INCREASE_SNAKEBOX = str(Keyboard.CONTROL + Keyboard.SHIFT + 43)
    SPEED_DECREASE_SNAKEBOX = str(Keyboard.CONTROL + Keyboard.SHIFT + 45)

    # Keyboard values for Tower - Axis 2
    UP_AXIS_2 = '87'
    DOWN_AXIS_2 = '83'
    SPEED_INCREASE_TOWER = str(Keyboard.CONTROL + 43)
    SPEED_DECREASE_TOWER = str(Keyboard.CONTROL + 45)

    # Keyboard values for Snake - Axis 3
    EXTEND_AXIS_3 = '81'
    RETRACT_AXIS_3 = '69'
    SPEED_INCREASE_SNAKE = str(Keyboard.SHIFT + 43)
    SPEED_DECREASE_SNAKE = str(Keyboard.SHIFT + 45)

    # Keyboard values for Snake tip - Axis 4
    LEFT_AXIS_4 = '90'
    RIGHT_AXIS_4 = '67'

    # Keyboard values for Packet handling
    PACKET_ATTACH = '32'
    PACKET_DETACH = '65568'

    # ------------------- CAMERA --------------
    camera_top = None
    camera_snake = None

    # ------------------- STATE MACHINE VARIABLES --------------
    state = 0
    state_agv = 0

    # ------------------- PACKET VARIABLES --------------
    packet_dimensions = None
    pallet_list = None
    packet_list = None
    packet_num = 0
    current_gps_pos = None
    previous_gps_pos = None

    def __init__(self):
        # Instantiate receiver node to obtain data from emitters.
        self.receiver = robot.getDevice('receiver_agv')
        self.receiver.enable(self.TIME_STEP)

        # Instantiate emitter node to send data to receivers.
        self.emitter = robot.getDevice('emitter_agv')

        # Instantiate cameras.
        self.camera_top = robot.getDevice('camera_top')
        self.camera_top.enable(self.TIME_STEP)
        self.camera_snake = robot.getDevice('camera_snake')
        self.camera_snake.enable(self.TIME_STEP)

        # AGV position initialized.
        self.agv_positioned[1] = True # No movements in y-direction

        # Create pallet on AGV.
        self.create_pallet(1)

        # Run the program.
        self.run()

    # ----------------- COMMUNICATION ---------------
    # Receive message through receiver sent from emitter.
    # Received in utf-8 format.
    def receive_message(self):
        message = ""
        if self.receiver.getQueueLength() > 0:
            message = self.receiver.getData().decode('utf-8')
            self.receiver.nextPacket()

        # Splits the message.
        message = message.split()

        return message

    # Send message from emitter.
    # Sent in utf-8 format.
    def send_message(self, message):
        # message_sent = False
        if message != '':  # and message != self.previous_message:
            self.previous_message = message
            # message_packed = struct.pack(message)
            # self.emitter.send(message_packed)
            self.emitter.send(message.encode('utf-8'))
            # message_sent = True
        # return message_sent

    def send_message_struct(self, message_list):
        # messages = [i for i in range(0,7)]
        # buffer = struct.pack('%sf' % len(message_list), *message_list)
        # var = struct.pack('hhl', 'test')
        s = struct.Struct('4s 4s 4s 4s 4s 4s 4s')
        packed = s.pack(message_list.encode('utf-8'))
        return packed

    # ------------------- METHODS --------------
    def create_pallet(self, num_pallets):
        self.pallet_list = [""] * num_pallets

        for i in range(num_pallets):
            pallet_name = "AGV_PALLET_" + str(i+1)
            pallet = Pallet(length=1.200, width=0.800, height=0.155)
            self.pallet_list[i] = pallet
            print("Pallet ", pallet_name, " placed on AGV.")

    def populate_snake(self, num_joints):
        # Variables
        # global motor_axis_3_pt, axis_3_pos_pt, ps_axis_3_pt

        self.motor_axis_3_pt = [""] * num_joints
        self.axis_3_pos_pt = [0] * num_joints
        self.ps_axis_3_pt = [""] * num_joints

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
    def drive_agv(self, left_fwd, left_aft, right_fwd, right_aft, speed):
        self.motor_rightFWD_AGV.setVelocity(right_fwd * speed)
        self.motor_rightAFT_AGV.setVelocity(right_aft * speed)
        self.motor_leftFWD_AGV.setVelocity(left_fwd * speed)
        self.motor_leftAFT_AGV.setVelocity(left_aft * speed)

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

# Dataclass for Packet with immutable input.
@dataclass(frozen=True, order=True)
class Packet:
    length: int = field()
    width: int = field()
    height: int = field()
    size: str = field(default="NO SIZE SELECTED!")

class Pallet:
    length = int(0)
    width = int(0)
    height = int(0)
    packets = []

    def __init__(self, length, height, width):
        self.length = length
        self.width = height
        self.height = width

    # Private function for setting length. Only used for testing. Not callable from outside class.
    def __set_length(self, length):
        self.length = length

    def add_packet(self, packet):
        self.packets.append(packet)

    def remove_packet(self, packet):
        pass

    def get_packets(self):
        return self.packets

def main():
    # Variables
    # agv_heading = 0
    # snake_box_angle = 0
    # snake_angle = 0
    
    # Create the snake and setup the linear motors
    def run(self):
        # Set robot in ready for operation.
        self.robot_ready = True

        # Variables
        # agv_heading = 0
        # snake_box_angle = 0
        # snake_angle = 0

        # Create the snake and setup the linear motors
        self.populate_snake(self.num_snake_joints)


        # Communication with TwinCAT/PLC over EtherCAT
        #communication.set_ip("254.254.254.253")
        #print("IP is: ", str((communication.get_ip())))
        manual_message = ['None'] * 1  # Clear previous message.
        remote_message = ['None'] * 3  # Clear previous message.
        remote_message[2] = 'True' # Sets the robot as ready.

        # Set initial GPS values for storing of updated packet positions.
        self.current_gps_pos = [0, 0, 0]
        self.previous_gps_pos = [0, 0, 0]

        # Robot mode selection
        mode_selection = 1
        mode = self.robot_mode(mode_selection)

        # Margins
        tower_margin = 0.001
        snakebox_margin = 2
        gripper_margin = 2

        # State machine init
        self.state_agv = 3

        # Main loop:
        # - perform simulation steps until Webots is stopping the controller
        while robot.step(self.TIME_STEP) != -1:
            # Connector for connecting to boxes
            # print("Connector presence: ", con_suction_cup.getPresence())
            # if (self.con_suction_cup.getPresence() == 1):
            #     # print("Connector presence")
            #     self.con_suction_cup.lock()

            # Get GPS position
            # print("Heading of AGV: ", self.get_agv_heading())

            # Get snake box angle
            # angle_axis_1 = self.get_angular_position(name=self.ps_axis_1)
            # print("Snake box angle: ", angle_axis_1)

            # snake_length = self.axis_3_pos_pt[0]
            # print("Snake length: ", snake_length)

            # angle_snake =
            # snake_coordinates = self.circle_coordinates(length=snake_length, angle=angle_axis_1)
            # x = self.get_gps_pos(name=self.gps_axle)['gps_pos'][0] - snake_coordinates[0] + 0.16/2
            # z = self.get_gps_pos(name=self.gps_axle)['gps_pos'][2] - snake_coordinates[1]

            # print("X: ", x)
            # print("Z: ", z)
            # print("Axle position: ", self.get_gps_pos(name=self.gps_axle))
            # print("Snake coordinates: ", snake_coordinates)

            # print("==========               ============")
            # for packet in range(len(pallet1.get_packets())):
            #     packet_current = pallet1.get_packets()[packet]
            #     print("This should be 300: ", packet_current.width)
            # print("======================")






            # Runs either manual by keyboard input, or in automatic or remote mode.
            if mode == 'Manual mode':
                # Communication
                # Receive message from emitter
                message = self.receive_message()
                # print("Message received AGV:", message)

                # Read keyboard values
                keystrokes = self.read_keyboard()

                # print("Keystroke class: ", type(keystrokes))
                self.manual_control_keyboard(keystrokes=keystrokes)

                if (self.PACKET_ATTACH in keystrokes):
                    manual_message[0] = 'attach'
                elif (self.PACKET_DETACH in keystrokes):
                    manual_message[0] = 'detach'

                # message = self.receive_message()
                # print("Message received AGV:", message)

                self.emitted_message = self.list_to_string(manual_message)

                # print("Am I close to desired heading?: ",)
            elif mode == "Remote mode":
                # Communication
                # Receive message from emitter
                message = self.receive_message()
                print("Message received AGV:", message)




                if self.robot_ready:
                    # self.move_agv(message[0])

                    if message[0] == 'idle' and self.packet_picking_started:
                        print("AGV going to idle position.")
                        self.go_init_pos = True
                    else:
                        # Variables for packet handling.
                        packet_pos = [float for i in range(3)]
                        packet_pos_new = [float for i in range(3)]

                        # Get packet position
                        packet_pos[0] = float(message[8])
                        packet_pos[1] = float(message[9])
                        packet_pos[2] = float(message[10])


                    # print("Packet position to be handled: ", packet_pos)

                    # Get current AGV position.
                    agv_pos = self.get_gps_pos(name=self.gps_axle)['gps_pos']
                    # print("AGV positon: ", agv_pos)

                    # Get actual snake tip heading.
                    snaketip_angle = self.snaketip_angle()
                    # print("Actual snaketip angle: ", snaketip_angle)

                    # Get the angle of attack for the snake tip.
                    snaketip_attack_angle = self.get_angle_between_coords(agv_pos, packet_pos)
                    # print("Snake tip angle: ", snaketip_angle, " Attack angle: ", snaketip_attack_angle)
                    axis_4_angle = self.get_angular_position(self.ps_axis_4)
                    remote_message[1] = (snaketip_angle + axis_4_angle)

                    snake_length = self.get_snake_length((360 - snaketip_attack_angle), agv_pos, packet_pos) + 0.08
                    # print("Snake length attack: ", snake_length)

                    # self.state_machine_agv(self.state_agv)
                    # self.turn_agv(90)
                    # Move AGV to desired position.
                    print("Packet position: ", packet_pos)
                    if not all(self.agv_positioned) and not self.packet_picking_started:
                        self.speed_agv = 0.5 * self.max_velocity_agv
                        self.move_agv_to(0, 0, packet_pos[2])

                    elif all(self.agv_positioned) and not self.packet_attached and not self.deliver_on_agv and not self.go_idle and not self.snake_extending:
                        self.speed_agv = 0.1 * self.max_velocity_agv
                        self.packet_picking_started = True
                        self.move_agv_to(0, 0, packet_pos[2])

                        if not self.tower_pos_reached and not self.snake_pos_reached and not self.agv_moving:
                            desired_tower_height = packet_pos[1] - 0.2798 + 0.2
                            self.move_tower_to(desired_tower_height)
                            if (desired_tower_height >= (self.ps_axis_2.getValue() - tower_margin) and desired_tower_height <= (self.ps_axis_2.getValue() + tower_margin)):
                                self.tower_pos_reached = True
                                print("Tower positon reached!")

                        if not self.snakebox_pos_reached and self.tower_pos_reached:
                            desired_snakebox_angle = snaketip_attack_angle - 270
                            print("Rotating snakebox")
                            self.rotate_snakebox_to(desired_snakebox_angle)

                            print("Reading angle: ", math.degrees(self.ps_axis_1.getValue()))

                            if (desired_snakebox_angle >= (math.degrees(self.ps_axis_1.getValue()) - snakebox_margin)) and \
                                    (desired_snakebox_angle <= (math.degrees(self.ps_axis_1.getValue()) + snakebox_margin)):
                                self.snakebox_pos_reached = True

                        if not self.snake_pos_reached and self.snakebox_pos_reached:
                            self.move_snake_to(snake_length)
                            if self.snake_pos_reached:
                                self.tower_pos_reached = False

                        if not self.tower_pos_reached and self.snake_pos_reached:
                            desired_tower_height = packet_pos[1] - 0.2798
                            self.move_tower_to(desired_tower_height)

                            if (desired_tower_height >= (
                                    self.ps_axis_2.getValue() - tower_margin) and desired_tower_height <= (
                                    self.ps_axis_2.getValue() + tower_margin)):
                                self.tower_pos_reached = True
                                print("Tower position reached!")

                            if self.tower_pos_reached:
                                remote_message[0] = 'attach'

                        print("Lowered message", message[11].lower())
                        if (message[11].lower() == 'true'):
                            print("It is connected!")
                            self.packet_attached = True

                        if self.packet_attached and self.tower_pos_reached:
                            # Reseting status variables
                            self.reset_pick_packet_variables()
                            print("Variables reset!")
                            self.snake_retracting = True


                    elif self.packet_attached and self.packet_picking_started and not self.go_idle:
                        print("Packet attached.")

                        if not self.deliver_on_agv:

                            if not self.packet_created:
                                self.packet_num += 1
                                # self.packet_dimensions = [0, 0, 0]
                                # self.packet_dimensions[0] = float(message[16]) # Width
                                # self.packet_dimensions[1] = float(message[17]) # Length
                                # self.packet_dimensions[2] = float(message[18]) # Height
                                width = float(message[16]) # Width
                                length = float(message[17]) # Length
                                height = float(message[18]) # Height
                                packet_size = message[19] # Size of packet.

                                packet = Packet(width=width, length=length, height=height, size=packet_size)
                                self.pallet_list[0].add_packet(packet=packet)
                                print("Packet ", self.pallet_list[0].get_packet(self.packet_num), " has been added on AGV pallet.")

                                self.packet_created = True

                            # print("Packet dimensions: ",
                            #       "Width = ", self.pallet_list[0].get_packet(self.packet_num).get_width(),
                            #       "Length = ", self.pallet_list[0].get_packet(self.packet_num).get_length(),
                            #       "Height = ", self.pallet_list[0].get_packet(self.packet_num).get_height())

                            packet_coordinates = self.get_packet_position(pallet_num=0, packet_num=self.packet_num)
                            print("Packet position on pallet: ", self.pallet_list[0].get_packet_position(self.packet_num))
                            print("Packet position in LCS: ", packet_coordinates)

                            snaketip_coordinates = self.snake_tip_kinematics(packet_pos=packet_coordinates)
                            print("Snake tip coordinates:", snaketip_coordinates)

                            # Get new preliminary packet position
                            # packet_pos_new[0] = float(message[12])
                            # packet_pos_new[1] = float(message[13])
                            # packet_pos_new[2] = float(message[14])
                            # print("New packet position: ", packet_pos_new)

                            if self.snake_retracting and not self.snake_extending and not self.deliver_on_agv:
                                if not self.tower_pos_reached:
                                    desired_tower_height = packet_pos[1] - 0.2798 + 0.2
                                    self.move_tower_to(desired_tower_height)
                                    if (desired_tower_height >= (self.ps_axis_2.getValue() - tower_margin) and desired_tower_height <= (self.ps_axis_2.getValue() + tower_margin)):
                                        self.tower_pos_reached = True
                                        print("Tower position reached!")

                                if not self.snake_pos_reached and self.tower_pos_reached:
                                    self.move_snake_to(0)
                                    if self.snake_pos_reached:
                                        print("Snake retracted!")
                                        # self.reset_retracting_variables()
                                        # self.snake_extending = True

                            # elif self.snake_extending and not self.snake_retracting and not self.deliver_on_agv:
                                # if not self.tower_pos_reached:
                                #     desired_tower_height = packet_pos[1] - 0.2798 + 0.2
                                #     self.move_tower_to(desired_tower_height)
                                #     if (desired_tower_height >= (self.ps_axis_2.getValue() - tower_margin) and desired_tower_height <= (self.ps_axis_2.getValue() + tower_margin)):
                                #         self.tower_pos_reached = True
                                #         print("Tower position reached!")

                                # if not self.snake_pos_reached and self.tower_pos_reached:
                                #     self.move_snake_to(0)
                                #     print("Snake retracted!")

                                if self.snake_pos_reached and self.tower_pos_reached:
                                    self.deliver_on_agv = True
                                    self.snake_extending = True

                                    # Reseting variables.
                                    self.reset_retracting_variables()


                        elif self.deliver_on_agv and self.packet_picking_started:
                            print("Delivering packet to AGV!")

                            # Get the angle of attack for the snake tip.
                            snaketip_attack_angle = snaketip_coordinates[1] + 90
                            # snaketip_attack_angle = self.get_angle_between_coords(packet_coordinates, [0,0,0]) + 90
                            # snaketip_attack_angle = self.get_angle_between_coords(agv_pos, packet_pos_new)
                            print("New snake tip angle: ", snaketip_coordinates[1], " New attack angle: ", snaketip_attack_angle)
                            # snake_length_new = self.get_snake_length((snaketip_attack_angle), packet_coordinates, [0,0,0]) + 0.08
                            # snake_length_new = self.get_snake_length((snaketip_attack_angle), agv_pos, packet_pos_new)
                            if self.snake_extending and not self.snake_retracting:
                                if not self.tower_pos_reached and not self.snake_pos_reached:
                                    desired_tower_height = snaketip_coordinates[2] + 0.2
                                    # desired_tower_height = packet_coordinates[1] + 0.2798 + 0.2 # - 0.83 + 0.2
                                    self.move_tower_to(desired_tower_height)
                                    if (desired_tower_height >= (self.ps_axis_2.getValue() - tower_margin) and desired_tower_height <= (self.ps_axis_2.getValue() + tower_margin)):
                                        self.tower_pos_reached = True
                                        print("Tower position reached!")
                                if not self.snakebox_pos_reached and self.tower_pos_reached:
                                    desired_snakebox_angle = snaketip_attack_angle - 180
                                    print("Desired angle: ", desired_snakebox_angle, " Attack angle: ", snaketip_attack_angle)
                                    # print("Rotating snakebox")
                                    self.rotate_snakebox_to(desired_snakebox_angle)
                                    print("Reading angle: ", math.degrees(self.ps_axis_1.getValue()))
                                    if (desired_snakebox_angle >= (
                                            math.degrees(self.ps_axis_1.getValue()) - snakebox_margin)) and \
                                            (desired_snakebox_angle <= (math.degrees(self.ps_axis_1.getValue()) + snakebox_margin)):
                                        self.snakebox_pos_reached = True

                                if not self.snake_pos_reached and self.snakebox_pos_reached:
                                    self.move_snake_to(snaketip_coordinates[0])
                                    if self.snake_pos_reached:
                                        self.tower_pos_reached = False

                                if not self.gripper_pos_reached and self.snake_pos_reached:
                                    desired_gripper_angle = 360 - snaketip_coordinates[1]
                                    print("Gripper angle: ", desired_gripper_angle)

                                    self.rotate_snaketip_to(angle=desired_gripper_angle, speed=2)

                                    if (desired_gripper_angle >= (
                                            math.degrees(self.ps_axis_4.getValue()) - gripper_margin)) and \
                                            (desired_gripper_angle <= (math.degrees(self.ps_axis_4.getValue()) + gripper_margin)):
                                        self.gripper_pos_reached = True

                                if not self.tower_pos_reached and self.snake_pos_reached and self.gripper_pos_reached:
                                    desired_tower_height = snaketip_coordinates[2]
                                    # desired_tower_height = packet_coordinates[1] + 0.2798 # - 0.83
                                    self.move_tower_to(desired_tower_height)

                                    if (desired_tower_height >= (self.ps_axis_2.getValue() - tower_margin) and desired_tower_height <= (self.ps_axis_2.getValue() + tower_margin)):
                                        self.tower_pos_reached = True
                                        print("Tower position reached!")

                                if self.tower_pos_reached and self.snake_pos_reached and self.gripper_pos_reached:
                                    self.packet_placed = True

                                    # if self.
                                    # remote_message[0] = 'detach'
                                    # print("Lowered message", message[11].lower())
                                    # if (message[11].lower() == 'false'):
                                    #     print("It is connected!")
                                    #     self.packet_attached = False

                        if self.packet_placed:
                            # Place the packet.
                            remote_message[0] = 'detach'
                            self.packet_attached = False

                            self.go_init_pos = True
                            self.reset_picking_variables()
                            print("Go to initial position.")
                            # remote_message[0] = 'detach'
                            # self.deliver_on_agv = False

                    elif self.go_init_pos:
                        remote_message[0] = 'idle'

                        if not self.tower_pos_reached and not self.snake_pos_reached:
                            desired_tower_height = snaketip_coordinates[2] + 0.2
                            # desired_tower_height = packet_coordinates[1] + 0.2798 + 0.2 # - 0.83 + 0.2
                            self.move_tower_to(desired_tower_height)
                            if (desired_tower_height >= (
                                    self.ps_axis_2.getValue() - tower_margin) and desired_tower_height <= (
                                    self.ps_axis_2.getValue() + tower_margin)):
                                self.tower_pos_reached = True
                                print("Tower position reached!")

                        if not self.gripper_pos_reached and self.tower_pos_reached:
                            desired_gripper_angle = 0
                            print("Gripper angle: ", desired_gripper_angle)

                            self.rotate_snaketip_to(angle=desired_gripper_angle, speed=2)

                            if (desired_gripper_angle >= (
                                    math.degrees(self.ps_axis_4.getValue()) - gripper_margin)) and \
                                    (desired_gripper_angle <= (
                                            math.degrees(self.ps_axis_4.getValue()) + gripper_margin)):
                                self.gripper_pos_reached = True

                        if not self.snake_pos_reached and self.gripper_pos_reached:
                            self.move_snake_to(0)

                        if not self.snakebox_pos_reached and self.snake_pos_reached:
                            desired_snakebox_angle = 0
                            self.rotate_snakebox_to(desired_snakebox_angle)

                            if (desired_snakebox_angle >= (
                                    math.degrees(self.ps_axis_1.getValue()) - snakebox_margin)) and \
                                    (desired_snakebox_angle <= (
                                            math.degrees(self.ps_axis_1.getValue()) + snakebox_margin)):
                                self.snakebox_pos_reached = True

                            if self.snakebox_pos_reached:
                                self.tower_pos_reached = False

                        if not self.tower_pos_reached and self.snake_pos_reached and self.gripper_pos_reached and self.snakebox_pos_reached:
                            desired_tower_height = 1
                            self.move_tower_to(desired_tower_height)

                            if (desired_tower_height >= (
                                    self.ps_axis_2.getValue() - tower_margin) and desired_tower_height <= (
                                    self.ps_axis_2.getValue() + tower_margin)):
                                self.tower_pos_reached = True
                                print("Tower position reached!")

                        if self.tower_pos_reached and self.snakebox_pos_reached and self.gripper_pos_reached and self.snakebox_pos_reached:
                            print("Robot is now in idle position.")
                            self.robot_ready = False

                    # if not self.packet_attached and not remote_message[0] == 'detach':
                    #     remote_message[0] = 'idle'
                    #     remote_message[1] = 0


                else:
                    remote_message[0] = 'idle'
                    print("Robot is not ready for new packet.")

                    if message[0] == 'idle' and self.packet_picking_started:
                        print("AGV going to idle position.")
                        self.speed_agv = 0.5 * self.max_velocity_agv
                        self.move_agv_to(0, 0, 8)
                        self.go_idle = True

                    elif self.restart_picking_process() and not message[0] == 'idle':
                        print("Robot has been reset and is ready.")
                        remote_message[2] = 'True'
                        self.robot_ready = True

                    elif self.go_idle:
                        print("Robot is idle.")
                        # self.reset_extending_variables()
                        # self.snake_retracting = True
                        pass

                    else:
                        print("I'm idle.")








                # elif not self.packet_attached and self.packet_picking_started:
                #     pass




                    # if not self.snake_pos_reached and self.tower_pos_reached:
                    #



                # if (self.PACKET_ATTACH in keystrokes):
                #     remote_message[0] = 'attach'
                # elif (self.PACKET_DETACH in keystrokes):
                #     remote_message[0] = 'detach'

                self.emitted_message = self.list_to_string(remote_message)


            elif mode == "Automatic mode":
                pos = 2
                self.motor_rightFWD_AGV.setVelocity(5)
                self.motor_rightAFT_AGV.setVelocity(5)
                self.motor_leftFWD_AGV.setVelocity(5)
                self.motor_leftAFT_AGV.setVelocity(5)
                self.motor_rightFWD_AGV.setPosition(pos)
                self.motor_rightAFT_AGV.setPosition(pos)
                self.motor_leftFWD_AGV.setPosition(pos)
                self.motor_leftAFT_AGV.setPosition(pos)
            else:
                # Print error
                sys.stderr.write("No or incorrect mode selected.\n")

                # The program will exit
                sys.exit("The program will now be terminated.")
                pass

            # Send the message at the end of the iteration.
            self.send_message(self.emitted_message)

            # Reset emitted message.
            # self.emitted_message = ""
    populate_snake(num_snake_joints)
    
    
    # Communication with TwinCAT/PLC over EtherCAT
    #communication.set_ip("254.254.254.253")
    #print("IP is: ", str((communication.get_ip())))

    # Robot mode selection
    mode_selection = 1
    mode = robot_mode(mode_selection)

    # Populate with packets
    packet1 = Packet(300, 200, 180, "small")
    packet2 = Packet(300, 200, 180, "large")

    pallet1 = Pallet(1200, 800, 145)
    pallet1.add_packet(packet1)
    pallet1.add_packet(packet2)

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

        # print("==========               ============")
        # for packet in range(len(pallet1.get_packets())):
        #     packet_current = pallet1.get_packets()[packet]
        #     print("This should be 300: ", packet_current.width)
        # print("======================")









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