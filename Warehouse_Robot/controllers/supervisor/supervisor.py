"""supervisor controller."""
from controller import Supervisor, Keyboard
import traceback
import sys, math, struct, time
from dataclasses import dataclass, field

# Error codes
INVALID_NUM_MSG = "The parameter must be integer!"

# Switch case variables
ROBOT_IDLE = '1'
MOVE_AGV = '2'
MOVE_TOWER = '3'
ROTATE_SNAKE = '4'
EXTEND_SNAKE = '5'
ATTACH_PACKET = '6'
DETACH_PACKET = '7'
RETRACT_SNAKE = '8'

class Driver(Supervisor):
    TIME_STEP = 32
    x = int(0.0)
    y = int(0.0)
    z = int(0.0)
    translation = [x, y, z]

    # Instantiated objects
    AGV_ROBOT_1 = None
    SUCTION_CUP = None
    #agv_robot_1_translation_field = agv_robot_1.getField('translation')
    #suction_cup = self.getFromDef('suction_cup')
    #box_1 =
class Driver(Supervisor):
    TIME_STEP = 20
    x = int(0.0)
    y = int(0.0)
    z = int(0.0)
    translation = [x, y, z]


    # Variables for AGV, pallets and packets
    picking_started = False
    picking_finished = False
    robot_status = False
    MAX_NUM_PALLETS = None
    AGV_PALLET_NO = None
    count_now = False
    packet_removed = False
    packet_added = False

    # Pallets
    pallet_list = None
    pallet_num = None

    # Packets
    picked_packets = None
    packet_list = None
    packet_num = None
    agv_packet_num = None

    # Variables to hold and update the location of packets.
    current_gps_pos = None
    previous_gps_pos = None

    # ----------------- COMMUNICATION ---------------
    # Instantiate receiver node to obtain data from emitters.
    emitter = None
    receiver = None

    # Message sent by emitter.
    emitted_message = None
    previous_message = None

    # ------------------- KEYBOARD --------------
    # # Enable the keyboard.
    # kb = Keyboard()
    # kb.enable(TIME_STEP)

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

    def __init__(self):
        # Initialize the driver.
        super(Driver, self).__init__()

        # Instantiate emitter node to send data to receivers.
        self.emitter = self.getDevice('emitter_supervisor')

        # Instantiate receiver node to obtain data from emitters.
        self.receiver = self.getDevice('receiver_supervisor')
        self.receiver.enable(self.TIME_STEP)

        # Create list for emitted message.

        # ========== AGV ROBOTS ==========
        # Instantiate the AGV robots.
        self.AGV_ROBOT_1 = self.getFromDef('AGV_ROBOT1')
        self.robot_status = False

        # Instantiate suction gripper
        self.SUCTION_CUP = self.getFromDef('SUCTION_CUP')

        self.MAX_NUM_PALLETS = 13
        self.AGV_PALLET_NO = (self.MAX_NUM_PALLETS - 1)
        self.create_pallets(num_pallets=self.MAX_NUM_PALLETS)
        self.picked_packets = []

        # Set initial GPS values for storing of updated packet positions.
        self.current_gps_pos = [0, 0, 0]
        self.previous_gps_pos = [0, 0, 0]

        # Enable the keyboard.
        self.kb = Keyboard()
        self.kb.enable(self.TIME_STEP)

        self.run()

        # ========== PACKET ROBOTS ==========
        # Instantiate the active packets/boxes.

        
    def run(self):
    
        # Main loop:
        # while self.step(timestep) != -1:
        while True:
            print("Translation field: ", self.agv_robot_1_translation_field.getSFVec3f())
            
            
            #
            while self.step(self.TIME_STEP) == -1:
                print("============== I'm breaking up! ==============")
                break

    def error_testing_thingy(self, num):
        assert isinstance(num, int), INVALID_NUM_MSG

    def instantiate_packets(self):
        try:
            pass
        except Exception as e:
            e = traceback.format_exc()
            print("Error: ", e)


controller = Driver()
controller.run()