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

    # Continuously reads the keyboard for user inputs.
    # Reads up to 7 simultaneous/combined key presses.
    def read_keyboard(self):
        # ------ KEYBOARD -----
        # https://cyberbotics.com/doc/reference/keyboard
        # Register keystrokes
        keystrokes = [str(-1)] * 7
        for i in range(0, 7):
            keystrokes[i] = str(self.kb.getKey())
        # print(keystrokes)
        return keystrokes
        
    # Returns the distance of two objects as coordinate system.
    # Distance calculated as difference between object 1 and object 2.
    def dist_two_objects(self, obj_1, obj_2):
        obj_1_pos = self.get_position(obj_1)
        obj_2_pos = self.get_position(obj_2)

        # Object distance in millimeters
        dist = [0] * 3
        dist[0] = obj_1_pos[0] - obj_2_pos[0]
        dist[1] = obj_2_pos[1] - obj_1_pos[1]
        dist[2] = obj_1_pos[2] - obj_2_pos[2]

        return dist

    def get_translation(self, object):
        return object.getField('translation').getSFVec3f()

    def get_position(self, object):
        return object.getPosition()

    def get_rotation(self, object):
        return object.getOrientation()
    def create_pallets(self, num_pallets):
        # Create pallet list.
        self.pallet_list = [""] * num_pallets

        for i in range(num_pallets):
            pallet_name = "PALLET_" + str(i+1)
            pallet_obj = self.getFromDef(pallet_name)
            pallet = Pallet(pallet=pallet_obj, length=1.200, width=0.800, height=0.155)
            self.pallet_list[i] = pallet
            print("Pallet position ", i+1, ": ", self.pallet_list[i].get_pallet_position())
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
# Dataclass for Packet with immutable input.
@dataclass(frozen=True, order=True)
class Packet:
    length: float = field()
    width: float = field()
    height: float = field()
    size: str = field(default="NO SIZE SELECTED!")
    name: str = field(default="NO PACKET NAME")

    def get_length(self):
        return self.length

    def get_width(self):
        return self.width

    def get_height(self):
        return self.height

    def get_size(self):
        return self.size

    def get_name(self):
        return self.name

@dataclass
class Pallet:
    max_packets = 64  # Max number of packets considered as 16 packets for each level, up to 4 levels.
    length = float(0)
    width = float(0)
    height = float(0)
    count_packets = 0
    packets = {}

    def __init__(self, pallet, length, height, width):
        self.pallet = pallet
        self.length = length
        self.width = width
        self.height = height

        # Populate the pallet list with None objects.
        self.__create_pallet_list()


    # Private function for setting length. Only used for testing. Not callable from outside class.
    def __set_length(self, length):
        self.length = length

    # Private function for setting width. Only used for testing. Not callable from outside class.
    def __set_width(self, width):
        self.width = width

    # Private function for setting height. Only used for testing. Not callable from outside class.
    def __set_height(self, height):
        self.height = height

    def __create_pallet_list(self):
        for i in range(self.max_packets):
            self.packets[i+1] = None

    def add_packet(self, packet):
        self.count_packets += 1
        self.packets[self.count_packets] = packet

    def remove_packet(self, packet_num):
        self.count_packets -= 1
        self.packets[packet_num] = None

    # Returns a dictionary of packets with Packet and packet number.
    def get_packets(self):
        return self.packets

    def get_packet(self, packet_num):
        return self.packets.get(packet_num)

    def get_packet_dimensions(self, packet_num):
        dimensions = [0, 0, 0]
        packet = self.packets.get(packet_num)
        width = packet.get_width()
        length = packet.get_length()
        height = packet.get_height()

        dimensions = [width, length, height]

        return dimensions

    def get_pallet_position(self):
        # Initial displacement of pallets due to inaccuracy of STL file.
        #disp_x = -0.046
        disp_x = -1.371
        disp_y = 0.0775
        #disp_z = -0.798
        disp_z = 0.8

        # Position without displacement.
        inaccurate_pos = self.get_pallet().getPosition()

        x = inaccurate_pos[0] + disp_x
        y = inaccurate_pos[1] + disp_y
        z = inaccurate_pos[2] + disp_z

        coordinates = [0, 0, 0]
        coordinates[0] = x
        coordinates[1] = y
        coordinates[2] = z

        return coordinates

    def get_num_packets(self):
        # return len(self.packets)
        return self.count_packets

    def get_pallet(self):
        return self.pallet

    def get_packet_position(self, packet_num):
        coordinates = [0, 0, 0]

        packet = self.get_packet(packet_num=packet_num)

        max_width = self.width / packet.get_width()
        max_length = self.length / packet.get_length()
        max_packets_level = max_width * max_length

        # Find the level where the packet is located. Each level has maximum 16 packets.
        level = ((packet_num - 1) - (packet_num - 1) % max_packets_level) / max_packets_level + 1

        # Find the position on the given level where the packet is located.
        # ver_pos = ((packet_num - 1) - (packet_num - 1) % max_width) / max_width + 1
        # print("Vertical pos. 1: ", ver_pos)
        #test = ((packet_num - 1) % max_packets_level - ((5 - 1) % 16) % 4 ) / 4 + 1
        #print("Test: ", test)
        ver_pos = (((packet_num - 1) % max_packets_level) - ((packet_num - 1) % max_packets_level) % max_width) / max_width + 1
        # hor_pos = packet_num - max_width * (ver_pos - 1)
        hor_pos = ((packet_num - 1) % max_packets_level + 1) - max_width * (ver_pos - 1)

        # Coordinates for packet.
        x_init = packet.get_length() / 2 + packet.get_length() * (ver_pos - 1)
        y_init = packet.get_height() / 2 + packet.get_height() * (level - 1)
        z_init = packet.get_width() / 2 + packet.get_width() * (hor_pos - 1)
        # print("X: ", x_init, " Y: ", y_init, " Z: ", z_init)

        # Coordinates for packet with reference to GCS.
        x = x_init + self.get_pallet_position()[0] - self.length / 2
        y = y_init + self.get_pallet_position()[1]*2 + packet.get_height() / 2
        z = z_init + self.get_pallet_position()[2] - self.width / 2

        coordinates[0] = x
        coordinates[1] = y
        coordinates[2] = z

        # if packet.get_size() == "small":
        #     pass
        # elif packet.get_size() == "large":
        #     pass
        # else:
        #     pass

        return coordinates

controller = Driver()
controller.run()
if __name__ == "__main__":
    Driver()
# controller = Driver()
# controller.run()