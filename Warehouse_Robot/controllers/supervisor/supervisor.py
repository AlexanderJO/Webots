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

    def error_testing_thingy(self, num):
        assert isinstance(num, int), INVALID_NUM_MSG

    def instantiate_packets(self):
        try:
            pass
        except Exception as e:
            e = traceback.format_exc()
            print("Error: ", e)

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

    def populate_packets(self, no_packets):
        for i in no_packets:
            pass

    def create_pallets(self, num_pallets):
        # Create pallet list.
        self.pallet_list = [""] * num_pallets

        for i in range(num_pallets):
            pallet_name = "PALLET_" + str(i+1)
            pallet_obj = self.getFromDef(pallet_name)
            pallet = Pallet(pallet=pallet_obj, length=1.200, width=0.800, height=0.155)
            self.pallet_list[i] = pallet
            print("Pallet position ", i+1, ": ", self.pallet_list[i].get_pallet_position())

    def place_pallets(self):
        #pallets = self.create_pallets(num_pallets=12)
        #self.pallet_list
        pass

    def get_pallet_pos(self, pallet_num):
        pass

    def get_packet_level(self, packet_num, packet_size, pallet_num):
        max_packets_level = 9

    def set_small_dummy_pallets(self, pallet_num, num_packets, packet_size):
        # Set number of dummy packets.
        length = 0
        width = 0
        height = 0

        if packet_size == "small":
            length = 0.300
            width = 0.200
            height = 0.180
        elif packet_size == "large":
            length = 0.600
            width = 0.400
            height = 0.369
        else:
            packet_size = "small"
            length = 0.300
            width = 0.200
            height = 0.180

        for i in range(num_packets):
            packet_name = "PACKET_" + str(i+1)
            packet = Packet(length, width, height, packet_size, packet_name)
            self.pallet_list[pallet_num-1].add_packet(packet=packet)


        # Populate with packets
        # packet1 = Packet(0.300, 0.200, 0.180, "small")
        # packet2 = Packet(0.300, 0.200, 0.180, "small")
        # packet3 = Packet(0.300, 0.200, 0.180, "small")
        # packet4 = Packet(0.300, 0.200, 0.180, "small")
        # packet5 = Packet(0.300, 0.200, 0.180, "small")
        # packet6 = Packet(0.300, 0.200, 0.180, "small")
        # packet7 = Packet(0.300, 0.200, 0.180, "small")
        # packet8 = Packet(0.300, 0.200, 0.180, "small")
        # packet9 = Packet(0.300, 0.200, 0.180, "small")
        # packet10 = Packet(0.300, 0.200, 0.180, "small")
        # packet11 = Packet(0.300, 0.200, 0.180, "small")
        # packet12 = Packet(0.300, 0.200, 0.180, "small")
        # packet13 = Packet(0.300, 0.200, 0.180, "small")
        # packet14 = Packet(0.300, 0.200, 0.180, "small")
        # packet15 = Packet(0.300, 0.200, 0.180, "small")
        # packet16 = Packet(0.300, 0.200, 0.180, "small")
        # packet17 = Packet(0.300, 0.200, 0.180, "small")
        # packet18 = Packet(0.300, 0.200, 0.180, "small")

        # self.pallet_list[0].add_packet(packet=packet1)
        # self.pallet_list[0].add_packet(packet=packet2)
        # self.pallet_list[0].add_packet(packet=packet3)
        # self.pallet_list[0].add_packet(packet=packet4)
        # self.pallet_list[0].add_packet(packet=packet5)
        # self.pallet_list[0].add_packet(packet=packet6)
        # self.pallet_list[0].add_packet(packet=packet7)
        # self.pallet_list[0].add_packet(packet=packet8)
        # self.pallet_list[0].add_packet(packet=packet9)
        # self.pallet_list[0].add_packet(packet=packet10)
        # self.pallet_list[0].add_packet(packet=packet11)
        # self.pallet_list[0].add_packet(packet=packet12)
        # self.pallet_list[0].add_packet(packet=packet13)
        # self.pallet_list[0].add_packet(packet=packet14)
        # self.pallet_list[0].add_packet(packet=packet15)
        # self.pallet_list[0].add_packet(packet=packet16)
        # self.pallet_list[0].add_packet(packet=packet17)
        # self.pallet_list[0].add_packet(packet=packet18)

    # def check_dist

    def manual_control(self, keystrokes):

        # ------ MOVE AGV -----
        # Increment AGV speed
        # self.increment_speed_agv(keystrokes=keystrokes)

        # Move AGV
        # self.emitter.send()
        # self.AGV_ROBOT_1.move_agv()
        # self.move_agv(keystrokes=keystrokes)

        # ------ HASSELHOFF HUG / SNAKE BOX - AXIS 1 -----
        # Increment Axis 1 speed
        # self.increment_speed_snakebox(keystrokes=keystrokes)

        # Rotate snake box - Axis 1
        # self.rotate_snakebox(keystrokes=keystrokes)

        # ------ TOWER - AXIS 2 -----
        # Increment Tower speed
        # self.increment_speed_tower(keystrokes=keystrokes)

        # Move tower height
        # self.change_tower_height(keystrokes=keystrokes)

        # ------ SNAKE - AXIS 3 -----
        # Increment Snake speed
        # self.increment_speed_snake_manual(keystrokes=keystrokes)

        # Move snake part 1
        # self.move_snake_manual(keystrokes=keystrokes)
        pass

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
        if message != '':# and message != self.previous_message:
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

    def update_picked_packet_positions(self):
        updated = False

        if len(self.picked_packets) == 0:
            updated = False
        else:
            for i in range(len(self.picked_packets)):
                updated_pos = [0, 0, 0]

                # Get the packet and current position.
                packet_num = self.picked_packets[i]
                # print("Packet number: ", packet_num)
                packet_name = "PACKET_" + str(packet_num)
                packet = self.getFromDef(packet_name)
                current_pos = packet.getField('translation').getSFVec3f()

                # Get difference in current and previous gps position.
                gps_diff_x = self.current_gps_pos[0] - self.previous_gps_pos[0]
                gps_diff_y = self.current_gps_pos[1] - self.previous_gps_pos[1]
                gps_diff_z = self.current_gps_pos[2] - self.previous_gps_pos[2]
                gps_pos = [gps_diff_x, gps_diff_y, gps_diff_z]

                # Get updated packet position.
                for i in range(3):
                    updated_pos[i] = current_pos[i] + gps_pos[i]
                # updated_pos_x = current_pos[0] + gps_pos[0]
                # updated_pos_y = current_pos[1] + gps_pos[1]
                # updated_pos_z = current_pos[2] + gps_pos[2]

                # Set updated packet position.
                packet.getField('translation').setSFVec3f(updated_pos)

            updated = True

            print("Updated position for: ", self.picked_packets)

        return updated

    # Convert list to string. List shall contain
    def list_to_string(self, list_elem):
        text_string = ' '.join([str(elem) for elem in list_elem])

        return text_string


    def run(self):
        # Set initial conditions.
        self.pallet_num = 0
        self.packet_num = 30
        self.agv_packet_num = 0
        # self.picking_started = True

        # Populate pallets with packets.
        self.set_small_dummy_pallets(pallet_num=(self.pallet_num + 1), num_packets=self.packet_num, packet_size="small") # Create a dummy pallet.

        #pallet1 = Pallet(pallet=self.pallet_list[0],length=1200, width=800, height=145)
        #pallet1.add_packet(packet1)
        #pallet1.add_packet(packet2)

        # print("Number of packets: ", self.pallet_list[0].get_num_packets())
        # print("Packet num 1: ", self.pallet_list[0].get_packet(1))
        # print("Packets: ", self.pallet_list[self.pallet_num].get_packets())
        # print("Num packets: ", sum(1 for _ in filter(None.__ne__, self.pallet_list[0].get_packets())))
        # print("Num packets: ", sum(x is not None for x in self.pallet_list[0].get_num_packets()))

        # print("Num packets: ", self.pallet_list[self.AGV_PALLET_NO].get_num_packets())
        # print("Num packets pallet 0: ", self.pallet_list[0].get_num_packets())
        # print("Packets pallet 0: ", self.pallet_list[0].get_packets())
        # print("Num packets pallet 1: ", self.pallet_list[1].get_num_packets())
        # print("Packets pallet 1: ", self.pallet_list[1].get_packets())
        # print("Num packets pallet 2: ", self.pallet_list[2].get_num_packets())
        # print("Packets pallet 2: ", self.pallet_list[2].get_packets())

        # print("Packet deleted: ", self.pallet_list[0].remove_packet(1))
        # print("Packets: ", self.pallet_list[0].get_packets())

        # Position data for connecting packets.
        positioned = [False for i in range(3)]
        error_margin = [0] * 3
        error_margin[0] = 0.1
        error_margin[1] = 0.025
        error_margin[2] = 0.05

        # Robot mode selection
        mode_selection = 2
        mode = self.robot_mode(mode_selection)

        # Initial robot state
        # self.state_machine("")

        # Main loop:
        # while self.step(timestep) != -1:
        while True:
            # Read keyboard values
            keystrokes = self.read_keyboard()

            # Update GPS positions.
            self.current_gps_pos = self.getFromDef('body_agv').getPosition()
            # print("Current gps pos of AGV: ", self.current_gps_pos)

            # print("==================")
            # print("AGV position      2: ", self.get_position(self.AGV_ROBOT_1))
            # print("Suction cup position: ", self.get_position(self.SUCTION_CUP))
            # print("==================")

            dist = self.dist_two_objects(self.AGV_ROBOT_1, self.SUCTION_CUP)
            # print("Difference AGV and Snake tip: ", dist)
            # print(self.get_position(self.SUCTION_CUP))

            # test = self.getFromDef('body_agv')
            # print("Parent node: ", test.getPosition())

            suction_cup_pos = self.get_position(self.SUCTION_CUP)
            # packet_picked_pos_init = self.pallet_list[0].get_packet_position(16)
            # print("Suction cup position: ", suction_cup_pos)
            # print("Packet picked position: ", packet_picked_pos_init)



            # print(positioned)

            #self.switch_case(argument=1)





            # Runs either manual by keyboard input, or in automatic or remote mode.
            if mode == 'Manual mode':
                manual_message = ['None'] * 12 # Clear previous message.

                message = self.receive_message()
                # print("Message received supervisor:", message)

                # AGV keyboard input.
                if (self.FORWARD in keystrokes):  # Drive forwards
                    if (self.TURN_LEFT in keystrokes):  # Turn left
                        agv_state = 'forward_left'
                    elif (self.TURN_RIGHT in keystrokes):  # Turn right
                        agv_state = 'forward_right'
                    else:
                        agv_state = 'forward'
                elif (self.BACKWARD in keystrokes):  # Drive backwards
                    if (self.TURN_LEFT in keystrokes):  # Turn left
                        agv_state = 'backward_left'
                    elif (self.TURN_RIGHT in keystrokes):  # Turn right
                        agv_state = 'backward_right'
                    else:
                        agv_state = 'backward'
                elif (self.TURN_LEFT in keystrokes):  # Turn left
                    agv_state = 'left'
                elif (self.TURN_RIGHT in keystrokes):  # Turn right
                    agv_state = 'right'
                else:
                    agv_state = 'idle'
                manual_message[0] = agv_state

                # Tower keyboard input.
                if (self.UP_AXIS_2 in keystrokes):
                    tower_state = 'up'
                elif (self.DOWN_AXIS_2 in keystrokes):
                    tower_state = 'down'
                else:
                    tower_state = 'idle'
                manual_message[5] = tower_state

                # Snakebox keyboard input.
                if (self.LEFT_AXIS_1 in keystrokes):
                    snakebox_state = 'left'
                elif (self.RIGHT_AXIS_1 in keystrokes):
                    snakebox_state = 'right'
                else:
                    snakebox_state = 'idle'
                manual_message[4] = snakebox_state

                # Snake keyboard input.
                if (self.EXTEND_AXIS_3 in keystrokes):
                    snake_state = 'extend'
                elif (self.RETRACT_AXIS_3 in keystrokes):
                    snake_state = 'retract'
                else:
                    snake_state = 'idle'
                manual_message[6] = snake_state

                # Snaketip keyboard input.
                if (self.LEFT_AXIS_4 in keystrokes):
                    snaketip_state = 'left'
                elif (self.RIGHT_AXIS_4 in keystrokes):
                    snaketip_state = 'right'
                else:
                    snaketip_state = 'idle'
                manual_message[7] = snaketip_state



                # if (self.axis_2_pos > self.motor_axis_2.getMaxPosition()):
                #     self.axis_2_pos = round(self.motor_axis_2.getMaxPosition(), 2)
                #     # print("Desired pos: ", self.axis_2_pos)
                #     # print("Maximum position: ", motor_axis_2.getMaxPosition())
                #     sys.stderr.write("Axis 2 has reached maximum height.\n")
                # elif (self.axis_2_pos < self.motor_axis_2.getMinPosition()):
                #     self.axis_2_pos = round(self.motor_axis_2.getMinPosition(), 2)
                #     # print("Min. pos: ", motor_axis_2.getMaxPosition())
                #     sys.stderr.write("Axis 2 has reached minimum height.\n")
                # else:
                #     self.motor_axis_2.setPosition(self.axis_2_pos)

                self.emitted_message = self.list_to_string(manual_message)

            elif mode == "Remote mode":
                remote_message = ['None'] * 20 # Clear previous message.

                message = self.receive_message()
                # print("Message received supervisor:", message)

                # print("=========================================")
                # print("OTHER pallet packets: ", self.pallet_list[self.pallet_num].get_packets())
                # print("             --------------                 ")
                # print("AGV pallet packets: ", self.pallet_list[self.AGV_PALLET_NO].get_packets())
                # print("=========================================")

                # robot_status = 'true'  # message[2].lower()

                try:
                    self.robot_status = message[2].lower()
                    self.picking_started = True
                except:
                    pass

                print("Currently packed: ", self.picked_packets)

                if self.packet_num <= 0:
                    self.picking_finished = True
                else:
                    # Picked packets.
                    packet_picked_pos_init = self.pallet_list[self.pallet_num].get_packet_position(self.packet_num)
                    remote_message[8] = packet_picked_pos_init[0]
                    remote_message[9] = packet_picked_pos_init[1]
                    remote_message[10] = packet_picked_pos_init[2]

                # pallet_num = 0
                #
                # packet_num = 16

                # print("AGV Pallet: ", "Num. packets: ", self.pallet_list[self.AGV_PALLET_NO].get_num_packets(),
                #       " and stored: ", self.pallet_list[self.AGV_PALLET_NO].get_packets())
                # print("Main Pallet: ", "Num. packets: ", self.pallet_list[self.pallet_num].get_num_packets(),
                #       " and stored: ", self.pallet_list[self.pallet_num].get_packets())
                if self.picking_started and not self.picking_finished:
                    if (self.robot_status == 'true'):
                        # print("This is remaining packets: ", self.pallet_list[self.pallet_num].get_packets())

                        if all(positioned):
                            if (message[0] == 'attach'):
                                # print("We're about to connect!")
                                packet_name = self.pallet_list[self.pallet_num].get_packet(self.packet_num).get_name()
                                packet = self.getFromDef(packet_name)
                                x = round(suction_cup_pos[0], 7)
                                y = round(suction_cup_pos[1], 7) - self.pallet_list[self.pallet_num].get_packet(self.packet_num).get_height() / 2
                                z = round(suction_cup_pos[2], 7)
                                pos = [x, y, z]
                                packet.getField('translation').setSFVec3f(pos)
                                # print("New packet position: ", self.get_position(packet))
                                # print("Used position: ", pos)

                                # packet_picked_pos
                                # setSFVec3f(suction_cup_pos)
                                remote_message[11] = 'True'
                                print("Picked packet: ", packet_name)
                                print("Picked packet number: ", self.packet_num)
                                # print("We're connected!")
                                # print("=============")

                                # Rotate the package in accordance with the snake angle.
                                gripper_angle = math.radians(float(message[1]))
                                gripper_angle_list = [0, 1, 0, gripper_angle]
                                # gripper = self.getFromDef('motor_axis_4')
                                # gripper_angle_1 = gripper.getField('rotation')
                                # print("Gripper angle first: ", gripper_angle_1)
                                # gripper_angle_2 = gripper.getField('rotation').getSFRotation()
                                # print("Gripper angle first: ", gripper_angle_2)
                                # gripper_angle = gripper.getSFRotation()
                                # print("Gripper angle: ", gripper_angle)
                                packet.getField('rotation').setSFRotation(gripper_angle_list)

                                # Sets status that packet has been picked.
                                remote_message[11] = "True"

                                if not self.packet_added:
                                    # Packet to be moved.
                                    packet = self.pallet_list[self.pallet_num].get_packet(packet_num=self.packet_num)
                                    self.picked_packets.append(self.packet_num)
                                    print("PACKET ADDED: ", packet)

                                    # Find position on new pallet and place packet.
                                    # self.pallet_list[self.AGV_PALLET_NO].add_packet(packet=packet)
                                    self.agv_packet_num += 1
                                    # self.agv_packet_num = self.pallet_list[self.AGV_PALLET_NO].get_num_packets()

                                    # Packet has been added.
                                    self.packet_added = True

                                if self.packet_added:
                                    # print("AGV pallet packets: ", self.pallet_list[self.AGV_PALLET_NO].get_packets())
                                    # print("Other pallet packets: ", self.pallet_list[self.pallet_num].get_packets())

                                    # Get the next position for the packet - Only preliminary position. AGV to decide final position.
                                    # packet_picked_pos_final = self.pallet_list[self.AGV_PALLET_NO].get_packet_position(self.agv_packet_num)
                                    # remote_message[12] = packet_picked_pos_final[0]
                                    # remote_message[13] = packet_picked_pos_final[1]
                                    # remote_message[14] = packet_picked_pos_final[2]

                                    remote_message[15] = self.agv_packet_num
                                    packet_dimensions = self.pallet_list[self.pallet_num].get_packet_dimensions(self.packet_num)
                                    # packet_dimensions = self.pallet_list[self.AGV_PALLET_NO].get_packet_dimensions(self.agv_packet_num)
                                    remote_message[16] = packet_dimensions[0] # Width
                                    remote_message[17] = packet_dimensions[1] # Length
                                    remote_message[18] = packet_dimensions[2] # Height

                                    # print("Packet dimensions: ", "Width = ", remote_message[16], "Length = ", remote_message[17], "Height = ", remote_message[18])



                                # Ready to do a count when detaching packet.
                                self.count_now = True


                            elif (message[0] == 'detach'):
                                if self.count_now:
                                    if not self.packet_removed:
                                        # Remove packet from initial pallet.
                                        # self.pallet_list[self.pallet_num].remove_packet(packet_num=self.packet_num)
                                        self.packet_removed = True

                                    self.packet_num -= 1
                                    print("I counted now up to: ", self.packet_num)
                                    self.count_now = False
                                    self.packet_added = False
                                remote_message[11] = "False"

                            elif (message[0] == 'idle'):
                                remote_message[11] = "False"
                                print("Currently I have: ", self.packet_num, " packets.")
                        else:
                            for i in range(3):
                                # Check x, y and z position
                                if suction_cup_pos[i] >= (packet_picked_pos_init[i] - error_margin[i]) and suction_cup_pos[i] <= \
                                        (packet_picked_pos_init[i] + error_margin[i]):
                                    positioned[i] = True
                                else:
                                    positioned[i] = False

                    else:
                        print("AGV not ready.")

                elif self.picking_finished:
                    print("Picking order has finished.")
                    remote_message[0] = 'idle'

                else:
                    print("AGV robot is not ready.")
                    self.picking_started = False

                # Updating packet position on AGV.
                if self.update_picked_packet_positions():
                    print("Packet positions on AGV updated.")

                # print("Packet position: ", positioned)

                self.emitted_message = self.list_to_string(remote_message)


            elif mode == "Automatic mode":
                pass

            else:
                # Print error
                sys.stderr.write("No or incorrect mode selected.\n")

                # The program will exit
                sys.exit("The program will now be terminated.")
                pass

            # time1 = time.time()
            # count = 0
            # for i in range(100):
            #     count += 1
            #     s = self.emitted_message+str(count)
            #     self.send_message(s)
            # time2 = time.time()
            # print("Time 1: ", time1)
            # print("Time 2: ", time2)
            # print("========")
            # print("Keys: ", keystrokes)
            # print("Type: ", type(keystrokes[0]))
            # print(*keystrokes[0])
            # bufferTest = self.send_message_struct(keystrokes)






            # Send the message at the end of the iteration.
            self.send_message(self.emitted_message)

            # Update previous GPS position.
            self.previous_gps_pos = self.current_gps_pos

            try:
                pass
                # print(self.pallet_list[0].get_pallet_position())
                # print(self.pallet_list[0].get_packet_position(16))
            except AttributeError:
                print("Pallet number is incorrect.")

            # Reset emitted message.
            self.emitted_message = ""


            # Breaks the simulation.
            while self.step(self.TIME_STEP) == -1:
                print("============== I'm breaking up! ==============")
                break

    def switch_case_robot(self, argument):
        # Creating a dictionary of the states.
        switcher = {
            1: self.robot_idle_sc,
            2: self.move_agv_sc,
            3: self.move_tower_sc,
            4: self.rotate_snakebox_sc,
            5: self.extend_snake_sc,
            6: self.attach_packet_sc,
            7: self.detach_packet_sc,
            8: self.retract_snake_sc
        }

        # Get the function from switcher dictionary
        func = switcher.get(argument, lambda: "Invalid robot state")
        # Execute the function
        state = func()
        return state

    def robot_idle_sc(self):
        print("Robot is in idle mode!")
        pass

    def move_agv_sc(self):
        pass

    def move_tower_sc(self):
        pass

    def rotate_snakebox_sc(self):
        pass

    def extend_snake_sc(self):
        pass

    def attach_packet_sc(self):

        pass

    def detach_packet_sc(self):
        pass

    def retract_snake_sc(self):
        pass

    # ------ Switch case -----
    def manual_mode_sc(self):
        return "Manual mode"

    def remote_mode_sc(self):
        return "Remote mode"

    def automatic_mode_sc(self):
        return "Automatic mode"

    # Set the type of robot mode.
    def robot_mode(self, argument):
        switcher = {
            1: self.manual_mode_sc,
            2: self.remote_mode_sc,
            3: self.automatic_mode_sc
        }

        # Get the function from switcher dictionary
        func = switcher.get(argument, lambda: "Invalid robot mode")
        # Execute the function
        mode = func()
        return mode


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






if __name__ == "__main__":
    Driver()
# controller = Driver()
# controller.run()