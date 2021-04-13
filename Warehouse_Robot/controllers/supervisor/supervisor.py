"""supervisor controller."""
from controller import Supervisor, Keyboard
import traceback

# Error codes
INVALID_NUM_MSG = "The parameter must be integer!"

class Driver(Supervisor):
    TIME_STEP = 20
    x = int(0.0)
    y = int(0.0)
    z = int(0.0)
    translation = [x, y, z]


    def __init__(self):
        # Initialize the driver.
        super(Driver, self).__init__()

        # ========== AGV ROBOTS ==========
        # Instantiate the AGV robots.
        agv_robot_1 = self.getFromDef('AGV_ROBOT1')
        self.agv_robot_1_translation_field = agv_robot_1.getField('translation')

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