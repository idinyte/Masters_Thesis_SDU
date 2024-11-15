import atexit
from dynamixel_sdk import *  # Uses Dynamixel SDK library

DEVICENAME = 'COM5'
BAUDRATE = 57600
PROTOCOL_VERSION = 2.0

ADDR_PRESENT_POSITION = 132

class GripperMotors():
    def __init__(self):
        self.portHandler = PortHandler(DEVICENAME)
        self.packetHandler = PacketHandler(PROTOCOL_VERSION)

        if not self.portHandler.openPort():
            print("Failed to open the port")
            quit()

        if not self.portHandler.setBaudRate(BAUDRATE):
            print("Failed to change the baudrate")
            quit()

        self.motor_ids = [1, 2] # 1 right, 2 - left
        self.positions = [2000, 2000]

        atexit.register(self.cleanup_on_exit)
        
    def positions_healthy(self):
        for position in self.positions:
            if not (1650 <= position <= 2250):
                return False

        return True
      
    def normal_forces_to_torques(self, left_motor_force, right_motor_force):
      center_to_pad_distance = 0.07
      left_motor_torque = left_motor_force * center_to_pad_distance
      right_motor_torque = right_motor_force * center_to_pad_distance
      return left_motor_torque, right_motor_torque
    
    def pos_to_deg(self, pos):
      """ position is in thicks from 0 to 4095. 1 deg is 4095 / 360 = 11.375 """
      deg = pos / 11.375
      return deg

    def update_positions(self, verbose = False):
        for index, motor_id in enumerate(self.motor_ids):
            position, dxl_comm_result, dxl_error = self.packetHandler.read2ByteTxRx(self.portHandler, motor_id, ADDR_PRESENT_POSITION)

            if dxl_comm_result != COMM_SUCCESS:
                print(f"Communication error with motor {motor_id}: {self.packetHandler.getTxRxResult(dxl_comm_result)}")
            elif dxl_error != 0:
                print(f"Hardware error with motor {motor_id}: {self.packetHandler.getRxPacketError(dxl_error)}")
            else:
                self.positions[index] = position

        if verbose:
          print(f"Motor ID {self.motor_ids[0]} position: {self.positions[0]} deg: {self.pos_to_deg(self.positions[0])} Motor ID {self.motor_ids[1]} position: {self.positions[1]} deg: {self.pos_to_deg(self.positions[1])}")
        
        if not self.positions_healthy():
          quit()
            
    def close_port(self):
        self.portHandler.closePort()
        print("Motor port closed")

    def cleanup_on_exit(self):
        print("Program is exiting. Cleaning up resources...")
        self.close_port()
