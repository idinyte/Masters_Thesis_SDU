import atexit
from dynamixel_sdk import *
import time
import matplotlib.pyplot as plt
import math
import numpy as np
import traceback

DEVICENAME = 'COM5'
BAUDRATE = 1000000
PROTOCOL_VERSION = 2.0

# Control table addresses https://emanual.robotis.com/docs/en/dxl/x/xm540-w270/
ADDR_OPERATING_MODE = 11
ADDR_TORQUE_ENABLE = 64
ADDR_GOAL_CURRENT = 102
ADDR_PRESENT_CURRENT = 126
ADDR_PRESENT_POSITION = 132
ADDR_CURRENT_LIMIT = 38
ADDR_PWM_LIMIT = 36

# Constants
TORQUE_ENABLE = 1
TORQUE_DISABLE = 0
OPERATING_MODE_CURRENT_CONTROL = 0
OPERATING_MODE_PWM_CONTROL = 16

MAX_PWM = 885

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
        self.portHandler.setPacketTimeoutMillis(1000)
    
        self.right_motor_id = 1
        self.left_motor_id = 2

        self.motor_ids = [self.left_motor_id, self.right_motor_id] 
        self.positions = [2000, 2000]
        self.currents = [0, 0]

        # Constants for XM540-W270-R
        self.torque_constant = 10.6 / 4.4  # Stall torque divided by stall current
        self.current_unit = 0.00269  # 1 unit = 2.69 mA
        
        self.torque_limit = None # None to disable
        self.PWM_limit = 100 # software controller limit
        self.PWM_safety_limit = 150 # hardware limit, affects current control as well
        self.current_limit = 0.4
        
        self.script_public = True
        
        # pid plot
        self.present_current = []
        self.target_current = []
        self.pwm_values = []
        self.last_pwm = 0
        
        # controller
        self.previous_time = None
        self.integral_error = 0
        
        # physical parameters
        self.distance_between_motors = 73 / 1000
        self.motor_finger_radius = 62 / 1000
        
        # Enable motors and set Control Mode
        #self.reset_operating_mode_current(enable_torque=True)
        self.torque_enable = TORQUE_DISABLE
        self.operating_mode = None
        
        # Exponential smoothing
        self.smoothed_average_torque = 0.0
        self.window_size = 10
        self.alpha = 2 / (self.window_size + 1)
        # plotting
        self.smoothed_curves = {}
        
        self._set_motor_limits()
        
        atexit.register(self.cleanup_on_exit)
        
    def _set_motor_limits(self):
        raw_current_limit = int(self.current_limit / self.current_unit) if self.current_limit is not None else None

        for motor_id in self.motor_ids:
            # Set Current Limit
            if raw_current_limit is not None:
                result, error = self.packetHandler.write2ByteTxRx(self.portHandler, motor_id, ADDR_CURRENT_LIMIT, raw_current_limit)
                if result != COMM_SUCCESS:
                    print(f"Failed to set current limit for motor {motor_id}: {self.packetHandler.getTxRxResult(result)}")
                    quit()
                elif error != 0:
                    print(f"Hardware error for motor {motor_id}: {self.packetHandler.getRxPacketError(error)}")
                    quit()
            
            # Set PWM Limit
            result, error = self.packetHandler.write2ByteTxRx(self.portHandler, motor_id, ADDR_PWM_LIMIT, self.PWM_safety_limit)
            if result != COMM_SUCCESS:
                print(f"Failed to set PWM limit for motor {motor_id}: {self.packetHandler.getTxRxResult(result)}")
                quit()
            elif error != 0:
                print(f"Hardware error for motor {motor_id}: {self.packetHandler.getRxPacketError(error)}")
                quit()

        
    def reset_operating_mode_pwm(self, enable_torque = True):
        self.disable_torques(script_accesed_from_outside=False)
        self.operating_mode = OPERATING_MODE_PWM_CONTROL
        for motor_id in self.motor_ids:
            self.disable_torque(motor_id, script_accesed_from_outside=False)
            self._set_operating_mode(motor_id, OPERATING_MODE_PWM_CONTROL, script_accesed_from_outside=False)
            self.set_pwm(motor_id, 0, script_accesed_from_outside=False)

        if enable_torque:
            self.enable_torques()
        
    def reset_operating_mode_current(self, enable_torque = True):
        self.operating_mode = OPERATING_MODE_CURRENT_CONTROL
        self.disable_torques(script_accesed_from_outside=False)
        for motor_id in self.motor_ids:
            self.disable_torque(motor_id, script_accesed_from_outside=False)
            self._set_operating_mode(motor_id, OPERATING_MODE_CURRENT_CONTROL, script_accesed_from_outside=False)
            self.set_current(motor_id, 0, script_accesed_from_outside=False)

        if enable_torque:
            self.enable_torques()
        

    def _set_operating_mode(self, motor_id, mode, script_accesed_from_outside=True):
        if script_accesed_from_outside and not self.script_public:
            return

        try:
            result, error = self.packetHandler.write1ByteTxRx(self.portHandler, motor_id, ADDR_OPERATING_MODE, mode)
            if result != COMM_SUCCESS:
                print(f"Failed to set operating mode for motor {motor_id}: {self.packetHandler.getTxRxResult(result)}")
            elif error != 0:
                print(f"Hardware error for motor {motor_id}: {self.packetHandler.getRxPacketError(error)}")
        except KeyboardInterrupt:
            quit()
            
    def set_pwm(self, motor_id, pwm_value, script_accesed_from_outside=True):
        if script_accesed_from_outside and not self.script_public:
            return
        
        if self.operating_mode != OPERATING_MODE_PWM_CONTROL:
            self.reset_operating_mode_pwm()

        max_pwm = max(min(MAX_PWM, self.PWM_limit), 0)
        pwm_value = max(min(pwm_value, max_pwm), -max_pwm)

        try:
            result, error = self.packetHandler.write2ByteTxRx(self.portHandler, motor_id, 100, pwm_value)
            if result != COMM_SUCCESS:
                print(f"Failed to set PWM for motor {motor_id}: {self.packetHandler.getTxRxResult(result)}")
            elif error != 0:
                print(f"Hardware error for motor {motor_id}: {self.packetHandler.getRxPacketError(error)}")
        except KeyboardInterrupt:
            quit()    

    def enable_torque(self, motor_id, script_accesed_from_outside = True):
        if script_accesed_from_outside and not self.script_public:
            return
        print(f"enable_torque {motor_id}")
        try:
            result, error = self.packetHandler.write1ByteTxRx(self.portHandler, motor_id, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
            if result != COMM_SUCCESS:
                print(f"Failed to enable torque for motor {motor_id}: {self.packetHandler.getTxRxResult(result)}")
            elif error != 0:
                print(f"Hardware error for motor {motor_id}: {self.packetHandler.getRxPacketError(error)}")
        except KeyboardInterrupt:
            quit()
    
    def set_current(self, motor_id, current, script_accesed_from_outside = True):
        if script_accesed_from_outside and not self.script_public:
            return
        
        if self.operating_mode != OPERATING_MODE_CURRENT_CONTROL:
            self.reset_operating_mode_current()

        current = max(min(current, self.current_limit), -self.current_limit)
        current_quantized = int(current / self.current_unit)
        try:
            result, error = self.packetHandler.write2ByteTxRx(self.portHandler, motor_id, ADDR_GOAL_CURRENT, current_quantized)
            if result != COMM_SUCCESS:
                print(f"Failed to set goal torque for motor {motor_id}: {self.packetHandler.getTxRxResult(result)}")
            elif error != 0:
                print(f"Hardware error for motor {motor_id}: {self.packetHandler.getRxPacketError(error)}")
        except KeyboardInterrupt:
            quit()

    def set_goal_torque(self, motor_id, torque_nm, script_accesed_from_outside = True):
        if self.torque_limit != None and self.torque_limit > 0:
            torque_nm = max(min(torque_nm, self.torque_limit), - self.torque_limit)

        # Convert torque to current
        current = torque_nm / self.torque_constant
        self.set_current(motor_id, current, script_accesed_from_outside)
        
    def set_goal_torques(self, torque_left_nm, torque_right_nm, script_accesed_from_outside = True):
        if self.torque_limit != None and self.torque_limit > 0:
            torque_left_nm = max(min(torque_left_nm, self.torque_limit), -self.torque_limit)
            torque_right_nm = max(min(torque_right_nm, self.torque_limit), -self.torque_limit)

        # Convert torque to current
        current_left = torque_left_nm / self.torque_constant
        self.set_current(self.left_motor_id, current_left, script_accesed_from_outside)
        
        current_right = torque_right_nm / self.torque_constant
        # print(f"L {current_left} R {current_right}")
        self.set_current(self.right_motor_id, current_right, script_accesed_from_outside)
        
            
    def get_present_current(self, motor_id, script_accesed_from_outside = True):
        if script_accesed_from_outside and not self.script_public:
            return

        return self.currents[self.motor_ids.index(motor_id)] * self.current_unit
    
    def get_present_position(self, motor_id):
        return self.positions[self.motor_ids.index(motor_id)]

    def get_present_position_deg(self, motor_id):
        return self.pos_to_deg(self.get_present_position(motor_id))
    
    def apply_current_compensation(self, motor_id, target_current):
        if self.previous_time == None:
            self.previous_time = time.time()
            self.prev_error = target_current - self.get_present_current(motor_id)
            return

        current_time = time.time()
        dt = current_time - self.previous_time

        present_current = self.get_present_current(motor_id)
        current_error = target_current - present_current
        
        # Try to tune with EA later

        kp = 150 # 150
        ki = 110 # 110
        kd = 1 # 1
        
        positional = current_error
        positional *= kp
        self.integral_error += current_error * dt
        integral = ki * self.integral_error
        derivative = (current_error - self.prev_error) / dt
        derivative *= kd
        
        print(f"p {positional} i {integral} d {derivative}")
        pwm_value = int(positional + integral + derivative)
        self.set_pwm(motor_id, pwm_value)
        
        #for debugging
        self.present_current.append(present_current)
        self.target_current.append(target_current)
        self.pwm_values.append(pwm_value)
        
        self.previous_time = current_time
        self.prev_error = current_error
        
    def get_delay_between_present_current_and_target(self):
        present_current = np.array(self.present_current)
        target_current = np.array(self.target_current)

        min_error = float('inf')
        best_shift = 0
        shifted_array = present_current

        for shift in range(len(target_current)):
            shifted = np.roll(present_current, -shift)
            error = np.sum(np.abs(shifted - target_current))
            
            if error < min_error:
                min_error = error
                best_shift = -shift
                shifted_array = shifted.copy()

        return best_shift, shifted_array
        
    def train_reset(self):
        self.present_current = []
        self.target_current = []
        self.pwm_values = []
        self.previous_time = None
        self.integral_error = 0

    def get_present_torque(self, motor_id, script_accesed_from_outside = True):
        if script_accesed_from_outside and not self.script_public:
            return

        present_current, result, error = self.get_present_current(motor_id, script_accesed_from_outside=False)
        torque_nm = present_current * self.torque_constant
        return torque_nm

    def update_state(self, verbose=False, script_accesed_from_outside = True):
        if script_accesed_from_outside and not self.script_public:
            return
        
        try:
            groupBulkReadPos = GroupBulkRead(self.portHandler, self.packetHandler)
            # Retrieve position data in bulk
            for motor_id in self.motor_ids:
                groupBulkReadPos.addParam(motor_id, ADDR_PRESENT_POSITION, 2)

            dxl_comm_result_pos = groupBulkReadPos.txRxPacket()
            if dxl_comm_result_pos != COMM_SUCCESS:
                print(f"Position bulk read communication error: {self.packetHandler.getTxRxResult(dxl_comm_result_pos)}")
                return

        
            for index, motor_id in enumerate(self.motor_ids):
                position = groupBulkReadPos.getData(motor_id, ADDR_PRESENT_POSITION, 2)
                if position is None:
                    print(f"Failed to get position for motor {motor_id}")
                else:
                    self.positions[index] = position
        except KeyboardInterrupt:
            quit()

        try:
            # Retrieve current data in bulk
            groupBulkReadCurr = GroupBulkRead(self.portHandler, self.packetHandler)
            
            for motor_id in self.motor_ids:
                groupBulkReadCurr.addParam(motor_id, ADDR_PRESENT_CURRENT, 2)

            dxl_comm_result_curr = groupBulkReadCurr.txRxPacket()
            if dxl_comm_result_curr != COMM_SUCCESS:
                print(f"Current bulk read communication error: {self.packetHandler.getTxRxResult(dxl_comm_result_curr)}")
                return

            for index, motor_id in enumerate(self.motor_ids):
                current = groupBulkReadCurr.getData(motor_id, ADDR_PRESENT_CURRENT, 2)
                if current is None:
                    print(f"Failed to get current for motor {motor_id}")
                else:
                    if current > 32767:
                        current -= 65536
                    self.currents[index] = current
        except KeyboardInterrupt:
            quit()

        if verbose:
            print(f"Motor ID {self.motor_ids[0]} position: {self.positions[0]} deg: {self.pos_to_deg(self.positions[0])} Motor ID {self.motor_ids[1]} position: {self.positions[1]} deg: {self.pos_to_deg(self.positions[1])}")
            
        # if not self.positions_healthy():
        #     quit()
            
            
    def positions_healthy(self):
        for position in self.positions:
            if not (1500 <= position <= 2500):
                return False

        return True
      
    def normal_forces_to_average_torque(self, left_motor_force, right_motor_force):
        average_force = (left_motor_force + right_motor_force) / 2
        average_torque = average_force * self.motor_finger_radius
        return average_torque
    
    def torques_to_currents(self, left_motor_torque, right_motor_torque):
        left_motor_current = left_motor_torque / self.torque_constant
        right_motor_current = right_motor_torque / self.torque_constant
        return left_motor_current, right_motor_current
    
    def pos_to_deg(self, pos):
        """ position is in thicks from 0 to 4095. 1 deg is 4095 / 360 = 11.375 """
        deg = pos / 11.375
        return deg
                
    def disable_torque(self, motor_id, script_accesed_from_outside = True):
        if script_accesed_from_outside and not self.script_public:
            return

        try:
            # Disable torque by writing 0 to the Torque Enable register
            result, error = self.packetHandler.write1ByteTxRx(self.portHandler, motor_id, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
            if result != COMM_SUCCESS:
                print(f"Failed to disable torque for motor {motor_id}: {self.packetHandler.getTxRxResult(result)}")
            elif error != 0:
                print(f"Hardware error for motor {motor_id}: {self.packetHandler.getRxPacketError(error)}")
            return result, error
        except KeyboardInterrupt:
            quit()
            
    def disable_torques(self, script_accesed_from_outside = True):
        if script_accesed_from_outside and not self.script_public:
            return

        # Print call stack
        # print("Call stack leading to disable_torques:")
        # traceback.print_stack()
        try:
            group_bulk_write = GroupBulkWrite(self.portHandler, self.packetHandler)
            
            for motor_id in self.motor_ids:
                param = [TORQUE_DISABLE]
                if not group_bulk_write.addParam(motor_id, ADDR_TORQUE_ENABLE, 1, param):
                    print(f"Failed to add motor {motor_id} to bulk write.")
                    return
            
            result = group_bulk_write.txPacket()
            if result != COMM_SUCCESS:
                print(f"Failed to disable torques: {self.packetHandler.getTxRxResult(result)}")
            else:
                self.torque_enable = TORQUE_DISABLE            

            group_bulk_write.clearParam()
            
            return result
        except KeyboardInterrupt:
            quit()
            
    def enable_torques(self, script_accesed_from_outside = True):
        if script_accesed_from_outside and not self.script_public:
            return
    
        try:
            group_bulk_write = GroupBulkWrite(self.portHandler, self.packetHandler)
            
            for motor_id in self.motor_ids:
                param = [TORQUE_ENABLE]
                if not group_bulk_write.addParam(motor_id, ADDR_TORQUE_ENABLE, 1, param):
                    print(f"Failed to add motor {motor_id} to bulk write.")
                    return
            
            result = group_bulk_write.txPacket()
            if result != COMM_SUCCESS:
                print(f"Failed to enable torques: {self.packetHandler.getTxRxResult(result)}")
            else:
                print("Successfully enabled torques for all motors.")
                self.torque_enable = TORQUE_ENABLE            

            group_bulk_write.clearParam()
            
            return result
        except KeyboardInterrupt:
            quit()

    def close_port(self):
        self.portHandler.closePort()
        print("Motor port closed")

    def cleanup_on_exit(self):
        print("Program is exiting. Cleaning up resources...")
        self.script_public = False
        self.portHandler.ser.flush()

        for motor_id in self.motor_ids:
            self.set_goal_torque(motor_id, 0, script_accesed_from_outside=False)
            self.set_pwm(motor_id, 0, script_accesed_from_outside=False)
            self.disable_torque(motor_id, script_accesed_from_outside=False)
        
        self.close_port()

        
    def plot(self, shifted_array = None, delay = None):
        fig, axs = plt.subplots(2, 1, figsize=(8, 6), sharex=True)  # 2 rows, 1 column

        axs[0].plot(self.present_current, label='Present Current', color='b')
        axs[0].plot(self.target_current, label='Target Current', color='r')
        if shifted_array is not None:
            axs[0].plot(shifted_array, label=f"Present current shifted by {delay} iterations", color='g')
        
        axs[0].set_title('Present Current vs Target Current')
        axs[0].set_ylabel('Current (A)')
        axs[0].legend()
        axs[0].grid(True)

        axs[1].plot(self.pwm_values, label='PWM Values', color='g', marker='s')
        
        axs[1].set_title('PWM Values Over Iterations')
        axs[1].set_xlabel('Iterations')
        axs[1].set_ylabel('PWM Value')
        axs[1].legend()
        axs[1].grid(True)

        plt.tight_layout()
        plt.show()
        
    def plot_current_control(self, shifted_array = None, delay = None):
        fig, ax = plt.subplots(figsize=(8, 6))  # Single plot (remove 2 rows)

        ax.plot(self.present_current, label='Present Current', color='b')
        ax.plot(self.target_current, label='Target Current', color='r')
        if shifted_array is not None:
            ax.plot(shifted_array, label=f"Present current shifted by {delay} iterations", color='g')
        
        ax.set_title('Present Current vs Target Current')
        ax.set_xlabel('Iterations')
        ax.set_ylabel('Current (A)')
        ax.legend()
        ax.grid(True)

        plt.tight_layout()
        plt.show()

    
    def train_target_current_function(self, i):
        # Changed COM5 port response time from 16ms to 1ms. Take changes accordingly
        i //= 6

        return 0.2 if 100 < i <= 200 else 0.1
    
    def test_target_current_function(self, i):
        # Changed COM5 port response time from 16ms to 1ms. Take changes accordingly
        i //= 6
        
        if i < 100:
            return 0.2
        elif i < 200:
            return 0.1
        elif i < 300:
            return 0.2
        elif i < 750:
            period = 100
            scale = 0.2
            return scale * abs(2/period * (i % period) - 1)
        else:
            period = 50
            scale = 0.1
            return scale/2 * math.sin(2 * math.pi * (i - period/4) / period) + scale/2
        
    def get_gripper_finger_distance(self, scale_range = None):
        scaled_distance = None
        deg1 = self.get_present_position_deg(self.motor_ids[0])
        deg2 = self.get_present_position_deg(self.motor_ids[1])
        
        # subtract 90 to make the angle 0 at x axis position
        deg1 -= 90
        deg2 -= 90
        
        projection1 = self.motor_finger_radius * math.cos(math.radians(deg1))
        projection1 *= -1 # Swap direction
        projection2 = self.motor_finger_radius * math.cos(math.radians(deg2))
        
        finger_distance = self.distance_between_motors + projection1 + projection2
        
        return finger_distance
    
    def get_scaled_finger_distance(self, scale_min, scale_max):
        scale = scale_max / self.distance_between_motors
        scaled_distance = scale * self.get_gripper_finger_distance()
        scaled_distance = max(min(scaled_distance, scale_max), scale_min)
        
        return scaled_distance
        
    
    ### PWM CONTROL ###
    def test_apply_current_compensation_PWM_PID(self, target_current_function, motor_id = 2, kp = 150, ki = 110 , kd = 1, iterations = 1800):
        self.train_reset()
        self.reset_operating_mode_pwm()
        sum_absolute_arror = 0
        
        max_p, max_i, max_d = 0, 0, 0
        for i in range(iterations):
            if not self.script_public:
                self.disable_torque(motor_id)
                return 0
            
            self.update_state()

            target_current = target_current_function(i)
            if self.previous_time == None:
                self.previous_time = time.time()
                self.prev_error = target_current - self.get_present_current(motor_id)
                continue

            current_time = time.time()
            dt = current_time - self.previous_time
            if dt < 1.01/1000:
                continue
            
            present_current = self.get_present_current(motor_id)
            
            current_error = target_current - present_current
            sum_absolute_arror += abs(current_error)
            
            positional = current_error
            positional *= kp
            self.integral_error += current_error * dt
            integral = ki * self.integral_error
            integral = max(min(integral, self.PWM_limit), -self.PWM_limit)
            derivative = (current_error - self.prev_error) / dt
            derivative *= kd
            pwm_value = int(positional + integral + derivative)
            self.set_pwm(motor_id, pwm_value)
            
            # for plotting
            self.present_current.append(present_current)
            self.target_current.append(target_current)
            self.pwm_values.append(pwm_value)
            
            self.previous_time = current_time
            self.prev_error = current_error
        
        
        self.set_pwm(motor_id, 0)
        
        return sum_absolute_arror
    
    ## CURRENT CONTROL ##
    def test_apply_current_compensation_CURRENT_BUILT_IN(self, target_current_function, motor_id = 2, iterations = 1800):
        self.train_reset()
        self.reset_operating_mode_current()
        sum_absolute_arror = 0
        for i in range(iterations):
            if not self.script_public:
                self.disable_torque(motor_id)
                return 0
            
            self.update_state()

            target_current = target_current_function(i)
            present_current = self.get_present_current(motor_id)
            
            current_error = target_current - present_current
            sum_absolute_arror += abs(current_error)
            
            self.set_current(motor_id, target_current)
            
            # for plotting
            self.present_current.append(present_current)
            self.target_current.append(target_current)
        
        self.set_current(motor_id, 0)
        
        return sum_absolute_arror
    
    def test_delays_direct_current_control(self, left_pad_normal_force, right_pad_normal_force):
        average_torque = self.normal_forces_to_average_torque(left_pad_normal_force, right_pad_normal_force)

        for window in [5, 10, 15, 20, 30, 50]:
            alpha = 2 / (window + 1)
            
            if window not in self.smoothed_curves:
                self.smoothed_curves[window] = 0.0
            
            average_current, _ = self.torques_to_currents(average_torque, average_torque)
            self.smoothed_curves[window] = (alpha * average_current +
                                    (1 - alpha) * self.smoothed_curves[window])

        left_pad_torque, right_pad_torque = average_torque, -average_torque
        if left_pad_normal_force + right_pad_normal_force < 0.01:
            if self.torque_enable == TORQUE_ENABLE:
                self.disable_torques()
        else:
            if self.torque_enable == TORQUE_DISABLE:
                self.enable_torques()
            self.set_goal_torques(left_pad_torque, right_pad_torque)
            
        # for plotting
        left_pad_current, right_pad_current = self.torques_to_currents(left_pad_torque, right_pad_torque)
        return self.smoothed_curves, left_pad_current, right_pad_current, self.get_present_current(self.left_motor_id), self.get_present_current(self.right_motor_id)
    
    def direct_current_control(self, left_pad_normal_force, right_pad_normal_force, exponential_smoothing = False, window = 10):
        average_torque = self.normal_forces_to_average_torque(left_pad_normal_force, right_pad_normal_force)
        if exponential_smoothing:
            if window != self.window_size:
                self.window_size = window
                self.alpha = 2 / (self.window_size + 1)
            self.smoothed_average_torque = (self.alpha * average_torque +
                                        (1 - self.alpha) * self.smoothed_average_torque)
            average_torque = self.smoothed_average_torque
        left_pad_torque, right_pad_torque = average_torque, -average_torque
        average_current, _ = self.torques_to_currents(average_torque, average_torque)
        if average_current < 0.001:
            if self.torque_enable == TORQUE_ENABLE:
                self.disable_torques()
        else:
            if self.torque_enable == TORQUE_DISABLE:
                self.enable_torques()
            self.set_goal_torques(left_pad_torque, right_pad_torque)
            
        # for plotting
        left_pad_current, right_pad_current = self.torques_to_currents(left_pad_torque, right_pad_torque)
        return left_pad_current, right_pad_current, self.get_present_current(self.left_motor_id), self.get_present_current(self.right_motor_id)

    def pwm_control(self, left_pad_normal_force, right_pad_normal_force, kp, ki, kd, update_state):
        average_torque = self.normal_forces_to_average_torque(left_pad_normal_force, right_pad_normal_force)
        left_pad_torque, right_pad_torque = average_torque, -average_torque
        left_pad_current, right_pad_current = self.torques_to_currents(left_pad_torque, right_pad_torque)
        if left_pad_normal_force + right_pad_normal_force < 0.01:
            if self.torque_enable == TORQUE_ENABLE:
                self.integral_error = 0
                self.previous_time = time.time()
                self.disable_torques()
        else:
            if self.torque_enable == TORQUE_DISABLE:
                self.enable_torques()
        
        if update_state:
            self.update_state()

        # Do PID only for left motor and set right as the opposite value
        target_current = left_pad_current
        if self.previous_time == None:
            self.previous_time = time.time()
            self.prev_error = target_current - self.get_present_current(self.left_motor_id)
            return left_pad_current, right_pad_current, self.get_present_current(self.left_motor_id), self.get_present_current(self.right_motor_id), self.last_pwm, -self.last_pwm

        current_time = time.time()
        dt = current_time - self.previous_time
        if dt < 1.01/1000:
            return left_pad_current, right_pad_current, self.get_present_current(self.left_motor_id), self.get_present_current(self.right_motor_id), self.last_pwm, -self.last_pwm
        
        present_current = self.get_present_current(self.left_motor_id)
        
        current_error = target_current - present_current
        
        positional = current_error
        positional *= kp
        self.integral_error += current_error * dt
        integral = ki * self.integral_error
        integral = max(min(integral, self.PWM_limit), -self.PWM_limit)
        derivative = (current_error - self.prev_error) / dt
        derivative *= kd
        # derivative = max(min(derivative, 30), -30)
        pwm_value = int(positional + integral + derivative)
        self.last_pwm = pwm_value
        self.set_pwm(self.left_motor_id, pwm_value)
        self.set_pwm(self.right_motor_id, -pwm_value)
        
        self.previous_time = current_time
        self.prev_error = current_error
        
        # for plotting
        return left_pad_current, right_pad_current, self.get_present_current(self.left_motor_id), self.get_present_current(self.right_motor_id), pwm_value, -pwm_value
 