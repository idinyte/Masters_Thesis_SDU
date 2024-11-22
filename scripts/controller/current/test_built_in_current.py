import sys
import os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../../..')))
from scripts.objects.gripper import GripperMotors
import numpy as np
import time

MOTOR_ID = 2

class BuiltInCurrentAlgorithm:
    def __init__(self):
        self.gripper_motors = GripperMotors()

    def evaluate_fitness(self):
        start = time.time()
        # iterations = 6000
        # fitness = self.gripper_motors.train_apply_current_compensation_CURRENT_BUILT_IN(self.gripper_motors.test_target_current_function, motor_id=MOTOR_ID, iterations=iterations)
        iterations = 600
        fitness = self.gripper_motors.train_apply_current_compensation_CURRENT_BUILT_IN(lambda x: 0.1, motor_id=MOTOR_ID, iterations=iterations)
        end = time.time()
        delay, shifted_array = self.gripper_motors.get_delay_between_present_current_and_target()
        iteration_time_ms = 1000*(end - start)/iterations
        delay_ms = delay * iteration_time_ms
        print(f"Built in current control. {iterations} iterations. Fitness = {fitness}. Iterration time = {iteration_time_ms} ms. Delay = {delay_ms} ms")
        self.gripper_motors.plot_current_control(shifted_array, delay)
        self.gripper_motors.plot_current_control()
        return fitness

   

algorithm = BuiltInCurrentAlgorithm()
algorithm.evaluate_fitness()