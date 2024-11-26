import sys
import os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../../..')))
from scripts.objects.gripper import GripperMotors
import random
import numpy as np
import time

KP = 28.809
KI = 503.211
KD = 0.191
MOTOR_ID = 2

class GeneticAlgorithmPID:
    def __init__(self):
        self.gripper_motors = GripperMotors()


    def evaluate_fitness(self):
        start = time.time()
        # iterations = 6000
        # fitness = self.gripper_motors.test_apply_current_compensation_PWM_PID(self.gripper_motors.test_target_current_function, motor_id=MOTOR_ID, kp = KP, ki = KI, kd = KD, iterations=iterations)
        iterations = 500
        fitness = self.gripper_motors.test_apply_current_compensation_PWM_PID(lambda x: 0.1, motor_id=MOTOR_ID, kp = KP, ki = KI, kd = KD, iterations=iterations)
        end = time.time()
        delay, shifted_array = self.gripper_motors.get_delay_between_present_current_and_target()
        iteration_time_ms = 1000*(end - start)/iterations
        delay_ms = delay * iteration_time_ms
        print(f"PWM control. Genetic algorithm. {iterations} iterations. Kp = {KP}, KI = {KI}, KD = {KD}. Fitness =  {fitness}. Iterration time = {iteration_time_ms} ms. Delay = {delay_ms} ms")
        self.gripper_motors.plot(shifted_array, delay)
        self.gripper_motors.plot()
        return fitness

   

ga_pid = GeneticAlgorithmPID()
ga_pid.evaluate_fitness()