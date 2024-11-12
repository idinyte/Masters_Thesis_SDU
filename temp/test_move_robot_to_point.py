import pybullet as p
import numpy as np
import os
from scripts.objects.ur5 import UR5Robot
import time

p.connect(p.GUI)
p.setGravity(0, 0, -9.81)

robot = UR5Robot(urdf_path=os.path.join(os.getcwd(), "assets/objects/UR5/urdf/ur5_robotiq_140_modified.urdf"), base_position=[0, 0, 0], base_orientation=[0, 0, 0, 1], use_fixed_base=True)

robot.load()
robot.step_simulation = p.stepSimulation()
robot.reset()

def move_to_position(robot, target_position, target_orientation_euler, joint_indices, tolerance=1e-3, max_iterations=1000):
    for iteration in range(max_iterations):
        current_position, current_orientation_euler = robot.get_ee_link_pose()

        position_error = np.array(target_position) - np.array(current_position)
        orientation_error = np.array(target_orientation_euler) - np.array(current_orientation_euler)
        orientation_error *= 0.011
        error = np.hstack((position_error, orientation_error))
        error_magnitude = np.linalg.norm(error)

        if error_magnitude < tolerance:
            print("Target reached within tolerance.")
            break

        Jv, Jw = robot.get_jacobian()
        J = np.vstack((Jv, Jw))
        roll, pitch, yaw = current_orientation_euler
        B = np.array([
            [1, 0, np.sin(pitch)],
            [0, np.cos(roll), -np.cos(pitch) * np.sin(roll)],
            [0, np.sin(roll), np.cos(pitch) * np.cos(roll)]
        ])

        I = np.eye(3)
        zeros = np.zeros((3, 3))
        T_euler = np.vstack((np.hstack((I, zeros)), np.hstack((zeros, B))))

        jacobian_analytical = T_euler @ np.vstack((Jv, Jw))

        jacobian_inverse = np.linalg.inv(jacobian_analytical)

        delta_q = jacobian_inverse @ error

        for i, joint_index in enumerate(joint_indices):
            current_joint_position = p.getJointState(robot.robot_id, joint_index)[0]
            new_joint_position = current_joint_position + delta_q[i]
            p.resetJointState(robot.robot_id, joint_index, new_joint_position)

        p.stepSimulation()

    if error_magnitude >= tolerance:
        print("Max iterations reached; target may not be fully reached.")

target_position = [0.5, 0.0, 0.5]
move_to_position(robot, target_position, [0, np.pi/2, -np.pi/2], joint_indices=[1, 2, 3, 4, 5, 6])

print(robot.get_ee_link_pose())
time.sleep(100)

p.disconnect()
