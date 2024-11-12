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
        

def get_jacobian(joint_axes, joint_positions, pos_end_effector, orientation_rpy, num_joints = 6):
    joint_axes = [(0.0, 0.0, 1.0), (1.0, 0.0, 0.0), (1.0, 0.0, 0.0), (1.0, 0.0, 0.0), (0.0, 0.0, 1.0), (0.0, 1.0, 0.0)]
    Jv = np.zeros((3, num_joints))
    Jw = np.zeros((3, num_joints))

    pos_end_effector = np.array(pos_end_effector)
    joint_axes = np.array(joint_axes)
    joint_positions = np.array(joint_positions)
    for i in range(num_joints):
        Jv[:, i] = np.cross(joint_axes[i], (pos_end_effector - joint_positions[i]))
        Jw[:, i] = joint_axes[i]
    
    return  Jv, Jw


def getMotorJointStates(robot):
  joint_states = p.getJointStates(robot, range(p.getNumJoints(robot)))
  joint_infos = [p.getJointInfo(robot, i) for i in range(p.getNumJoints(robot))]
  joint_states = [j for j, i in zip(joint_states, joint_infos) if i[3] > -1]
  joint_positions = [state[0] for state in joint_states]
  joint_velocities = [state[1] for state in joint_states]
  joint_torques = [state[3] for state in joint_states]
  return joint_positions, joint_velocities, joint_torques

def get_quaternion_derivative(angular_velocity, robot):
    wx, wy, wz = angular_velocity

    q0, q1, q2, q3 = robot.get_ee_link_pose()[1]

    q_dot_0 = 0.5 * (-q1 * wx - q2 * wy - q3 * wz)
    q_dot_1 = 0.5 * (q0 * wx + q2 * wz - q3 * wy)
    q_dot_2 = 0.5 * (q0 * wy - q1 * wz + q3 * wx)
    q_dot_3 = 0.5 * (q0 * wz + q1 * wy - q2 * wx)

    q_dot = np.array([q_dot_0, q_dot_1, q_dot_2, q_dot_3])

    return q_dot

def test_jacobian(robot, joint_indices, jacobian_func, ee_link_index, pybullet_jacobian = False):
    delta_q = np.zeros(len(joint_indices))
    delta_v = 1

    for i in range(len(joint_indices)):
        delta_q[i] = delta_v

        p.setJointMotorControlArray(bodyIndex=robot.robot_id,
                        jointIndices=joint_indices,
                        controlMode=p.VELOCITY_CONTROL,
                        targetVelocities=delta_q)
        
        for _ in range(100):
          p.stepSimulation()

        _, _, _, _, _, _, link_linear_velocity, link_angular_velocity = p.getLinkState(
            robot.robot_id,
            robot.eef_id,
            computeLinkVelocity=1,
            computeForwardKinematics=1
        )
        actual_ee_velocity = np.hstack((link_linear_velocity, link_angular_velocity))

        
        mpos, mvel, mtorq = getMotorJointStates(robot.robot_id)
        result = p.getLinkState(robot.robot_id,
                                robot.eef_id,
                                computeLinkVelocity=1,
                                computeForwardKinematics=1)
        link_trn, link_rot, com_trn, com_rot, frame_pos, frame_rot, link_vt, link_vr = result
        zero_vec = [0.0] * len(mpos)
        jac_t, jac_r = p.calculateJacobian(robot.robot_id, robot.eef_id, com_trn, mpos, zero_vec, zero_vec)

        j_t = np.array([jac_t[0][:6], jac_t[1][:6], jac_t[2][:6]])
        j_r = np.array([jac_r[0][:6], jac_r[1][:6], jac_r[2][:6]])
        jacobian_pybullet = np.vstack((j_t, j_r))
        expected_ee_velocity_pybullet = jacobian_pybullet @ delta_q

        print(f"Testing joint {i+1}")
        print(f"actual velocity {actual_ee_velocity}")
        print("Expected EE Velocity py:", expected_ee_velocity_pybullet)
        py_diff = actual_ee_velocity - expected_ee_velocity_pybullet
        print(f"Difference pybullet: pos {np.linalg.norm(py_diff[:3])} rot {np.linalg.norm(py_diff[3:])}")
        print("="*30)

        delta_q[i] = 0.0
        time.sleep(0.1)

test_jacobian(robot, joint_indices=[1, 2, 3, 4, 5, 6], jacobian_func=get_jacobian, ee_link_index=robot.eef_id)

p.disconnect()
