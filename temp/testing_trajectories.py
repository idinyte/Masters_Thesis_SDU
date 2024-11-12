import pybullet as p
import numpy as np
import time
from scipy.spatial.transform import Rotation as R, Slerp
import os
from scripts.objects.ur5 import UR5Robot

# Initialize PyBullet
p.connect(p.GUI)
p.setGravity(0, 0, -9.81)

# Load robot URDF and set up end-effector link index
robot = UR5Robot(urdf_path=os.path.join(os.getcwd(), "assets/objects/UR5/urdf/ur5_robotiq_140_modified.urdf"), base_position=[0, 0, 0], base_orientation=[0.0, 0.0, 0, 1], use_fixed_base=True)
end_effector_link_index = 7  # Replace with actual index of your robot's end-effector

 # Load robot
robot.load()
robot.step_simulation = p.stepSimulation()
robot.reset()

joint_index = 6

# Get joint information for verification (optional)
joint_info = p.getJointInfo(robot.robot_id, joint_index)
print("Joint info:", joint_info)

# Remove joint limits by setting a large range for the joint
p.changeDynamics(robot.robot_id, joint_index, jointLowerLimit=-2*np.pi, jointUpperLimit=2*np.pi)
p.stepSimulation()
# Get joint information for verification (optional)
joint_info = p.getJointInfo(robot.robot_id, joint_index)
print("Joint info:", joint_info)

        
# Desired trajectory points for position and orientation
target_positions = [
    [0.5, -0.5, 0.5], [-0.5, -0.5, 0.5], [0.5, -0.5, 0.5], [-0.5, -0.5, 0.5],
    [0.5, -0.5, 0.5], [-0.5, -0.5, 0.5], [0.5, -0.5, 0.5], [-0.5, -0.5, 0.5],
    [0.5, -0.5, 0.5], [-0.5, -0.5, 0.5], [0.5, -0.5, 0.5], [-0.5, -0.5, 0.5],
    [0.5, -0.5, 0.5], [-0.5, -0.5, 0.5], [0.5, -0.5, 0.5], [-0.5, -0.5, 0.5],
]


target_quaternion = p.getQuaternionFromEuler([0, np.pi/2, np.pi/2])

ori1 = p.getQuaternionFromEuler([0, np.pi/2, 0])
ori2 = p.getQuaternionFromEuler([0, np.pi/2, np.pi/2])
ori3 = p.getQuaternionFromEuler([0, np.pi/2, 3*np.pi/4])
ori4 = p.getQuaternionFromEuler([0, np.pi/2, np.pi])

target_orientations = [[0.7071067811865476, 0.0, 0.7071067811865476, 0.0],
[0.6935199226610738, -0.1379496896414715, 0.6935199226610738, 0.1379496896414715],
[0.6532814824381883, -0.2705980500730985, 0.6532814824381883, 0.2705980500730985],
[0.5879378012096794, -0.39284747919355106, 0.5879378012096794, 0.39284747919355106],
[0.5000000000000001, -0.5000000000000001, 0.5000000000000001, 0.5000000000000001],
[0.3928474791935512, -0.5879378012096794, 0.3928474791935512, 0.5879378012096794],
[0.27059805007309856, -0.6532814824381883, 0.27059805007309856, 0.6532814824381883],
[0.13794968964147156, -0.6935199226610738, 0.13794968964147156, 0.6935199226610738],
[4.329780281177467e-17, -0.7071067811865476, 4.329780281177467e-17, 0.7071067811865476],
[-0.13794968964147147, -0.6935199226610738, -0.13794968964147147, 0.6935199226610738],
[-0.2705980500730985, -0.6532814824381883, -0.2705980500730985, 0.6532814824381883],
[-0.39284747919355095, -0.5879378012096794, -0.39284747919355095, 0.5879378012096794],
[-0.5, -0.5000000000000001, -0.5, 0.5000000000000001],
[-0.5879378012096794, -0.39284747919355106, -0.5879378012096794, 0.39284747919355106],
[-0.6532814824381883, -0.2705980500730986, -0.6532814824381883, 0.2705980500730986],
[-0.6935199226610738, -0.13794968964147175, -0.6935199226610738, 0.13794968964147175]]
target_orientations = [p.getQuaternionFromEuler([0, np.pi/2, np.pi/2]) for _ in range(16)]

target_gripper = [0, 0, 0, 0, 0.12, 0.12, 0.12, 0.12, 0, 0, 0, 0, 0.12, 0.12, 0.12, 0.12]

durations = [2, 2, 2, 2, 1, 1, 1, 1, 2, 2, 2, 2, 1, 1, 1, 1]
blendiness = 0
sampling_interval = 0.05
kp_pos = 1000
kp_ori = 0

def linear_interpolation(start, end, duration, t):
    return start + (end - start) * (t / duration)

def slerp(Q1, Q2, duration, t):
    # semester 1 robotics lecture 3
    y = np.arccos(np.dot(Q1, Q2) / (np.linalg.norm(Q1) * np.linalg.norm(Q2)))
    
    p = t / duration
    
    if np.isclose(y, 0):
        return Q2

    Q = (np.sin((1 - p) * y) / np.sin(y)) * Q1 + (np.sin(p * y) / np.sin(y)) * Q2
    
    return Q
    
def generate_trajectory(points, orientations, gripper, durations, sampling_interval=0.05):
    trajectory = []
    for i in range(len(points) - 1):
        P1, P2 = np.array(points[i]), np.array(points[i + 1])
        duration = durations[i]
        Q1, Q2 = np.array(orientations[i]), np.array(orientations[i + 1])
        G1, G2 = np.array(gripper[i]), np.array(gripper[i + 1])

        for t in np.arange(0, duration, sampling_interval):
            pos = linear_interpolation(P1, P2, duration, t)
            ori = slerp(Q1, Q2, duration, t)
            gri = linear_interpolation(G1, G2, duration, t)
            trajectory.append((pos, ori, gri))
    return trajectory

trajectory = generate_trajectory(target_positions, target_orientations, target_gripper, durations)

def get_jacobian():
    mpos, mvel, mtorq = robot.getMotorJointStates()
    result = p.getLinkState(robot.robot_id,
                            robot.eef_id,
                            computeLinkVelocity=1,
                            computeForwardKinematics=1)

    zero_vec = [0.0] * len(mpos)
    jac_t, jac_r = p.calculateJacobian(robot.robot_id, robot.eef_id, result[2], mpos, zero_vec, zero_vec)

    j_t = np.array([jac_t[0][:6], jac_t[1][:6], jac_t[2][:6]])
    j_r = np.array([jac_r[0][:6], jac_r[1][:6], jac_r[2][:6]])
    jacobian = np.vstack((j_t, j_r))
    
    return  jacobian



# print("j")
# print(jacobian)
# Apply torque to make the joint spin

# p.setJointMotorControl2(robot.robot_id, jointIndex=1, controlMode=p.TORQUE_CONTROL, force=500.0) 
# for _ in range(1000000):
#     p.stepSimulation()
#     time.sleep(0.0001)
# time.sleep(30)

# Apply the trajectory in PyBullet
for point in trajectory:
    target_position = point[0]
    target_orientation = point[1]
    target_gripper = point[2]
    # Set the end effector position in task space using inverse kinematics
    joint_positions = p.calculateInverseKinematics(robot.robot_id, 7, target_position, targetOrientation=target_orientation)
    #print(robot.get_ee_link_pose()[1])
    
    # Apply joint positions to the robot
    p.setJointMotorControlArray(bodyIndex=robot.robot_id, jointIndices=[1, 2, 3, 4, 5, 6], controlMode=p.POSITION_CONTROL, targetPositions=joint_positions[:6])
    robot.move_gripper_length(target_gripper)
    
    print(f"{target_gripper}")
    # Step the simulation
    p.stepSimulation()
    time.sleep(sampling_interval)



# p.setJointMotorControlArray(robot.robot_id, robot.arm_controllable_joints[:6], p.VELOCITY_CONTROL, forces=[0, 0, 0, 0, 0, 0])
# # Jacobian Transpose Control
# for pos, ori in trajectory:
#     # Compute position and or ientation error
#     current_pos, current_ori = p.getLinkState(robot.robot_id, robot.eef_id)[:2]
#     current_ori = np.array(p.getMatrixFromQuaternion(current_ori)).reshape(3, 3)
    
#     pos_error = np.array([0.5, -0.5, 0.5]) - np.array(current_pos)
#     ori_error_mat = ori @ current_ori.T
#     ori_error_vec = R.from_matrix(ori_error_mat).as_rotvec()  # Convert to rotation vector
#     ori_error_vec = [0, 0, 0]

#     current_pos, current_ori = p.getLinkState(robot.robot_id, robot.eef_id)[:2]
#     current_ori = np.array(p.getMatrixFromQuaternion(current_ori)).reshape(3, 3)
    
#     pos_error = np.array(pos) - np.array(current_pos)
#     ori_error_mat = ori @ current_ori.T
#     ori_error_vec = R.from_matrix(ori_error_mat).as_rotvec()  # Convert to rotation vector

#     # Calculate the Jacobian
#     ee_position, ee_rpy = robot.get_com_ee_link_pose()
#     jv, jw = get_jacobian(robot.controllable_joints_axis[:6], robot.get_arm_link_positions(), ee_position, ee_rpy)
    
    
#     mpos, mvel, mtorq = getMotorJointStates(robot.robot_id)

#     result = p.getLinkState(robot.robot_id,
#                             robot.eef_id,
#                             computeLinkVelocity=1,
#                             computeForwardKinematics=1)
#     link_trn, link_rot, com_trn, com_rot, frame_pos, frame_rot, link_vt, link_vr = result
#     # Get the Jacobians for the CoM of the end-effector link.
#     # Note that in this example com_rot = identity, and we would need to use com_rot.T * com_trn.
#     # The localPosition is always defined in terms of the link frame coordinates.

#     zero_vec = [0.0] * len(mpos)
#     jac_t, jac_r = p.calculateJacobian(robot.robot_id, robot.eef_id, com_trn, mpos, zero_vec, zero_vec)
    
#     j_t = np.array([jac_t[0][:6], jac_t[1][:6], jac_t[2][:6]])
#     j_r = np.array([jac_r[0][:6], jac_r[1][:6], jac_r[2][:6]])
#     jacobian = np.vstack((j_t, j_r))

#     # Convert errors into joint torques using the Jacobian transpose
#     task_force = kp_pos * pos_error
#     task_torque = kp_ori * ori_error_vec

#     task_wrench = np.hstack([task_force, task_torque])
#     joint_torques = jacobian.T @ task_wrench

#     # Apply joint torques to the robot
#     p.setJointMotorControlArray(bodyIndex=robot.robot_id, jointIndices=robot.controllable_joints[:6], controlMode=p.TORQUE_CONTROL, forces=joint_torques)
    
#     # Step simulation
#     p.stepSimulation()
#     time.sleep(sampling_interval)

# Disconnect after execution
p.disconnect()
