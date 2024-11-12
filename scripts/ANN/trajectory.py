import numpy as np
import pybullet as p

class Trajectory():
  def __init__(self, TIME_STEP, robot, main_loop) -> None:
    self.TIME_STEP = TIME_STEP
    self.robot = robot
    self.main_loop = main_loop
    
  def linear_interpolation(self, start, end, duration, t):
    return start + (end - start) * (t / duration)

  def slerp(self, Q1, Q2, duration, t):
      # semester 1 robotics lecture 3
      cos_theta = np.dot(Q1, Q2) / (np.linalg.norm(Q1) * np.linalg.norm(Q2))
      cos_theta = np.clip(cos_theta, -1.0, 1.0)
      y = np.arccos(cos_theta)
      p = t / duration
      
      if np.isclose(y, 0):
          return Q2

      Q = (np.sin((1 - p) * y) / np.sin(y)) * Q1 + (np.sin(p * y) / np.sin(y)) * Q2
      
      return Q
      
  def generate_trajectory(self, points, orientations, gripper, durations):
      trajectory = []
      for i in range(len(points) - 1):
          P1, P2 = np.array(points[i]), np.array(points[i + 1])
          Q1, Q2 = np.array(orientations[i]), np.array(orientations[i + 1])
          G1, G2 = np.array(gripper[i]), np.array(gripper[i + 1])
          duration = durations[i]

          for t in np.arange(0, duration, self.TIME_STEP):
              pos = self.linear_interpolation(P1, P2, duration, t)
              ori = self.slerp(Q1, Q2, duration, t)
              gri = self.linear_interpolation(G1, G2, duration, t)
              trajectory.append((pos, ori, gri))
  
      return trajectory
    
  def follow_trajectory(self, trajectory):
    for point in trajectory:
      target_position = point[0]
      target_orientation = point[1]
      target_gripper = point[2]

      joint_positions = p.calculateInverseKinematics(self.robot.robot_id, self.robot.eef_id, target_position, targetOrientation=target_orientation)

      p.setJointMotorControlArray(bodyIndex=self.robot.robot_id, jointIndices=[1, 2, 3, 4, 5, 6], controlMode=p.POSITION_CONTROL, targetPositions=joint_positions[:6])
      self.robot.move_gripper_length(target_gripper)

      self.main_loop()

  # trajectory = generate_trajectory(target_positions, target_orientations, target_gripper, durations)