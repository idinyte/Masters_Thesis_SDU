import pybullet as p
from collections import namedtuple
import os
import numpy as np
import matplotlib.pyplot as plt
from numpy.polynomial import Polynomial

class UR5Robot:
    def __init__(self, urdf_path, base_position=[0, 0, 0], base_orientation=[0, 0, 0, 1], use_fixed_base=True):
        self.eef_id = 7
        self.arm_num_dofs = 6
        self.arm_rest_poses = [-1.57, -1.55, 1.34, -1.37, -1.57, 0]
        self.gripper_range = [0, 0.127] # max should be 0.14, but when actually measured in simulation it's less
        self.debug_line_handle = None
      
        # Load URDF for UR5 robot
        self.urdf_path = urdf_path
        self.base_position = base_position
        self.base_orientation = base_orientation
        self.use_fixed_base = use_fixed_base
        
    def load(self):
        self.robot_id = self._load_ur5()
        
        self._parse_joint_info()
        self._set_robot_arm_limits()
        self._gripper_contraints()
        
    def _parse_joint_info(self):
        """Populate self.joints"""
        numJoints = p.getNumJoints(self.robot_id)
        jointInfo = namedtuple('jointInfo', 
            ['id','name','type','damping','friction','lowerLimit','upperLimit','maxForce','maxVelocity','controllable'])
        self.joints = []
        self.controllable_joints = []
        for i in range(numJoints):
            info = p.getJointInfo(self.robot_id, i)
            jointID = info[0]
            jointName = info[1].decode("utf-8")
            jointType = info[2]  # JOINT_REVOLUTE, JOINT_PRISMATIC, JOINT_SPHERICAL, JOINT_PLANAR, JOINT_FIXED
            jointDamping = info[6]
            jointFriction = info[7]
            jointLowerLimit = info[8]
            jointUpperLimit = info[9]
            jointMaxForce = info[10]
            jointMaxVelocity = info[11]
            controllable = (jointType != p.JOINT_FIXED)
            if controllable:
                self.controllable_joints.append(jointID)
                p.setJointMotorControl2(self.robot_id, jointID, p.VELOCITY_CONTROL, targetVelocity=0, force=0)
            info = jointInfo(jointID,jointName,jointType,jointDamping,jointFriction,jointLowerLimit,
                            jointUpperLimit,jointMaxForce,jointMaxVelocity,controllable)
            self.joints.append(info)
        
    def _set_robot_arm_limits(self):
      assert len(self.controllable_joints) >= self.arm_num_dofs

      self.arm_controllable_joints = self.controllable_joints[:self.arm_num_dofs]
      self.arm_lower_limits = [joint.lowerLimit for joint in self.joints if joint.controllable][:self.arm_num_dofs]
      self.arm_upper_limits = [joint.upperLimit for joint in self.joints if joint.controllable][:self.arm_num_dofs]
      self.arm_joint_ranges = [joint.upperLimit - joint.lowerLimit for joint in self.joints if joint.controllable][:self.arm_num_dofs]
        
    def _gripper_contraints(self):
      """Move children gripper joints according to parent joint"""
      gripper_name = 'finger_joint'
      gripper_mimic_joints = {'right_outer_knuckle_joint': -1,
                              'left_inner_knuckle_joint': -1,
                              'right_inner_knuckle_joint': -1,
                              'left_inner_finger_joint': 1,
                              'right_inner_finger_joint': 1}
      
      self.gripper_id = [joint.id for joint in self.joints if joint.name == gripper_name][0]
      self.mimic_child_multiplier = {joint.id: gripper_mimic_joints[joint.name] for joint in self.joints if joint.name in gripper_mimic_joints}

      for joint_id, multiplier in self.mimic_child_multiplier.items():
          constraint = p.createConstraint(self.robot_id, self.gripper_id,
                                  self.robot_id, joint_id,
                                  jointType=p.JOINT_GEAR,
                                  jointAxis=[0, 1, 0],
                                  parentFramePosition=[0, 0, 0],
                                  childFramePosition=[0, 0, 0])
          p.changeConstraint(constraint, gearRatio=-multiplier, maxForce=100, erp=1)

    def _load_ur5(self):
        """Load the UR5 URDF model."""
        if not os.path.exists(self.urdf_path):
            raise FileNotFoundError(f"URDF file not found at: {self.urdf_path}")

        robot_id = p.loadURDF(self.urdf_path, 
                              self.base_position, 
                              self.base_orientation, 
                              useFixedBase=self.use_fixed_base,
                              flags=p.URDF_ENABLE_CACHED_GRAPHICS_SHAPES)
        return robot_id

    def reset(self):
        self.reset_arm()
        self.reset_gripper()

    def reset_arm(self):
        for rest_pose, joint_id in zip(self.arm_rest_poses, self.arm_controllable_joints):
            p.resetJointState(self.robot_id, joint_id, rest_pose)
            
        for rest_pose, joint_id in zip(self.arm_rest_poses, self.arm_controllable_joints):
            self.move_joint(joint_id, rest_pose)

    def reset_gripper(self):
        self.open_gripper()

    def open_gripper(self):
        self.move_gripper_length(self.gripper_range[1])

    def close_gripper(self):
        self.move_gripper_length(self.gripper_range[0])

    def move_joint(self, joint_id, target_position):
        """Move a specific joint to a target position."""
        p.setJointMotorControl2(self.robot_id, 
                                joint_id, 
                                p.POSITION_CONTROL, 
                                targetPosition=target_position)
        
    def move_gripper_length(self, open_length):
        open_angle = 0.69432087 - 4.83034527*open_length - 4.74692119*open_length*open_length
        self.move_gripper_angle(open_angle)
        
    def move_gripper_angle(self, angle):
      p.setJointMotorControl2(self.robot_id, self.gripper_id, p.POSITION_CONTROL, targetPosition=angle,
                                force=self.joints[self.gripper_id].maxForce, maxVelocity=self.joints[self.gripper_id].maxVelocity)
  
    def _gripper_distance_formula(self, min_angle=0, max_angle=1, degree=2):
      """Gets relationship between gripper finger distance (X) and open angle (Y), creates polynomial function."""
      for _ in range(1000):
          p.stepSimulation()

      angles = np.linspace(min_angle, max_angle, 100)
      distances = []

      for angle in angles:
          self.move_gripper_angle(angle)

          for _ in range(100):
              p.stepSimulation()

          distance = self.actual_gripper_finger_distance()
          distances.append(distance)

          if distance <= 0.001:
              break

      angles = angles[:len(distances)]
      distances = np.array(distances)

      coefficients = np.polyfit(distances, angles, degree)
      polynomial = Polynomial(coefficients[::-1])
      print(f"Polynomial formula: {polynomial}")
      
      # Plot both the original data and the fitted polynomial curve
      plt.plot(distances, angles, 'o-', label='Original Data', color='blue')
      plt.plot(distances, polynomial(distances), '--', label=f'Fitted Polynomial (Degree {degree})', color='red')

      plt.xlabel('Finger Distance (m)')
      plt.ylabel('Gripper Angle (rad)')
      plt.title('Gripper Angle vs. Finger Distance')
      plt.grid(True)
      plt.legend()
      plt.show()
        
    def actual_gripper_finger_distance(self, visualize=False):
      """
      Calculate the distance between the two gripper fingers.
      """
      left_finger_joint = [joint.id for joint in self.joints if joint.name == 'left_inner_finger_joint'][0]
      right_finger_joint = [joint.id for joint in self.joints if joint.name == 'right_inner_finger_joint'][0]

      # Get the world positions of the two gripper fingers
      left_finger_pos, _ = p.getLinkState(self.robot_id, left_finger_joint)[:2]
      right_finger_pos, _ = p.getLinkState(self.robot_id, right_finger_joint)[:2]


      distance = np.linalg.norm(np.array(left_finger_pos) - np.array(right_finger_pos)) 
      distance -= 0.0345 # Offset to show distance 0 when fully closed
      
      # Visualize the distance in the simulation
      if visualize:
        if self.debug_line_handle is not None:
          p.removeUserDebugItem(self.debug_line_handle)
      
        self.debug_line_handle = p.addUserDebugLine(
            lineFromXYZ=left_finger_pos, 
            lineToXYZ=right_finger_pos, 
            lineColorRGB=[1, 0, 0],
            lineWidth=2
        )
      
      return max(distance, 0)
