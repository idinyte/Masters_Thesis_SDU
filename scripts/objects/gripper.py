import pybullet as p
import os
import numpy as np
from collections import namedtuple
import matplotlib.pyplot as plt
from scripts.objects.gripper_motors import GripperMotors

class Gripper:
  def __init__(self, SIMULATION_STEP, exosceleton_on = False):
    self.LEFT_PAD_GRIPPER_INDEX = 3
    self.RIGHT_PAD_GRIPPER_INDEX = 8
    self.gripper_range = [0, 0.127]
    self.SIMULATION_STEP = SIMULATION_STEP
    self.left_forces = []
    self.right_forces = []
    self.exosceleton_on = exosceleton_on
    if exosceleton_on:
      self.gripper_motors = GripperMotors()

  def initialize_gripper_controller(self, pos, orn):
        self.id = p.loadURDF(os.path.join(os.getcwd(), "assets/objects/UR5/urdf/robotiq_140_modified.urdf"), pos, orn)
        self._parse_joint_info(print_info=False)
        self._gripper_contraints()
        self.open_gripper()
        
  def _parse_joint_info(self, print_info=False):
        """Populate self.joints"""
        numJoints = p.getNumJoints(self.id)
        jointInfo = namedtuple('jointInfo', 
            ['id','name','type','damping','friction','lowerLimit','upperLimit','maxForce','maxVelocity','controllable'])
        self.joints = []
        self.controllable_joints = []
        for i in range(numJoints):
            info = p.getJointInfo(self.id, i)
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
                p.setJointMotorControl2(self.id, jointID, p.VELOCITY_CONTROL, targetVelocity=0, force=0)
            info = jointInfo(jointID,jointName,jointType,jointDamping,jointFriction,jointLowerLimit,
                            jointUpperLimit,jointMaxForce,jointMaxVelocity,controllable)
            self.joints.append(info)
            
            if print_info:
              print(info)
              
  def _gripper_contraints(self):
      """Move children gripper joints according to parent joint"""
      gripper_name = 'finger_joint'
      gripper_mimic_joints = {'left_outer_knuckle_joint': -1,
                              'right_outer_knuckle_joint': -1,
                              'left_inner_knuckle_joint': -1,
                              'right_inner_knuckle_joint': -1,
                              'left_inner_finger_joint': 1,
                              'right_inner_finger_joint': 1}
      
      self.gripper_id = [joint.id for joint in self.joints if joint.name == gripper_name][0]
      self.mimic_child_multiplier = {joint.id: gripper_mimic_joints[joint.name] for joint in self.joints if joint.name in gripper_mimic_joints}

      for joint_id, multiplier in self.mimic_child_multiplier.items():
          constraint = p.createConstraint(self.id, self.gripper_id,
                                  self.id, joint_id,
                                  jointType=p.JOINT_GEAR,
                                  jointAxis=[0, 1, 0],
                                  parentFramePosition=[0, 0, 0],
                                  childFramePosition=[0, 0, 0])
          p.changeConstraint(constraint, gearRatio=-multiplier, maxForce=100, erp=1)
    
  def get_contact_forces(self, object_id):
        contact_points = p.getContactPoints(bodyA=self.id, bodyB=object_id)

        left_pad_force = 0
        right_pad_force = 0

        for contact in contact_points:
            link_id = contact[3]
            if link_id == self.LEFT_PAD_GRIPPER_INDEX:
                left_pad_force += contact[9]
            elif link_id == self.RIGHT_PAD_GRIPPER_INDEX:
                right_pad_force += contact[9]
        
        return left_pad_force, right_pad_force

  def track_pose(self, target_position, target_orientation):
    # Calculate velocity based on error between gripper and desired positions
    current_position, current_orientation = p.getBasePositionAndOrientation(self.id)
    position_gain = 1 / self.SIMULATION_STEP
    orientation_gain = 2 / self.SIMULATION_STEP
    max_linear_velocity = 50

    position_error = np.array(target_position) - np.array(current_position)
    linear_velocity = position_gain * position_error
    linear_velocity = np.clip(linear_velocity, -max_linear_velocity, max_linear_velocity)
    
    orientation_error = p.getDifferenceQuaternion(current_orientation, target_orientation)
    angular_velocity = orientation_gain * np.array(orientation_error[:3])
    
    # Apply the calculated velocities to the gripper
    p.resetBaseVelocity(self.id, linearVelocity=linear_velocity.tolist(), angularVelocity=angular_velocity.tolist())
    
  def exosceleton_update(self, verbose = False):
    if self.exosceleton_on:
      self.gripper_motors.update_positions(verbose)
    
  def move_gripper_length(self, open_length):
    open_angle = self.gripper_distance_to_angle(open_length)
    self.move_gripper_angle(open_angle)
        
  def gripper_distance_to_angle(self, open_length):
    return 0.69432087 - 4.83034527*open_length - 4.74692119*open_length*open_length
    
  def gripper_angle_to_distance(self, angle):
    a = 4.74692119
    b = 4.83034527
    c = angle - 0.69432087

    discriminant = b**2 - 4*a*c
    if discriminant >= 0:
        return (-b + np.sqrt(discriminant)) / (2*a)
    
    return 0
  
  def move_gripper_angle(self, angle):
    p.setJointMotorControl2(self.id, self.gripper_id, p.POSITION_CONTROL, targetPosition=angle)
    
  def open_gripper(self):
    self.move_gripper_length(self.gripper_range[1])

  def close_gripper(self):
    self.move_gripper_length(self.gripper_range[0])
    
  def collect_force_data(self, object_id):
    left_pad_force, right_pad_force = self.get_contact_forces(object_id)
    
    self.left_forces.append(left_pad_force)
    self.right_forces.append(right_pad_force)
  
  def plot_forces(self, gripper_opening = None, ball_youngs_modulus = None):
    if not self.left_forces or not self.right_forces:
        print("No force data to plot.")
        return
    
    if len(self.left_forces) > 10000:
      self.left_forces = self.left_forces[-10000:]
      self.right_forces = self.right_forces[-10000:]

    time_steps = range(len(self.left_forces))
    
    plt.figure(figsize=(10, 6))
    plt.plot(time_steps, self.left_forces, label="Left Pad Force", color="blue")
    plt.plot(time_steps, self.right_forces, label="Right Pad Force", color="red")
    
    plt.xlabel("Time Steps")
    plt.ylabel("Force (N)")
    title = "Forces Over Time"
    if gripper_opening and ball_youngs_modulus:
      title = f"Forces Over Time. Gripper opening {gripper_opening} m. Ball's Youngs modulus {ball_youngs_modulus} Pa"
    plt.title("")
    plt.legend()
    plt.grid(True)
    
    plt.show()
    
  