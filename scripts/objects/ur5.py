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
        
        # for debugging
        self.arm_rest_poses = [-1.789883877596033, -1.430909590958482, 1.954781054223412, -2.095465266615244, -1.5537133877330433, -1.5151807365373613]
        
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
    
    def print_joint_info(self):
        for info in self.joints:
            print(info)
            
    def visualize_joints(self):
        for joint in self.joints:
            # Get the world position of the joint's parent link
            joint_state = p.getLinkState(self.robot_id, joint.id)
            joint_position = joint_state[0]  # position in [x, y, z]
            
            # Add a small sphere at the joint's position for visualization
            p.addUserDebugText(f"{joint.name} (ID: {joint.id})", joint_position, textColorRGB=[1, 0, 0], textSize=1.2)
            p.addUserDebugLine(joint_position, [joint_position[0], joint_position[1], joint_position[2] + 0.01], 
                            lineColorRGB=[0, 1, 0], lineWidth=2.0)

            # Optional: print joint information for reference
            print(f"Joint {joint.id}: {joint.name} at {joint_position}")
            
    def visualize_gripper_pads_grab_pos(self):
        p.removeAllUserDebugItems()
        
        left_pad_pos = np.array(p.getLinkState(self.robot_id, 12)[0])
        right_pad_pos = np.array(p.getLinkState(self.robot_id, 17)[0])
        
        middle_point = (left_pad_pos + right_pad_pos) / 2
        
        p.addUserDebugLine(left_pad_pos, right_pad_pos, lineColorRGB=[0, 1, 0], lineWidth=2.0)
        p.addUserDebugLine([middle_point[0], middle_point[1], middle_point[2] - 0.05], 
                        [middle_point[0], middle_point[1], middle_point[2] + 0.05], 
                        lineColorRGB=[1, 0, 0], lineWidth=2.0)
        
    def get_gripper_middle_pad_pos(self):
        left_pad_pos = np.array(p.getLinkState(self.robot_id, 12)[0])
        right_pad_pos = np.array(p.getLinkState(self.robot_id, 17)[0])
        
        middle_point = (left_pad_pos + right_pad_pos) / 2
        return middle_point.tolist()
    
    def get_gripper_contact_forces(self, ball_id):
        left_pad_id = 12
        right_pad_id = 17
        
        contact_points = p.getContactPoints(bodyA=self.robot_id, bodyB=ball_id)
        # print(f"contact_points {contact_points}")
        
        left_pad_force = 0
        right_pad_force = 0
        
        # contact_points = p.getContactPoints(bodyA=self.robot_id, bodyB=ball_id, linkIndexA=1)
        # compressive_normal_force = sum(contact[9] for contact in contact_points)
        link_ids = []
        for contact in contact_points:
            link_id = contact[3]
            link_ids.append((link_id, contact[9], contact[10], contact[12]))
            if link_id == left_pad_id:
                left_pad_force += contact[9]
            elif link_id == right_pad_id:
                right_pad_force += contact[9]
        
        return left_pad_force, right_pad_force, link_ids

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
  
    def move_ee_to_target_pos(self, targetPos, targetRot = None):
        ik_solution = self.calculate_IK(targetPos, targetRot)
        self.move_joints_arr([1, 2, 3, 4, 5, 6] , ik_solution[:6])
  
    def calculate_IK(self, targetPos, targetRot = None):
        ee_id = 7
        if targetRot == None:
            return p.calculateInverseKinematics(self.robot_id, ee_id, targetPos)
        
        targetRotQuart = p.getQuaternionFromEuler(targetRot)
        return p.calculateInverseKinematics(self.robot_id, ee_id, targetPos, targetRotQuart)
        
    def move_joints_arr(self, indices, poses, targetRot = None):
        p.setJointMotorControlArray(
            bodyUniqueId=self.robot_id,
            jointIndices=indices,
            controlMode=p.POSITION_CONTROL,
            targetPositions=poses
        )
        
    def get_robot_state(self, ballId):
        joint_states = p.getJointStates(self.robot_id, self.arm_controllable_joints)
        
        joint_angles = [state[0] for state in joint_states]
        

        gripper_joint_angle= p.getJointState(self.robot_id, self.gripper_id)[0]
        gripper_open_length = self.gripper_angle_to_distance(gripper_joint_angle)
        
        gripper_pos = self.get_gripper_middle_pad_pos()
        left_pad_force, right_pad_force, link_ids = self.get_gripper_contact_forces(ballId)
        
        return joint_angles, gripper_open_length, gripper_pos, left_pad_force, right_pad_force, link_ids
