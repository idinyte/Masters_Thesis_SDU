import pybullet as p
from scripts.objects.ur5 import UR5Robot
from scripts.environments.commonEnv import CommonEnv
from scripts.environments.VREnv import VREnv
from scripts.environments.sortBallsEnv import SortBallsEnv
import os
import sys

# Initialize the UR5 robot
robot = UR5Robot(urdf_path=os.path.join(os.getcwd(), "assets/objects/UR5/urdf/ur5_robotiq_140.urdf"), base_position=[0, 0, 0], base_orientation=[0, 0, 0, 1], use_fixed_base=True)
env = SortBallsEnv(robot, vis=True, debug=True, realtime=False, VR=False)

print("Start")
i = 0
while env.is_connected():
  #robot.visualize_gripper_pads_grab_pos()
  i+=1
  if env.debug:
    x, y, z, roll, pitch, yaw, gripper_opening_length = env.read_debug_parameter()
    robot.move_gripper_length(gripper_opening_length)
    robot.move_ee_to_target_pos([x, y, z], [roll, pitch, yaw])
    #print(f"{robot.actual_gripper_finger_distance(visualize=True):.4f} m")

  hard_ball_goal_pose, soft_ball_goal_pose, ball_position, robot_joint_angles, robot_gripper_open_length, gripper_pos, left_pad_force, right_pad_force, link_ids = env.main_loop()
  
  if i == 400:
    print(f"hard_ball_goal_pose {hard_ball_goal_pose}, soft_ball_goal_pose {soft_ball_goal_pose}, ball_position {ball_position}, robot_joint_angles {robot_joint_angles}, robot_gripper_open_length {robot_gripper_open_length}, gripper_pos {gripper_pos}, left_pad_force {left_pad_force}, right_pad_force {right_pad_force}, link_ids {link_ids}")
    i=0

  if env.restart_episode:
    p.disconnect(env.baseEnv.physicsClient)
    env = SortBallsEnv(robot, vis=True, debug=False, realtime=False, VR=False)