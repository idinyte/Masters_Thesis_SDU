import pybullet as p
from scripts.objects.ur5 import UR5Robot
from scripts.environments.commonEnv import CommonEnv
from scripts.environments.VREnv import VREnv
from scripts.environments.sortBallsEnv import SortBallsEnv
from scripts.ANN.sortBallsANN import SortBallsANN
import numpy as np
import os
import sys
import time
import signal

def cleanup(signum, frame):
    print("Program interrupted. Cleaning up resources...")
    p.disconnect()
    
    sys.exit(0)  # Exit the program gracefully

# Register the signal handler for SIGINT (Ctrl + C)
signal.signal(signal.SIGINT, cleanup)

# Initialize the UR5 robot
robot = UR5Robot(urdf_path=os.path.join(os.getcwd(), "assets/objects/UR5/urdf/ur5_robotiq_140_modified.urdf"), base_position=[0, 0, 0], base_orientation=[0.0, 0.0, 0.0, 1.0], use_fixed_base=True)
camera=None
vis=True
debug=False
realtime=False
VR=False
VRCameraPos = [0, 0, 1]
VRCameraRot = [0, 0, 180]

robot_base_position = [1, -1, 1] if VR else [0, 0, 1]
env = SortBallsEnv(robot, camera, vis, realtime, debug, VR, VRCameraPos, VRCameraRot, robot_base_position = robot_base_position)

while env.is_connected():
  if env.debug:
    x, y, z, roll, pitch, yaw, gripper_opening_length = env.read_debug_parameter()
    robot.move_gripper_length(gripper_opening_length)
    robot.move_ee_to_target_pos([x, y ,z], [roll, pitch, yaw])
    current_position, current_orientation_euler = robot.get_ee_link_pose()
  
  hard_ball_goal_pose, soft_ball_goal_pose, ball_position, robot_joint_angles, robot_gripper_open_length, gripper_pos, left_pad_force, right_pad_force = env.main_loop()

  if env.restart_episode:
    p.disconnect(env.baseEnv.physicsClient)
    env = SortBallsEnv(robot, camera, vis, realtime, debug, VR, VRCameraPos, VRCameraRot)
