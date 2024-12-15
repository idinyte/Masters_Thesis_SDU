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
debug=True
realtime=False
VR=False
VRCameraPos = [0, 0, 1]
VRCameraRot = [0, 0, 180]

robot_base_position = [1, -1, 1] if VR else [0, 0, 1]
env = SortBallsEnv(robot, camera, vis, realtime, debug, VR, VRCameraPos, VRCameraRot, robot_base_position = robot_base_position)

robot.print_joint_info()

def _get_reward(state, env):
    ball_position_world_coordinates = state[13:16] + state[7:10]
    
    # reward for gripper being close to the ball
    reward = 0.5 - np.linalg.norm(state[13:16])

    # reward for holding ball
    left_pad_force, right_pad_force = state[11], state[12]
    gripper_opening_length = state[10]
    if gripper_opening_length < 0.06:
      if left_pad_force > 0 and right_pad_force > 0:
        reward += 0.5
      else:
        reward -= 0.5
    
    # reward for ball being close to goal
    reward += 0.5 - np.linalg.norm(np.array(env.get_corresponding_ball_box()) - np.array(ball_position_world_coordinates))

    # reward for crashing the environment (ball being too far or exploding)
    if env.restart_episode:
      reward -= 10
      
    # reward for being in collision with something that is not ball
    if env.robot.non_ball_contact:
      reward -= 1

    # reward for placing ball in correct box
    if env.ball.is_in_box(env.get_corresponding_ball_box_id()):
      reward += 500

    return reward

while env.is_connected():
  if env.debug:
    x, y, z, roll, pitch, yaw, gripper_opening_length = env.read_debug_parameter()
    robot.move_gripper_length(gripper_opening_length)
    robot.move_ee_to_target_pos([x, y ,z], [roll, pitch, yaw])
    current_position, current_orientation_euler = robot.get_ee_link_pose()
    print(_get_reward(env.state_to_gym_state(env.get_state()), env))
  
  env.main_loop()

  if env.restart_episode:
    p.disconnect(env.baseEnv.physicsClient)
    env = SortBallsEnv(robot, camera, vis, realtime, debug, VR, VRCameraPos, VRCameraRot)
