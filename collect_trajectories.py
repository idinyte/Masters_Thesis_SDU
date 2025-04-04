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
VR=True
VRCameraPos = [0, 0, 0]
VRCameraRot = [0, 0, 180]
ball_class = 3 # <-----------------------------

robot_base_position = [1, -1, 1] if VR else [0, 0, 1]
env = SortBallsEnv(robot, camera, vis, realtime, debug, VR, VRCameraPos, VRCameraRot, robot_base_position = robot_base_position, ball_idx=ball_class, fixed_gripper_ori=True)

collect_trajectories = False
trajectory = []

while env.is_connected() and not env.baseEnv.done:
  if env.debug:
    x, y, z, roll, pitch, yaw, gripper_opening_length = env.read_debug_parameter()
    robot.move_gripper_length(gripper_opening_length)
    robot.move_ee_to_target_pos([x, y ,z], [roll, pitch, yaw])
    current_position, current_orientation_euler = robot.get_ee_link_pose()
  
  state = env.main_loop(gym_state=True)
  if collect_trajectories:
    trajectory.append(state)

save_path = os.path.join(os.getcwd(), f"scripts/IRL/trajectories/ball-class-{ball_class}")
os.makedirs(save_path, exist_ok=True)
file_name = os.path.join(save_path, f"trajectory_{int(time.time())}.npy")
if collect_trajectories:
  np.save(file_name, np.array(trajectory))
  print(f"Trajectory saved to {file_name}")