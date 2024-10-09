import pybullet as p
from scripts.objects.ur5 import UR5Robot
from scripts.environments.commonEnv import CommonEnv
from scripts.environments.VREnv import VREnv
from scripts.environments.sortBallsEnv import SortBallsEnv
import os

# Initialize the UR5 robot
robot = UR5Robot(urdf_path=os.path.join(os.getcwd(), "assets/objects/UR5/urdf/ur5_robotiq_140.urdf"), base_position=[0, 0, 0], base_orientation=[0, 0, 0, 1], use_fixed_base=True)
env = SortBallsEnv(robot, vis=True, debug=False, realtime=False, VR=False)

print("Start")
while env.is_connected():
  if env.debug:
    x, y, z, roll, pitch, yaw, gripper_opening_length = env.read_debug_parameter()
    robot.move_gripper_length(gripper_opening_length)
    #print(f"{robot.actual_gripper_finger_distance(visualize=True):.4f} m")

  env.main_loop()