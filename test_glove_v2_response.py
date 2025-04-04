import pybullet as p
from scripts.objects.ur5 import UR5Robot
from scripts.environments.commonEnv import CommonEnv
from scripts.environments.VREnv import VREnv
from scripts.environments.sortBallsEnv import SortBallsEnv
from scripts.ANN.sortBallsANN import SortBallsANN
from scripts.objects.gripper import Gripper, ControlType
from scripts.objects.gripper_motorsV2 import GripperMotors
import keyboard
import numpy as np
import os


# Initialize the UR5 robot
robot = UR5Robot(urdf_path=os.path.join(os.getcwd(), "assets/objects/UR5/urdf/ur5_robotiq_140_modified.urdf"), base_position=[0, 0, 0], base_orientation=[0.0, 0.0, 0.0, 1.0], use_fixed_base=True)
camera=None
vis=True
debug=False
realtime=False
VR=False
VRCameraPos = [0, 0, 1]
VRCameraRot = [0, 0, 180]

softBallPos = [0, 0, 1.045]
softBallYoungsModulus = 3000
softBallName = "1"

robot_base_position = [1, -1, 1]
env = SortBallsEnv(robot, camera, vis, realtime, debug, VR, VRCameraPos, VRCameraRot, robot_base_position = robot_base_position, softBallPos=softBallPos, softBallYoungsModulus=softBallYoungsModulus, softBallName=softBallName)

gripper_target_position = [0, 0.17, 1.045]
gripper_target_orientation = p.getQuaternionFromEuler(np.radians([90, 90, 0]))
gripper = Gripper(1/1000, exosceleton_on=True, gripper_motors=GripperMotors())
gripper.initialize_gripper_controller(gripper_target_position, gripper_target_orientation)

gripper_opening = 0.06

i = 0
while True:
  if keyboard.is_pressed("q"):
        break

  if env.debug:
    x, y, z, roll, pitch, yaw, gripper_opening_length = env.read_debug_parameter()
    #gripper_target_orientation = p.getQuaternionFromEuler([roll, pitch, yaw])
    gripper_opening = gripper_opening_length
  
  env.main_loop()

  gripper.track_pose(gripper_target_position, gripper_target_orientation)
  # gripper.move_gripper_length(gripper_opening)

  gripper.exosceleton_update(env.ball.id, ControlType.Current, verbose = False, plot = True)
  #print(i)
  if 500 < i:
    gripper.collect_force_data(env.ball.id)
  
  if env.restart_episode:
    p.disconnect(env.baseEnv.physicsClient)
    env = SortBallsEnv(robot, camera, vis, realtime, debug, VR, VRCameraPos, VRCameraRot)
  
  i += 1
  
gripper.plot_forces(gripper_opening, softBallYoungsModulus)
