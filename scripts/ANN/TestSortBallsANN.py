import pybullet as p
import sys
import os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../..')))
from scripts.objects.ur5 import UR5Robot
from scripts.environments.sortBallsEnv import SortBallsEnv
from scripts.ANN.sortBallsANN import SortBallsANN

class TestSortBallsANN():
  def __init__(self, ball_class_name = None, record_trajectory = False, use_camera = True):
    self.ball_class_name = ball_class_name
    self.record_trajectory = record_trajectory
    self.use_camera = use_camera

  def reset(self):
    robot = UR5Robot(urdf_path=os.path.join(os.getcwd(), "assets/objects/UR5/urdf/ur5_robotiq_140_modified.urdf"), base_position=[0, 0, 0], base_orientation=[0.0, 0.0, 0.0, 1.0], use_fixed_base=True)
    camera=None
    vis=True
    debug=False
    realtime=False
    VR=False
    VRCameraPos = [0,-1, 0]
    VRCameraRot = [0,0,0]
    self.env = SortBallsEnv(robot, camera, vis, realtime, debug, VR, VRCameraPos, VRCameraRot, ball_class_name = self.ball_class_name)
    self.algorithm = SortBallsANN(self.env, record_trajectory=self.record_trajectory)
    
  def run_once(self):
    self.reset()
    results_file = os.path.join(os.getcwd(), "scripts/ANN/results/results.txt")
    if not os.path.exists(results_file):
      with open(results_file, "w") as file:
        pass

    success = self.algorithm.start(results_file, use_camera=self.use_camera)
    p.disconnect(self.env.baseEnv.physicsClient)
    return success