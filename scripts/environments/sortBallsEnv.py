from scripts.environments.commonEnv import CommonEnv
from scripts.environments.VREnv import VREnv
from scripts.objects.softBall import SoftBall 
import pybullet as p
import math
import os

class SortBallsEnv():
    def __init__(self, robot, camera=None, vis=False, realtime=False, debug=False, VR=False):
        self.robot = robot
        self.robot.base_position = [0, 0, 1]
        self.robot.base_orientation=[0, 0, 0, 1]

        self.vis = vis
        self.realtime = realtime
        self.debug=debug
        self.camera = camera
        self.VR = VR
        self.baseEnv = None

        if self.VR:
            self.baseEnv = VREnv(self.robot, camera=self.camera, vis=self.vis, realtime=self.realtime, debug=self.debug, VR=self.VR)
        else:
            self.baseEnv = CommonEnv(self.robot, camera=self.camera, vis=self.vis, realtime=self.realtime, debug=self.debug, VR=self.VR)

        self.balls = []
        self.init_objects()

    def init_objects(self):
        # Table
        self.table_id = p.loadURDF("table/table.urdf", basePosition=[0, -0.5, 0], baseOrientation=p.getQuaternionFromEuler([0, 0, 0]), globalScaling=1.6, useFixedBase=True)

        # Boxes
        self.box_robot_stiff_id = p.loadURDF(os.path.join(os.getcwd(), "assets/objects/box/urdf/box_dark.urdf"), basePosition=[0.75, -0.5, 1], baseOrientation=p.getQuaternionFromEuler([0, 0, 0]), useFixedBase=True)
        self.box_robot_not_stiff_id = p.loadURDF(os.path.join(os.getcwd(), "assets/objects/box/urdf/box_light.urdf"), basePosition=[-0.75, -0.5, 1], baseOrientation=p.getQuaternionFromEuler([0, 0, 0]), useFixedBase=True)

        # self.box_human_stiff_id = p.loadURDF(os.path.join(os.getcwd(), "assets/objects/box/urdf/box_dark.urdf"), basePosition=[0.75, -1.5, 1], baseOrientation=p.getQuaternionFromEuler([0, 0, 0]), useFixedBase=True)
        # self.box_human_not_stiff_id = p.loadURDF(os.path.join(os.getcwd(), "assets/objects/box/urdf/box_light.urdf"), basePosition=[-0.75, -1.5, 1], baseOrientation=p.getQuaternionFromEuler([0, 0, 0]), useFixedBase=True)

    def main_loop(self):
        self.create_balls()

        self.step_simulation()

    def create_ball(self, sitffness = 600, base_position = [0, -1, 20], radius = 0.05):
        ball = SoftBall(sitffness, base_position, radius)
        ball.instantiate()

        return ball

    def create_balls(self):
        return
        if len(self.balls) < 2:
            ball = self.create_ball()
            self.balls.append(ball)

    def is_connected(self):
        return self.baseEnv.is_connected()

    def step_simulation(self):
        self.baseEnv.step_simulation()

    def read_debug_parameter(self):
        self.baseEnv.read_debug_parameter()

    