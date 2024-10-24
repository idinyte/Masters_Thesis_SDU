from scripts.environments.commonEnv import CommonEnv
from scripts.environments.VREnv import VREnv
from scripts.objects.softBall import SoftBall 
import pybullet as p
import random
import os

SOFT_BALL_NAME = "soft"
HARD_BALL_NAME = "hard"
POISSON_RATIO = 0.4
RADIUS = 0.045
DENSITY = 300
SIMULATION_STEP = 1/1000

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
        self.restart_episode = False

        if self.VR:
            self.baseEnv = VREnv(self.robot, camera=self.camera, vis=self.vis, realtime=self.realtime, debug=self.debug, VR=self.VR, SIMULATION_STEP=SIMULATION_STEP)
        else:
            self.baseEnv = CommonEnv(self.robot, camera=self.camera, vis=self.vis, realtime=self.realtime, debug=self.debug, VR=self.VR, SIMULATION_STEP=SIMULATION_STEP)

        self.init_objects()
        
    def print_all_objects(self):
        num_bodies = p.getNumBodies()
        print(f"Total number of objects: {num_bodies}")
        
        for i in range(num_bodies):
            body_id = p.getBodyUniqueId(i)
            body_info = p.getBodyInfo(body_id)
            body_name = body_info[1].decode('utf-8')
            print(f"Object {i}: ID = {body_id}, Name = {body_name}")

    def init_objects(self):
        # Table
        self.table_id = p.loadURDF("table/table.urdf", basePosition=[0, -0.5, 0], baseOrientation=p.getQuaternionFromEuler([0, 0, 0]), globalScaling=1.6, useFixedBase=True)

        # Boxes
        box_hard_base_pos = [0.75, -0.5, 1]
        self.box_robot_stiff_id = p.loadURDF(os.path.join(os.getcwd(), "assets/objects/box/urdf/box_dark.urdf"), basePosition=[0.75, -0.5, 1], baseOrientation=p.getQuaternionFromEuler([0, 0, 0]), useFixedBase=True)
        self.hard_ball_goal_pose = box_hard_base_pos
        self.hard_ball_goal_pose[2] += RADIUS

        box_soft_base_pos = [-0.75, -0.5, 1]
        self.box_robot_not_stiff_id = p.loadURDF(os.path.join(os.getcwd(), "assets/objects/box/urdf/box_light.urdf"), basePosition=box_soft_base_pos, baseOrientation=p.getQuaternionFromEuler([0, 0, 0]), useFixedBase=True)
        self.soft_ball_goal_pose = box_soft_base_pos
        self.soft_ball_goal_pose[2] += RADIUS

        # Soft ball
        self.ball = self.create_random_ball()
    
    def create_ball(self, youngs_modulus_min, youngs_modulus_max, name):
        youngs_modulus = random.randint(youngs_modulus_min, youngs_modulus_max)
        ball = SoftBall(youngs_modulus, POISSON_RATIO, RADIUS, DENSITY, name, self.robot.base_position)
        return ball
    
    def create_random_ball(self):
        balls_map = {
            SOFT_BALL_NAME: lambda: self.create_ball(37500, 42500, SOFT_BALL_NAME),
            HARD_BALL_NAME: lambda: self.create_ball(47500, 52500, HARD_BALL_NAME),
        }
        ball_name = random.choice(list(balls_map.keys()))
        ball_obj = balls_map[ball_name]()
        
        # instantiate on the table
        pos = [random.randint(-25, 25) / 100, -0.5 + random.randint(-10, 10) / 100, 1.1]
        
        # for debugging
        pos = [0, -0.5, 1.045]
        ball_obj.instantiate(pos)

        return ball_obj

    def check_ball_health(self):
        if self.ball.should_self_destruct():
            self.restart_episode = True 

    def is_connected(self):
        return self.baseEnv.is_connected()

    def step_simulation(self):
        self.baseEnv.step_simulation()

    def read_debug_parameter(self):
        return self.baseEnv.read_debug_parameter()
    
    def get_state(self):
        robot_joint_angles, robot_gripper_open_length, gripper_pos, left_pad_force, right_pad_force, link_ids = self.robot.get_robot_state(self.ball.id)

        return self.hard_ball_goal_pose, self.soft_ball_goal_pose, self.ball.ball_position, robot_joint_angles, robot_gripper_open_length, gripper_pos, left_pad_force, right_pad_force, link_ids
    
    def main_loop(self):
        self.check_ball_health()
        self.step_simulation()
        
        return self.get_state()
        
        

    