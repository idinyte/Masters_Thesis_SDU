from scripts.environments.commonEnv import CommonEnv
from scripts.environments.VREnv import VREnv
from scripts.objects.softBall import SoftBall 
import pybullet as p
import random
import os

POISSON_RATIO = 0.4
RADIUS = 0.045
DENSITY = 400

BALL_TYPE_1 = "1"
BALL_TYPE_2 = "2"
BALL_TYPE_3 = "3"
BALL_TYPE_4 = "4"

BALLS_MAP = {
            BALL_TYPE_1: (5000, 9999),
            BALL_TYPE_2: (10000, 14999),
            BALL_TYPE_3: (15000, 19999),
            BALL_TYPE_4: (20000, 24999),
        }

class SortBallsEnv():
    def __init__(self, robot, camera=None, vis=False, realtime=False, debug=False, VR=False, VRCameraPos=[0,-3, 1], VRCameraRot=[0,0,0]):
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
            self.SIMULATION_STEP = 1/1000
            self.baseEnv = VREnv(self.robot, camera=self.camera, vis=self.vis, realtime=self.realtime, debug=self.debug, VR=self.VR, SIMULATION_STEP=self.SIMULATION_STEP, VRCameraPos=VRCameraPos, VRCameraRot=VRCameraRot, gripper_controller=True)
        else:
            self.SIMULATION_STEP = 1/1000
            self.baseEnv = CommonEnv(self.robot, camera=self.camera, vis=self.vis, realtime=self.realtime, debug=self.debug, VR=self.VR, SIMULATION_STEP=self.SIMULATION_STEP)

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
        pos = [random.randint(-25, 25) / 100, -0.5 + random.randint(-10, 10) / 100, 1.05]
        self.ball = self.create_random_ball(pos)
    
    def create_ball(self, youngs_modulus_min, youngs_modulus_max, name):
        youngs_modulus = random.randint(youngs_modulus_min, youngs_modulus_max)
        ball = SoftBall(youngs_modulus, POISSON_RATIO, RADIUS, DENSITY, name, self.robot.base_position)
        return ball
    
    def create_random_ball(self, pos):
        
        ball_name = random.choice(list(BALLS_MAP.keys()))
        min_youngs_modulus, max_youngs_modulus = BALLS_MAP[ball_name]
        ball_obj = self.create_ball(min_youngs_modulus, max_youngs_modulus, ball_name)
        # for debugging
        # pos = [0, -0.5, 1.045]
        ball_obj.instantiate(pos)

        return ball_obj

    def check_ball_health(self):
        if self.ball.should_self_destruct():
            self.restart_episode = True 

    def is_connected(self):
        return self.baseEnv.is_connected()

    def step_simulation(self):
        if self.VR:
            if self.baseEnv.gripper != None:
                print(self.baseEnv.gripper.get_contact_forces(self.ball.id))

        self.baseEnv.step_simulation()

    def read_debug_parameter(self):
        return self.baseEnv.read_debug_parameter()
    
    def get_state(self):
        robot_joint_angles, robot_gripper_open_length, gripper_pos, left_pad_force, right_pad_force = self.robot.get_robot_state(self.ball.id)

        return self.hard_ball_goal_pose, self.soft_ball_goal_pose, self.ball.ball_position, robot_joint_angles, robot_gripper_open_length, gripper_pos, left_pad_force, right_pad_force
    
    def main_loop(self):
        self.check_ball_health()
        self.step_simulation()
        
        return self.get_state()
        
        

    