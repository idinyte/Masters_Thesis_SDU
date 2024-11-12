import pybullet as p
import numpy as np
from scripts.ANN.pointcloud import PointCloud
from scripts.ANN.trajectory import Trajectory
from scripts.ANN.ANN import ANN
import copy
import torch
import time
from enum import Enum

class STATES(Enum):
    SUCCESS = 0
    FAILED_TO_FIND_BALL = 1
    FAILED_TO_GRAB_BALL = 2
    FAILED_TO_CLASSIFY_BALL = 3
    FAILED_TO_PLACE_BALL = 4
    
class SortBallsANN:
    def __init__(self, env):
        self.env = env
        self.robot = self.env.robot
        self.scene_point_cloud = None
        self.point_cloud_obj = PointCloud()
        self.trajectory = Trajectory(self.env.SIMULATION_STEP, self.env.robot, self.env.main_loop)
        self._load_neural_network()
    
    def _load_neural_network(self):
        input_dim = 8
        output_dim = 4
        self.model = ANN(input_dim=input_dim, output_dim=output_dim)
        weights_path = 'scripts/ANN/ann_weights_architecture_8_32_4_epochs_5000_acc_996.pth'
        self.model.load_state_dict(torch.load(weights_path))
        self.model.eval()

    def get_point_cloud_and_find_ball(self):
        # Obtain point cloud from camera
        self.scene_point_cloud = self.point_cloud_obj.get_point_cloud()
        
        # Crop to for working area only
        aabb_min = np.array(self.env.ball_pos_aabb_min) - 1.1 * self.env.ball_radius
        aabb_max = np.array(self.env.ball_pos_aabb_max) + 1.1 * self.env.ball_radius
        
        #self.point_cloud_obj.export_to_pcd(self.scene_point_cloud, "before_crop.pcd")
        self.scene_point_cloud = self.point_cloud_obj.crop_point_cloud_aabb(self.scene_point_cloud, aabb_min, aabb_max)
        #self.point_cloud_obj.export_to_pcd(self.scene_point_cloud, "after_crop.pcd")
        
        # Get pointcloud of object that we try to find
        self.ball_point_cloud = self.point_cloud_obj.get_point_cloud_from_object(self.env.ball.obj, num_points=5000)
        
        # Find ball position from ransac
        pose = self.point_cloud_obj.global_alignment(self.scene_point_cloud, self.ball_point_cloud)
        self.ball_position = np.array(pose)[:3, 3].tolist()
        
    def restart_episode(self):
        self.scene_point_cloud = None
        self.ball_position = None

    def grab_ball(self):
        gripper_to_eef_z_offset = 0.19
        gripper_grab_ball_length = 0.06
        points, orientations, gripper, durations = [], [], [], []
        
        # Current point
        p1, o1 = self.robot.get_ee_link_pose()
        points.append(p1)
        orientations.append(o1)
        gripper.append(self.robot.get_gripper_open_length())
        
        # Point above ball
        durations.append(3)
        p2 = copy.copy(self.ball_position)
        p2[2] += gripper_to_eef_z_offset + 0.4
        points.append(p2)
        orientations.append(p.getQuaternionFromEuler([0, np.pi/2, np.pi/2]))
        gripper.append(self.robot.gripper_range[1])
        
        # Point at ball
        durations.append(1)
        p3 = copy.copy(self.ball_position)
        p3[2] += gripper_to_eef_z_offset
        points.append(p3)
        orientations.append(p.getQuaternionFromEuler([0, np.pi/2, np.pi/2]))
        gripper.append(self.robot.gripper_range[1])
        
        # Close gripper at ball
        durations.append(2)
        p4 = copy.copy(self.ball_position)
        p4[2] += gripper_to_eef_z_offset
        points.append(p4)
        orientations.append(p.getQuaternionFromEuler([0, np.pi/2, np.pi/2]))
        gripper.append(gripper_grab_ball_length)
        
        # Lift ball
        durations.append(2)
        p5 = copy.copy(self.ball_position)
        p5[2] += gripper_to_eef_z_offset + 0.4
        points.append(p5)
        orientations.append(p.getQuaternionFromEuler([0, np.pi/2, np.pi/2]))
        gripper.append(gripper_grab_ball_length)
        
        trajectory = self.trajectory.generate_trajectory(points, orientations, gripper, durations)
        self.trajectory.follow_trajectory(trajectory)
            
        return not self.env.restart_episode
    
    def place_ball(self):
        gripper_grab_ball_length = 0.06
        points, orientations, gripper, durations = [], [], [], []
        
        # Current point
        p1, o1 = self.robot.get_ee_link_pose()
        points.append(p1)
        orientations.append(o1)
        gripper.append(self.robot.get_gripper_open_length())
        
        # Rise
        durations.append(1)
        p2 = copy.copy(self.ball_position)
        p2[2] = 1.8
        points.append(p2)
        orientations.append(p.getQuaternionFromEuler([0, np.pi/2, np.pi/2]))
        gripper.append(gripper_grab_ball_length)
        
        # Point above correct box
        durations.append(3)
        p3 = self.env.get_corresponding_ball_box()
        p3[2] = 1.8
        points.append(p3)
        orientations.append(p.getQuaternionFromEuler([0, np.pi/2, np.pi/2]))
        gripper.append(gripper_grab_ball_length)
        
        # Move down
        durations.append(1)
        p4 = self.env.get_corresponding_ball_box()
        p4[2] = 1.4
        points.append(p4)
        orientations.append(p.getQuaternionFromEuler([0, np.pi/2, np.pi/2]))
        gripper.append(gripper_grab_ball_length)
        
        # Release ball
        durations.append(1)
        p5 = self.env.get_corresponding_ball_box()
        p5[2] = 1.4
        points.append(p5)
        orientations.append(p.getQuaternionFromEuler([0, np.pi/2, np.pi/2]))
        gripper.append(self.robot.gripper_range[1])
        
        trajectory = self.trajectory.generate_trajectory(points, orientations, gripper, durations)
        self.trajectory.follow_trajectory(trajectory)
            
        return not self.env.restart_episode
    
    def move_gripper_and_get_forces(self, final_gripper_distance, transition_duration = 1):
        p1, o1 = self.robot.get_ee_link_pose()
        p2, o2 = p1, o1
        g1 = self.robot.get_gripper_open_length()
        g2 = final_gripper_distance
        trajectory = self.trajectory.generate_trajectory([p1, p2], [o1, o2], [g1, g2], [transition_duration])
        self.trajectory.follow_trajectory(trajectory)
        
        gripper_distances = []
        forces = []
        smooth = 5
        for _ in range(smooth):
            gripper_distance = self.robot.get_gripper_open_length()
            left_pad_force, right_pad_force = self.robot.get_gripper_contact_forces(self.env.ball.id)
            gripper_distances.append(gripper_distance)
            forces.append((left_pad_force + right_pad_force) / 2)
            p.stepSimulation()
        
        return sum(gripper_distances)/smooth, sum(forces)/smooth

    def classify_ball(self):
        distances_forces = []
        for distance in [0.05, 0.04, 0.07, 0.06]:
            d, f = self.move_gripper_and_get_forces(distance, transition_duration=0.5)
            distances_forces.append(d)
            distances_forces.append(f)
        
        distances_forces_tensor = torch.tensor(distances_forces, dtype=torch.float32).unsqueeze(0)
        # Run the forward pass
        with torch.no_grad():
            output = self.model(distances_forces_tensor)
            _, predicted_label = torch.max(output, 1)

        return int(predicted_label[0])
        
    def append_result(self, results_file, result):
        with open(results_file, 'a+') as file:
            file.write(result)
            
    def start(self, results_file):
        self.env.simulation_time = 0
        ball_find_end_time, ball_grab_start_time, ball_grab_end_time, ball_classify_start_time, ball_classify_end_time, ball_place_start_time, ball_place_end_time = 0, 0, 0, 0, 0, 0, 0
        
        ball_find_start_time = time.time()
        if self.scene_point_cloud == None:
            # Move the robot arm from camera view so point cloud can be taken
            self.env.robot.move_ee_to_target_pos([0, -0.3, 2.5], [0, np.pi/2, np.pi/2])
            for _ in range(300):
                self.env.main_loop()
                if self.env.restart_episode:
                    return False

            self.get_point_cloud_and_find_ball()
        ball_find_end_time = time.time()
        
        real_position, orientation = p.getBasePositionAndOrientation(self.env.ball.id)
        #self.ball_position = list(real_position)

        ball_position_error = np.linalg.norm(np.array(real_position) - np.array(self.ball_position))
        print(f"real ball position {real_position} found ball position {self.ball_position} error {ball_position_error}")

        if ball_position_error > 0.02:
            print("Failed to detect acurate ball position")
            result = f"{STATES.FAILED_TO_FIND_BALL.value} {ball_find_start_time} {ball_find_end_time} {ball_grab_start_time} {ball_grab_end_time} {ball_classify_start_time} {ball_classify_end_time} {ball_place_start_time} {ball_place_end_time}"
            self.append_result(results_file, result)
            return False

        ball_grab_start_time = self.env.simulation_time
        rc = self.grab_ball()
        ball_grab_end_time = self.env.simulation_time

        if rc == False or not self.robot.is_grabbing_ball(self.env.ball.id):
            print("Failed to grab ball")
            result = f"{STATES.FAILED_TO_GRAB_BALL.value} {ball_find_start_time} {ball_find_end_time} {ball_grab_start_time} {ball_grab_end_time} {ball_classify_start_time} {ball_classify_end_time} {ball_place_start_time} {ball_place_end_time}"
            self.append_result(results_file, result)
            return False

        ball_classify_start_time = self.env.simulation_time
        ball_class = self.classify_ball()
        ball_classify_end_time = self.env.simulation_time

        if str(ball_class + 1) != self.env.ball.name:
            print(f"Failed to classify ball predicted ball name {ball_class + 1} actual {self.env.ball.name}")
            result = f"{STATES.FAILED_TO_CLASSIFY_BALL.value} {ball_find_start_time} {ball_find_end_time} {ball_grab_start_time} {ball_grab_end_time} {ball_classify_start_time} {ball_classify_end_time} {ball_place_start_time} {ball_place_end_time}"
            self.append_result(results_file, result)
            return False

        ball_place_start_time = self.env.simulation_time
        rc = self.place_ball()
        ball_place_end_time = self.env.simulation_time
        if rc == False:
            print("Failed place ball")
            result = f"{STATES.FAILED_TO_PLACE_BALL.value} {ball_find_start_time} {ball_find_end_time} {ball_grab_start_time} {ball_grab_end_time} {ball_classify_start_time} {ball_classify_end_time} {ball_place_start_time} {ball_place_end_time}"
            self.append_result(results_file, result)
            return False

        for _ in range(200):
            self.env.main_loop()

        if not self.env.ball.is_in_box(self.env.get_corresponding_ball_box_id()):
            print("Ball is not in correct box")
            result = f"{STATES.FAILED_TO_PLACE_BALL.value} {ball_find_start_time} {ball_find_end_time} {ball_grab_start_time} {ball_grab_end_time} {ball_classify_start_time} {ball_classify_end_time} {ball_place_start_time} {ball_place_end_time}"
            self.append_result(results_file, result)
            return False

        result = f"{STATES.SUCCESS.value} {ball_find_start_time} {ball_find_end_time} {ball_grab_start_time} {ball_grab_end_time} {ball_classify_start_time} {ball_classify_end_time} {ball_place_start_time} {ball_place_end_time}"
        self.append_result(results_file, result)
        return True