import sys
import os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../..')))

import pybullet as p
from scripts.objects.ur5 import UR5Robot
from scripts.ANN.trajectory import Trajectory
import numpy as np
import os
import pybullet_data


SIMULATION_STEP = 1/1000

class RobotPress():
    def __init__(self):
        self.robot = UR5Robot(urdf_path=os.path.join(os.getcwd(), "assets/objects/UR5/urdf/ur5_robotiq_140_modified.urdf"), base_position=[0, 0, 0], base_orientation=[0.0, 0.0, 0.0, 1.0], use_fixed_base=True)
        self.trajectory = Trajectory(SIMULATION_STEP, self.robot, p.stepSimulation)

    def load_soft_ball(self, youngs_modulus, pos):
        density = 400
        ball_radius_m = 0.045
        volume = 4*np.pi*ball_radius_m**3/3
        mass = density*volume
        poson_ratio = 0.4

        neo_mu = youngs_modulus / (2 * (1 + poson_ratio))
        neo_lambda = youngs_modulus * poson_ratio / ((1 + poson_ratio)*(1 - 2 * poson_ratio))
        ballId = p.loadSoftBody(os.path.join(os.getcwd(), "assets/objects/softBall/ball_regular.obj"), 
                                simFileName=os.path.join(os.getcwd(), "assets/objects/softBall/ball_regular.vtk"), 
                                basePosition=pos, 
                                mass=mass, 
                                useNeoHookean=1, 
                                NeoHookeanMu=neo_mu, 
                                NeoHookeanLambda=neo_lambda,
                                NeoHookeanDamping=0.001,
                                useSelfCollision=1,
                                repulsionStiffness=800,
                                frictionCoeff=1, 
                                collisionMargin=0.0001)
        
        for _ in range(20):
            p.stepSimulation()

        return ballId

    def set_simulation_params(self):
        physic_client = p.connect(p.GUI)
        p.setTimeStep(SIMULATION_STEP)
        p.setRealTimeSimulation(0)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.resetSimulation(p.RESET_USE_DEFORMABLE_WORLD)
        #p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 0)
        p.setGravity(0, 0, -10)
        p.resetDebugVisualizerCamera(cameraDistance=2, 
                                        cameraYaw=0.0, 
                                        cameraPitch=-5.0, 
                                        cameraTargetPosition=[0, 0, 0.5])
        p.setPhysicsEngineParameter(sparseSdfVoxelSize=0.2)
        p.setPhysicsEngineParameter(numSubSteps=1)
        return physic_client
      
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
            left_pad_force, right_pad_force = self.robot.get_gripper_contact_forces(self.ballId)
            gripper_distances.append(gripper_distance)
            forces.append((left_pad_force + right_pad_force) / 2)
            p.stepSimulation()
        
        return sum(gripper_distances)/smooth, sum(forces)/smooth

    def run_test(self, youngs_modulus, compression_distances):
        self.physicsClient = self.set_simulation_params()
        assert self.physicsClient != 1

        planeId = p.loadURDF("plane.urdf", [0, 0, 0])

        self.robot.load()
        self.robot.step_simulation = p.stepSimulation
        self.robot.reset()
        self.robot.move_gripper_length(0.06)
        for _ in range(20):
          p.stepSimulation()
        pos = self.robot.get_gripper_middle_pad_pos()
        self.ballId = self.load_soft_ball(youngs_modulus, pos)


        distances = []
        forces = []
        for distance in compression_distances:
            d, f = self.move_gripper_and_get_forces(distance, transition_duration=0.5)
            distances.append(d)
            forces.append(f)
        
        p.disconnect(self.physicsClient)

        return (distances, forces)
