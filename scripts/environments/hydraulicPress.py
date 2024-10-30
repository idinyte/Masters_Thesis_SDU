import time
import os
import numpy as np
import pybullet as p
import pybullet_data
import matplotlib.pyplot as plt

SIMULATION_STEP = 1/1000

class HydraulicPress():

    def load_soft_ball(self, youngs_modulus):
        density = 400
        ball_radius_m = 0.045
        volume = 4*np.pi*ball_radius_m**3/3
        mass = density*volume
        poson_ratio = 0.4

        neo_mu = youngs_modulus / (2 * (1 + poson_ratio))
        neo_lambda = youngs_modulus * poson_ratio / ((1 + poson_ratio)*(1 - 2 * poson_ratio))
        ballId = p.loadSoftBody(os.path.join(os.getcwd(), "assets/objects/softBall/ball_regular.obj"), 
                                simFileName=os.path.join(os.getcwd(), "assets/objects/softBall/ball_regular.vtk"), 
                                basePosition=[0, 0, ball_radius_m], 
                                mass=mass, 
                                useNeoHookean=1, 
                                NeoHookeanMu=neo_mu, 
                                NeoHookeanLambda=neo_lambda,
                                NeoHookeanDamping=0.001,
                                useSelfCollision=1,
                                repulsionStiffness=800,
                                frictionCoeff=0.5, 
                                collisionMargin=0.0001)
        
        for _ in range(20):
            p.stepSimulation()

        return ballId

    def set_simulation_params(self):
        physic_client = p.connect(p.SHARED_MEMORY)
        p.setTimeStep(SIMULATION_STEP)
        p.setRealTimeSimulation(0)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.resetSimulation(p.RESET_USE_DEFORMABLE_WORLD)
        #p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 0)
        p.setGravity(0, 0, -10)
        p.resetDebugVisualizerCamera(cameraDistance=0.5, 
                                        cameraYaw=0.0, 
                                        cameraPitch=-5.0, 
                                        cameraTargetPosition=[0, 0, 0])
        p.setPhysicsEngineParameter(sparseSdfVoxelSize=0.2)
        p.setPhysicsEngineParameter(numSubSteps=1)
        return physic_client
        
    def load_hydraulic_press(self):
        hydraulic_press_id = p.loadURDF(os.path.join(os.getcwd(), "assets/objects/hydraulic_press/press.urdf"))
        p.setJointMotorControl2(hydraulic_press_id, 1, controlMode=p.VELOCITY_CONTROL, force=0)
        p.setJointMotorControl2(hydraulic_press_id, 0, controlMode=p.VELOCITY_CONTROL, force=0)
        return hydraulic_press_id

    def plot(self, pos_world_frame, compressive_normal_force):
        # Create a plot of the compressive normal force vs the press position
        plt.figure(figsize=(8, 6))
        plt.plot(compressive_normal_force, pos_world_frame, marker='o')
        plt.title('Compressiing a 90 mm diameter soft ball')
        plt.ylabel('Position of the Hydraulic Press (m)')
        plt.xlabel('Compressive Normal Force (N)')
        plt.grid(True)
        plt.show()
        
    def calculate_volume(self, object_id):
        aabb_min, aabb_max = p.getAABB(object_id)
        width = aabb_max[0] - aabb_min[0]
        height = aabb_max[1] - aabb_min[1]
        thickness = aabb_max[2] - aabb_min[2]

        volume = width * height * thickness
        return volume

    def reach_targe_pos(self, target_position, hydraulic_press_id, press_joint_index):
        threshold = 0.00005  # Acceptable error for position
        pos = 0

        while abs(pos - target_position) > threshold:
            pos, vel, rf, torque = p.getJointState(hydraulic_press_id, press_joint_index)
            p.stepSimulation()
            
        return pos

    def run_test(self, youngs_modulus, compression_distances = [0, 0.005, 0.01, 0.015], plot = False):
        self.physicsClient = self.set_simulation_params()
        assert self.physicsClient != 1

        planeId = p.loadURDF("plane.urdf", [0, 0, 0])
        ballId = self.load_soft_ball(youngs_modulus)
        
        hydraulic_press_id = self.load_hydraulic_press()
        #p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)
        
        # Positional control settings
        #pos_world_frame = [0.09 - pos for pos in compression_distances]
        target_positions = [0.01 + distance for distance in compression_distances]
        press_joint_index = 1
        max_force = 1000
        
        # Data collection lists
        pos_world_frame_list = []
        compressive_normal_force_list = []
        

        for target_position in target_positions:
            p.setJointMotorControl2(
                bodyIndex=hydraulic_press_id,
                jointIndex=press_joint_index,
                controlMode=p.POSITION_CONTROL,
                targetPosition=target_position,
                force=max_force,
                positionGain=1
            )
            
            pos = self.reach_targe_pos(target_position, hydraulic_press_id, press_joint_index)
            pos_world_frame = max(0.1 - pos, 0)
            
            # Wait some time in this position to let the simulation settle
            for _ in range(int(0.2 / SIMULATION_STEP)):
                p.stepSimulation()
                
            
            contact_points = p.getContactPoints(bodyA=hydraulic_press_id, bodyB=ballId, linkIndexA=1)
            compressive_normal_force = sum(contact[9] for contact in contact_points)

            # Store the data
            pos_world_frame_list.append(pos_world_frame)
            compressive_normal_force_list.append(compressive_normal_force)

        # Plot the collected data
        if plot:
            self.plot(pos_world_frame_list, compressive_normal_force_list)
        
        p.disconnect(self.physicsClient)

        return (pos_world_frame_list, compressive_normal_force_list)

        # Keep the simulation running to observe the behavior
        while True:
            pos, vel, rf, torque = p.getJointState(hydraulic_press_id, press_joint_index)
            pos_world_frame = max(0.1 - pos, 0)
            contact_points = p.getContactPoints(bodyA=hydraulic_press_id, bodyB=ballId, linkIndexA=1)
            compressive_normal_force = sum(contact[9] for contact in contact_points)
            print(f"{pos_world_frame} {compressive_normal_force}")
            p.stepSimulation()