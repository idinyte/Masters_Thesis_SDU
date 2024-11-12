import pybullet as p
import pybullet_data
import time
import numpy as np

class CommonEnv:


    def __init__(self, robot, camera=None, vis=False, realtime=False, debug=False, VR=False, SIMULATION_STEP=1/240):
        self.robot = robot
        self.vis = vis
        self.realtime = realtime
        self.debug=debug
        self.camera = camera
        self.VR = VR
        self.SIMULATION_STEP = SIMULATION_STEP

        # define environment
        if self.VR:
            self.physicsClient = p.connect(p.SHARED_MEMORY)
        else:
            self.physicsClient = p.connect(p.GUI if self.vis else p.DIRECT)
        
        assert self.physicsClient != -1, "Could not connect to the bullet server."
        self.connected = True

        p.resetSimulation(p.RESET_USE_DEFORMABLE_WORLD)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.8)
        p.setRealTimeSimulation(1 if self.realtime else 0)
        p.setPhysicsEngineParameter(sparseSdfVoxelSize=0.25)
        p.setTimeStep(self.SIMULATION_STEP)
        
        # Load the plane
        self.planeID = p.loadURDF("plane.urdf", [0, 0, 0], [0, 0, 0, 1])

        # Load robot
        self.robot.load()
        self.robot.step_simulation = self.step_simulation
        self.robot.reset()

        # custom sliders to tune parameters (name of the parameter,range,initial value)
        if self.debug:
            self.xin = p.addUserDebugParameter("x", -2, 2, 0)
            self.yin = p.addUserDebugParameter("y", -2, 2, -0.5)
            self.zin = p.addUserDebugParameter("z", 0, 2, 1.22)
            self.rollId = p.addUserDebugParameter("roll", -3.14, 3.14, 0)
            self.pitchId = p.addUserDebugParameter("pitch", -3.14, 3.14, np.pi/2)
            self.yawId = p.addUserDebugParameter("yaw", -3*np.pi, 2*np.pi, np.pi/2)
            self.gripper_opening_length_control = p.addUserDebugParameter("gripper_opening_length", self.robot.gripper_range[0], self.robot.gripper_range[1], 0.1)
            
        # debug camera position
        if self.vis:
            p.resetDebugVisualizerCamera(cameraDistance=2, 
                              cameraYaw=0.0, 
                              cameraPitch=-15.0, 
                              cameraTargetPosition=[0, 0, 1.5])

    def step_simulation(self):
        if self.realtime:
          time.sleep(self.SIMULATION_STEP)

        p.stepSimulation()
        # if self.vis:
        #     time.sleep(self.SIMULATION_STEP_DELAY)

    def main_loop(self):
        self.step_simulation()

    def is_connected(self):
        return self.connected
            
    def read_debug_parameter(self):
        x = p.readUserDebugParameter(self.xin)
        y = p.readUserDebugParameter(self.yin)
        z = p.readUserDebugParameter(self.zin)
        roll = p.readUserDebugParameter(self.rollId)
        pitch = p.readUserDebugParameter(self.pitchId)
        yaw = p.readUserDebugParameter(self.yawId)
        gripper_opening_length = p.readUserDebugParameter(self.gripper_opening_length_control)
        return x, y, z, roll, pitch, yaw, gripper_opening_length
            
    def close(self):
        self.connected = False
        p.disconnect(self.physicsClient)