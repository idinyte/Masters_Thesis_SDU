import pybullet as p
import pybullet_data
import time
import numpy as np

class CommonEnv:

    SIMULATION_STEP_DELAY = 1 / 240.

    def __init__(self, robot, camera=None, vis=False, realtime=False, debug=False, VR=False):
        self.robot = robot
        self.vis = vis
        self.realtime = realtime
        self.debug=debug
        self.camera = camera
        self.VR = VR

        # define environment
        if self.VR:
            self.physicsClient = p.connect(p.SHARED_MEMORY)
            assert self.physicsClient != -1, "Could not connect to the VR server. Is App_PhysicsServer_SharedMemory_VR_vs2010_x64_release.exe running?"
        else:
            #self.physicsClient = p.connect(p.GUI if self.vis else p.DIRECT)
            self.physicsClient = p.connect(p.SHARED_MEMORY)
        self.connected = True
        p.resetSimulation()
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.8)
        p.setRealTimeSimulation(1 if self.realtime else 0)

        # Load the plane
        self.planeID = p.loadURDF("plane.urdf", [0, 0, 0], [0, 0, 0, 1])

        # Load robot
        self.robot.load()
        self.robot.step_simulation = self.step_simulation
        self.robot.reset()

        # custom sliders to tune parameters (name of the parameter,range,initial value)
        if self.debug:
            self.xin = p.addUserDebugParameter("x", -0.224, 0.224, 0)
            self.yin = p.addUserDebugParameter("y", -0.224, 0.224, 0)
            self.zin = p.addUserDebugParameter("z", 0, 1., 0.5)
            self.rollId = p.addUserDebugParameter("roll", -3.14, 3.14, 0)
            self.pitchId = p.addUserDebugParameter("pitch", -3.14, 3.14, np.pi/2)
            self.yawId = p.addUserDebugParameter("yaw", -np.pi/2, np.pi/2, np.pi/2)
            self.gripper_opening_length_control = p.addUserDebugParameter("gripper_opening_length", self.robot.gripper_range[0], self.robot.gripper_range[1], (self.robot.gripper_range[1] - self.robot.gripper_range[0]) / 2)

    def step_simulation(self):
        if self.realtime:
          return

        p.stepSimulation()
        if self.vis:
            time.sleep(self.SIMULATION_STEP_DELAY)

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