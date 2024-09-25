from environments.commonEnv import CommonEnv
import pybullet as p
import numpy as np

class VREnv(CommonEnv):
    def __init__(self, robot, camera=None, vis=True, realtime=False, debug=False, VR=True):
        self.vis = True
        self.VR = True
        super().__init__(robot, camera, self.vis, realtime, debug, VR=self.VR)

        # Enable VR mode
        p.setVRCameraState([0,-3, 1.5],p.getQuaternionFromEuler([0,0,0]))
        self.vr_controller1 = None
        self.vr_controller2 = None
        self.vr_headset = None

    def update_vr_tracking(self):
        # Fetch the position and orientation of VR headset and controllers
        vr_events = p.getVREvents(p.VR_DEVICE_HMD + p.VR_DEVICE_GENERIC_TRACKER)

        # Check if there are any VR events
        # for event in vr_events:
        #     print(f"Event {event[0]} expected {p.VR_DEVICE_HMD}")



    def step_simulation(self):
        # First, update VR tracking for each simulation step
        self.update_vr_tracking()

        # Proceed with the normal simulation step
        super().step_simulation()
