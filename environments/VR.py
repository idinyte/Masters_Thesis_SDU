from commonEnv import CommonEnv
import pybullet as p

class VREnv(CommonEnv):
    def __init__(self, robot, camera=None, vis=True, realtime=False, debug=False):
        self.vis = True
        super().__init__(robot, camera, self.vis, realtime, debug)

        # Enable VR mode
        p.setVRCameraState([1,1,1],p.getQuaternionFromEuler([0,0,-90]))
        self.vr_controller1 = None
        self.vr_controller2 = None
        self.vr_headset = None

    def update_vr_tracking(self):
        # Fetch the position and orientation of VR headset and controllers
        vr_events = p.getVREvents()

        for event in vr_events:
            if event[0] == p.VR_DEVICE_CONTROLLER1:
                self.vr_controller1 = event
            elif event[0] == p.VR_DEVICE_CONTROLLER2:
                self.vr_controller2 = event
            elif event[0] == p.VR_DEVICE_HMD:
                self.vr_headset = event

    def step_simulation(self):
        # First, update VR tracking for each simulation step
        self.update_vr_tracking()

        # Proceed with the normal simulation step
        super().step_simulation()
