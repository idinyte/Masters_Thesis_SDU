from scripts.environments.commonEnv import CommonEnv
from scripts.objects.gripper import Gripper
import pybullet as p
import numpy as np
import os

class VREnv(CommonEnv):
    def __init__(self, robot, camera=None, vis=True, realtime=False, debug=False, VR=True, VRCameraPos = [0,-3, 1.5], VRCameraRot=[0,0,0], SIMULATION_STEP=1/240, gripper_controller=False):
        self.vis = True
        self.VR = True
        self.VRCameraPos = VRCameraPos
        self.VRCameraRot = VRCameraRot
        self.SIMULATION_STEP = SIMULATION_STEP

        super().__init__(robot, camera, self.vis, realtime, debug, VR=self.VR, SIMULATION_STEP=SIMULATION_STEP)

        # Enable VR mode
        p.setVRCameraState(VRCameraPos, p.getQuaternionFromEuler(VRCameraRot))
        self.vr_controller1_left = None
        self.vr_controller2_right = None
        self.vr_headset = None
        self.gripper = None
        self.gripper_controller = gripper_controller
    
    def get_gripper_target_pose(self):
        target_position = self.vr_controller2_right["position"]
        
        # Apply a 90-degree rotation to the gripper's orientation
        rotation_quaternion = p.getQuaternionFromEuler([0, np.pi / 2, 0])
        target_orientation = p.multiplyTransforms([0, 0, 0], self.vr_controller2_right["orientation"], [0, 0, 0], rotation_quaternion)[1]
        
        return target_position, target_orientation

    def update_gripper_pose(self):
        target_position, target_orientation = self.get_gripper_target_pose()
        self.gripper.track_pose(target_position, target_orientation)
        
    def create_gripper(self):
        self.gripper = Gripper(self.SIMULATION_STEP)
        target_position, target_orientation = self.get_gripper_target_pose()
        self.gripper.initialize_gripper_controller(target_position, target_orientation)
        
    def update_vr_tracking(self):
        vr_events = p.getVREvents()
        
        for event in vr_events:
            controller_id, pos, orn, controllerAnalogueAxis, numButtonEvents, numMoveEvents, buttons, deviceType = event
            if controller_id == 2:
                self.vr_controller2_right = {
                    "position": pos,
                    "orientation": orn
                }
                
                # B button, reinitialize gripper
                if buttons[1] == p.VR_BUTTON_IS_DOWN:
                    if self.gripper != None:
                        p.removeBody(self.gripper.id)
                    self.create_gripper()
                
                if self.gripper != None:
                    # Close gripper depending on trigger press range
                    self.gripper.move_gripper_length(self.gripper.gripper_range[1] - controllerAnalogueAxis * self.gripper.gripper_range[1])
            # elif controller_id == 1:
            #     self.vr_controller1_left = {
            #         "position": pos,
            #         "orientation": orn
            #     }

        if self.gripper != None and self.vr_controller2_right:
            self.update_gripper_pose()

    def step_simulation(self):
        self.update_vr_tracking()
        
        if self.gripper_controller and self.gripper == None and self.vr_controller2_right != None:
            self.create_gripper()

        super().step_simulation()