from scripts.environments.commonEnv import CommonEnv
from scripts.objects.gripper import ControlType
import pybullet as p
import numpy as np
import keyboard

class VREnv(CommonEnv):
    def __init__(self, robot, camera=None, vis=True, realtime=False, debug=False, VR=True, VRCameraPos = [0,-3, 1.5], VRCameraRot=[0,0,0], SIMULATION_STEP=1/240, gripper_controller=False, gripper=None, fixed_gripper_ori = False):
        self.vis = True
        self.VR = True
        self.VRCameraPos = VRCameraPos
        self.VRCameraRot = VRCameraRot
        self.SIMULATION_STEP = SIMULATION_STEP
        self.done = False
        self.fixed_gripper_ori = fixed_gripper_ori
        
        self.translation_offset = [0.02, 0, -0.04]
        self.rotation_offset = [127, -18, -11]
        self.mode = "rotation"

        super().__init__(robot, camera, self.vis, realtime, debug, VR=self.VR, SIMULATION_STEP=SIMULATION_STEP)

        # Enable VR mode
        p.setVRCameraState(VRCameraPos, p.getQuaternionFromEuler(VRCameraRot))
        self.vr_controller1_left = None
        self.vr_controller2_right = None
        self.vr_headset = None
        self.gripper = gripper
        self.gripper_controller = gripper_controller
        
        if self.gripper != None:
            self.create_gripper()
            
    def handle_keyboard_input(self):
        # Toggle mode with Spacebar
        if keyboard.is_pressed("space"):
            self.mode = "rotation" if self.mode == "translation" else "translation"
            print(f"Mode changed to: {self.mode}")
            keyboard.wait("space")  # Prevent multiple toggles from long press

        if self.mode == "translation":
            if keyboard.is_pressed("1"):
                self.translation_offset[0] -= 0.01
            if keyboard.is_pressed("2"):
                self.translation_offset[1] -= 0.01
            if keyboard.is_pressed("3"):
                self.translation_offset[2] -= 0.01
            if keyboard.is_pressed("4"):
                self.translation_offset[0] += 0.01
            if keyboard.is_pressed("5"):
                self.translation_offset[1] += 0.01
            if keyboard.is_pressed("6"):
                self.translation_offset[2] += 0.01

        elif self.mode == "rotation":
            if keyboard.is_pressed("1"):
                self.rotation_offset[0] -= 1
            if keyboard.is_pressed("2"):
                self.rotation_offset[1] -= 1
            if keyboard.is_pressed("3"):
                self.rotation_offset[2] -= 1
            if keyboard.is_pressed("4"):
                self.rotation_offset[0] += 1
            if keyboard.is_pressed("5"):
                self.rotation_offset[1] += 1
            if keyboard.is_pressed("6"):
                self.rotation_offset[2] += 1
    
    def get_gripper_target_pose(self):
        #self.handle_keyboard_input()  # Process keyboard input

        target_position = list(self.vr_controller2_right["position"])

        # Apply translation offset
        target_position[0] += self.translation_offset[0]
        target_position[1] += self.translation_offset[1]
        target_position[2] += self.translation_offset[2]

        if self.fixed_gripper_ori:
            target_orientation = p.getQuaternionFromEuler([np.radians(180),
                0,
                np.radians(-90)])
        else:
            # Apply rotation offset (Convert degrees to radians)
            rotation_quaternion = p.getQuaternionFromEuler([
                np.radians(self.rotation_offset[0]),
                np.radians(self.rotation_offset[1]),
                np.radians(self.rotation_offset[2])
            ])
            target_orientation = p.multiplyTransforms(
                [0, 0, 0], self.vr_controller2_right["orientation"], [0, 0, 0], rotation_quaternion
            )[1]
            
        #print(self.rotation_offset)
        return target_position, target_orientation

    def update_gripper_pose(self):
        target_position, target_orientation = self.get_gripper_target_pose()
        self.gripper.track_pose(target_position, target_orientation)
        
    def create_gripper(self):
        while self.vr_controller2_right == None:
            self.update_vr_tracking()
        target_position, target_orientation = self.get_gripper_target_pose()
        self.gripper.initialize_gripper_controller(target_position, target_orientation)
        
    def update_vr_tracking(self):
        vr_events = p.getVREvents()
        for event in vr_events:
            controller_id, pos, orn, controllerAnalogueAxis, numButtonEvents, numMoveEvents, buttons, deviceType = event
            if controller_id == 1:
                self.vr_controller2_right = {
                    "position": pos,
                    "orientation": orn
                }
                
                # B button
                if buttons[1] == p.VR_BUTTON_IS_DOWN:
                    self.done = True
                
                # Side button, reinitialize gripper
                if buttons[2] == p.VR_BUTTON_IS_DOWN:
                    if self.gripper != None:
                        p.removeBody(self.gripper.id)
                    self.create_gripper()
                
                # if self.gripper != None:
                #     # Close gripper depending on trigger press range
                #     self.gripper.move_gripper_length(self.gripper.gripper_range[1] - controllerAnalogueAxis * self.gripper.gripper_range[1])
            # elif controller_id == 1:
            #     self.vr_controller1_left = {
            #         "position": pos,
            #         "orientation": orn
            #     }

        if self.gripper != None and self.vr_controller2_right:
            self.update_gripper_pose()


    def track_gripper_opening(self, ball_id):
        self.gripper.exosceleton_update(ball_id, ControlType.Current, verbose = False, plot = False)

    def step_simulation(self, ball_id = None):
        self.update_vr_tracking()
        if ball_id != None:
            self.track_gripper_opening(ball_id)
        super().step_simulation()