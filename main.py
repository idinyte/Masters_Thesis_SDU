from ur5 import UR5Robot
from environments.commonEnv import CommonEnv
from environments.VREnv import VREnv

# Initialize the UR5 robot
robot = UR5Robot(urdf_path=r"C:\Users\Vivobook\Desktop\Masters_Thesis_SDU\assets\UR5\urdf\ur5_robotiq_140.urdf", base_position=[0, 0, 0], base_orientation=[0, 0, 0, 1], use_fixed_base=True)
env = VREnv(robot, vis=True, debug=False, realtime=False)
#robot._gripper_distance_formula()

print("Start")
while env.connected:
  if env.debug:
    x, y, z, roll, pitch, yaw, gripper_opening_length = env.read_debug_parameter()
    robot.move_gripper_length(gripper_opening_length)
    #print(f"{robot.actual_gripper_finger_distance(visualize=True):.4f} m")
  if not env.realtime:
    env.step_simulation()
  continue