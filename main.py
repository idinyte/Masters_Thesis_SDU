from ur5 import UR5Robot
from environments.commonEnv import CommonEnv

# Initialize the UR5 robot
robot = UR5Robot(urdf_path="assets/UR5/urdf/ur5_robotiq_140.urdf", base_position=[0, 0, 0], base_orientation=[0, 0, 0, 1], use_fixed_base=True)
env = CommonEnv(robot, vis=True, debug=True)
#robot._gripper_distance_formula()

while env.connected:
  if env.debug:
    x, y, z, roll, pitch, yaw, gripper_opening_length = env.read_debug_parameter()
    robot.move_gripper_length(gripper_opening_length)
    print(f"{robot.actual_gripper_finger_distance(visualize=True):.4f} m")
  if not env.realtime:
    env.step_simulation()
  continue