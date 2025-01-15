import sys
import os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../..')))
from gym import Env
from gym import spaces
import numpy as np
import pybullet as p
from scripts.objects.ur5 import UR5Robot
from scripts.environments.sortBallsEnv import SortBallsEnv

class GymWrapper(Env):
  def __init__(self, max_episode_steps = 10000, vis = False):
    self.vis = vis
    self.max_episode_steps = max_episode_steps
    self.episode = 0
    self.max_reward = -np.inf
    self.reset()
    self.observation_space = spaces.Box(low=-np.inf, high=np.inf, shape=self.state.shape, dtype=np.float32)
    self.action_space = spaces.Box(low=np.array([-0.02, -0.02, -0.02, 0]),
                               high=np.array([0.02, 0.02, 0.02, 0.127]),
                               dtype=np.float32)

  def step(self, action):
    info = {}
    self._perform_action(action)
    new_state = self.env.main_loop(gym_state = True)
    self.state = new_state
    reward = self._get_reward()
    self.cumulative_reward += reward
    self.step_counter += 1
    
    done = self.env.terminal_state or self.env.restart_episode or self.step_counter > self.max_episode_steps
    
    return self.state, reward, done, info
  
  def reset(self):
    self.episode += 1
    if self.episode > 1:
      self.max_reward = max(self.max_reward, self.cumulative_reward)

    self.close()

    robot = UR5Robot(urdf_path=os.path.join(os.getcwd(), "assets/objects/UR5/urdf/ur5_robotiq_140_modified.urdf"), base_position=[0, 0, 0], base_orientation=[0.0, 0.0, 0.0, 1.0], use_fixed_base=True)
    camera=None
    debug=False
    realtime=False
    VR=False
    VRCameraPos = [0, 0, 1]
    VRCameraRot = [0, 0, 180]
    robot_base_position = [0, 0, 1]
    self.env = SortBallsEnv(robot, camera, self.vis, realtime, debug, VR, VRCameraPos, VRCameraRot, robot_base_position = robot_base_position)
    self.env.check_ball_health()

    self.cumulative_reward = 0
    self.step_counter = 0

    self.state = self.env.state_to_gym_state(self.env.get_state())
    
    return self.state  
    
  
  def _perform_action(self, action):
    robot_pos, robot_ori = self.env.robot.get_ee_link_pose()
    target_position_delta = action[:3]
    target_position = robot_pos + target_position_delta
    target_orientation = np.array([ 0.49769387,  0.49769387, -0.50189555,  0.50269538])
    target_gripper = action[3]
    
    joint_positions = p.calculateInverseKinematics(self.env.robot.robot_id, self.env.robot.eef_id, target_position, targetOrientation=target_orientation)

    p.setJointMotorControlArray(bodyIndex=self.env.robot.robot_id, jointIndices=[1, 2, 3, 4, 5, 6], controlMode=p.POSITION_CONTROL, targetPositions=joint_positions[:6])
    self.env.robot.move_gripper_length(target_gripper)
    
  def _get_reward(self):
    ball_position_world_coordinates = self.state[13:16] + self.state[7:10]
    
    # reward for gripper being close to the ball
    reward = 0.5 - np.linalg.norm(self.state[13:16])

    # reward for holding ball
    left_pad_force, right_pad_force = self.state[11], self.state[12]
    gripper_opening_length = self.state[10]
    if gripper_opening_length < 0.08:
      if self._is_touching_ball(left_pad_force, right_pad_force) and gripper_opening_length > 0.02:
        reward += 0.5 - 5*abs(gripper_opening_length - 0.04)
        
        # reward for closing distance to target
        reward += max(2 - np.linalg.norm(np.array(self.env.get_corresponding_ball_box()) - np.array(ball_position_world_coordinates)), 0)
      
    # reward for lifting ball
    if not self._is_ball_touching_table():
      reward += 2

    # reward for crashing the environment (ball being too far or exploding)
    if self.env.restart_episode:
      reward -= 50
      
    # reward for being in collision with something that is not ball
    if self.env.robot.non_ball_contact:
      reward -= 1

    # reward for placing ball in correct box
    if self.env.ball.is_in_box(self.env.get_corresponding_ball_box_id()):
      reward += 500
      
    print(reward)

    return reward

  def _is_touching_ball(self, left_pad_force, right_pad_force):
    return left_pad_force > 0 and right_pad_force > 0
  
  def _is_ball_touching_table(self):
    return len(p.getContactPoints(bodyA=self.env.ball.id, bodyB=self.env.table_id)) > 0

  def render(self):
    pass
  
  def close(self):
    try:
      self.env.baseEnv.close()
    except:
      pass