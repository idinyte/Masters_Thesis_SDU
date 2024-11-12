import sys
import os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../..')))
import random
import scripts.environments.sortBallsEnv as sortBallsEnv
import scripts.environments.robotBallPress as robotBallPress


BALLS_MAP = sortBallsEnv.BALLS_MAP

# prepare folder
data_folder_path = os.path.join(os.path.dirname(__file__), 'data2')
data_file_path = os.path.join(data_folder_path, 'data.txt')
os.makedirs(data_folder_path, exist_ok=True)
with open(data_file_path, 'w') as file:
    pass

# generate data
DATA_POINTS = 5000
for i in range(DATA_POINTS):
  ball_name = random.choice(list(BALLS_MAP.keys()))
  min_youngs_modulus, max_youngs_modulus = BALLS_MAP[ball_name]
  youngs_modulus = random.randint(min_youngs_modulus, max_youngs_modulus)
  compression_distances = [0.05, 0.04, 0.07, 0.06]
  
  robot_press_env = robotBallPress.RobotPress()
  # pos_world_frame == squished ball diameter
  (pos_world_frame_list, compression_forces) = robot_press_env.run_test(youngs_modulus, compression_distances)
  
  data_str=""
  for pos_world_frame, force in zip(pos_world_frame_list, compression_forces):
    data_str += f"{pos_world_frame} "
    data_str += f"{force} "

  with open(data_file_path, 'a') as file:
    file.write(f"{ball_name} {data_str}\n")