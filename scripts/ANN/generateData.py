import sys
import os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../..')))
import random
import scripts.environments.hydraulicPress as hydraulicPress
import scripts.environments.sortBallsEnv as sortBallsEnv

BALLS_MAP = sortBallsEnv.BALLS_MAP

# prepare folder
data_folder_path = os.path.join(os.path.dirname(__file__), 'data')
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
  compression_distances = [random.randint(5, 60)/1000 for _ in range(4)]
  
  press = hydraulicPress.HydraulicPress()
  # pos_world_frame == squished ball diameter
  (pos_world_frame_list, compression_forces) = press.run_test(youngs_modulus, compression_distances, plot=False)
  
  data_str=""
  for pos_world_frame, force in zip(pos_world_frame_list, compression_forces):
    data_str += f"{pos_world_frame} "
    data_str += f"{force} "

  with open(data_file_path, 'a') as file:
    file.write(f"{ball_name} {data_str}\n")