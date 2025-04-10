import numpy as np
import os
import glob

# Config
in_base = 'scripts/IRL/trajectories/ball-class-{}'
out_base = 'scripts/IRL/trajectories/processed/ball-class-{}'
os.makedirs('scripts/IRL/trajectories/processed', exist_ok=True)

# Constants
ACTION_CLAMP = 0.02
GRIPPER_MIN, GRIPPER_MAX = 0.0, 0.127
NOISE_STD = 0.05  # 10%

for class_idx in range(4):
    in_folder = in_base.format(class_idx)
    out_folder = out_base.format(class_idx)
    os.makedirs(out_folder, exist_ok=True)

    file_paths = glob.glob(os.path.join(in_folder, '*.npy'))

    for file_path in file_paths:
        data = np.load(file_path)
        if data.shape[0] < 2 or data.shape[1] < 4:
            continue

        processed_trajectory = []

        for t in range(data.shape[0] - 1):
            state = data[t]
            next_state = data[t + 1]

            delta_pos = next_state[:3] - state[:3]

            noise = np.random.normal(0.0, np.abs(delta_pos) * NOISE_STD)
            noisy_delta_pos = delta_pos + noise

            noisy_delta_pos = np.clip(noisy_delta_pos, -ACTION_CLAMP, ACTION_CLAMP)

            gripper = np.clip(next_state[3], GRIPPER_MIN, GRIPPER_MAX)

            action = np.concatenate([noisy_delta_pos, [gripper]])

            processed_trajectory.append([state, action])

        processed_trajectory = np.array(processed_trajectory, dtype=object)

        filename = os.path.basename(file_path)
        save_path = os.path.join(out_folder, filename)
        np.save(save_path, processed_trajectory)
