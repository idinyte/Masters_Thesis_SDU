import numpy as np
import os
import glob
from imitation.data.types import Trajectory # Import the Trajectory type
from imitation.data import serialize        # Import the serialize module

in_base = 'scripts/IRL/trajectories/ball-class-{}'
# Output path for the serialized data (can be a directory)
processed_all_path = 'scripts/IRL/trajectories/processed_for_imitation_serialized'
os.makedirs(processed_all_path, exist_ok=True) # serialize.save often expects a directory

ACTION_CLAMP = 0.02
GRIPPER_MIN, GRIPPER_MAX = 0.0, 0.127
NOISE_STD = 0.05

all_trajectories = [] # List to hold Trajectory objects

for class_idx in range(4):
    in_folder = in_base.format(class_idx)
    file_paths = glob.glob(os.path.join(in_folder, '*.npy'))

    for file_path in file_paths:
        data = np.load(file_path)
        if data.shape[0] < 2 or data.shape[1] < 4:
            print(f"Skipping short/invalid trajectory: {file_path}")
            continue

        # Store transitions for this trajectory
        traj_obs = []
        traj_acts = []
        traj_infos = [] # infos are often expected, even if empty
        # traj_rews = [] # If you have rewards, collect them too

        for t in range(data.shape[0] - 1):
            state = data[t].astype(np.float32) # Ensure correct dtype
            next_state = data[t + 1].astype(np.float32)

            delta_pos = next_state[:3] - state[:3]
            noise = np.random.normal(0.0, np.abs(delta_pos) * NOISE_STD)
            noisy_delta_pos = delta_pos + noise
            noisy_delta_pos = np.clip(noisy_delta_pos, -ACTION_CLAMP, ACTION_CLAMP)
            gripper = np.clip(next_state[3], GRIPPER_MIN, GRIPPER_MAX)
            action = np.concatenate([noisy_delta_pos, [gripper]]).astype(np.float32)

            traj_obs.append(state)
            traj_acts.append(action)
            traj_infos.append({}) # Add an empty info dict for each step

        # Also need the final observation for the trajectory
        final_obs = data[-1].astype(np.float32)
        traj_obs.append(final_obs)

        # Check if trajectory has valid data before creating object
        if not traj_acts: # Need at least one action
             print(f"Skipping trajectory with no actions: {file_path}")
             continue

        # Create Trajectory object (without rewards)
        # Note: Trajectory expects observations to be one longer than actions/infos
        trajectory = Trajectory(
            obs=np.array(traj_obs),
            acts=np.array(traj_acts),
            infos=np.array(traj_infos),
            terminal=True # Assume each file is a complete trajectory ending in terminal state
        )
        all_trajectories.append(trajectory)
        print(f"Processed {file_path}, added trajectory with {len(trajectory.acts)} steps.")

if not all_trajectories:
     print("Error: No valid trajectories were processed. Cannot save.")
else:
     # Save the list of Trajectory objects using serialize.save
     print(f"Saving {len(all_trajectories)} trajectories to {processed_all_path}...")
     serialize.save(processed_all_path, all_trajectories)
     print("Demonstrations saved successfully using imitation.data.serialize.save.")