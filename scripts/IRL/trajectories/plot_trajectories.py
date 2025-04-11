import numpy as np
import os
import glob
import matplotlib.pyplot as plt
import seaborn as sns

# Initialize lists
all_first_diffs = []
all_second_diffs = []
all_third_diffs = []
all_fourth_diffs = []

# Load and compute diffs
for class_idx in range(4):
    folder_path = f'scripts/IRL/trajectories/ball-class-{class_idx}'
    file_paths = glob.glob(os.path.join(folder_path, '*.npy'))

    for file_path in file_paths:
        data = np.load(file_path)
        if data.shape[0] < 2 or data.shape[1] < 6:
            continue

        all_first_diffs.extend(data[1:, 0] - data[:-1, 0])
        all_second_diffs.extend(data[1:, 1] - data[:-1, 1])
        all_third_diffs.extend(data[1:, 2] - data[:-1, 2])
        all_fourth_diffs.extend(data[:, 3])

# Convert to numpy arrays
all_first_diffs = np.array(all_first_diffs)
all_second_diffs = np.array(all_second_diffs)
all_third_diffs = np.array(all_third_diffs)
all_fourth_diffs = np.array(all_fourth_diffs)

# Plot distributions
plt.figure(figsize=(15, 10))
sns.set(style="whitegrid")

titles = [
    "Pos ΔX", "Pos ΔY", "Pos ΔZ",
    "Gripper Openings"
]
diffs = [
    all_first_diffs, all_second_diffs, all_third_diffs,
    all_fourth_diffs
]

for i in range(4):
    plt.subplot(2, 2, i + 1)
    sns.histplot(diffs[i], bins=100, kde=True)
    plt.title(f"{titles[i]}")
    plt.xlabel("Difference")
    plt.ylabel("Frequency")

plt.tight_layout()
plt.show()
