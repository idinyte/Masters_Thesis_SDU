import numpy as np

data = np.load("scripts/IRL/trajectories/processed/ball-class-0/trajectory_1744185036.npy", allow_pickle=True)

print(data.shape)
print(data[100:110])