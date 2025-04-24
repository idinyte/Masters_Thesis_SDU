from imitation.data import serialize

# Load the saved trajectories
loaded_trajectories = serialize.load("scripts/IRL/trajectories/processed_for_imitation_2_serialized")

# Print basic info for inspection
for i, traj in enumerate(loaded_trajectories):
    print(f"Trajectory {i}:")
    print(f"  Observations shape: {traj.obs.shape}")
    print(f"  Actions shape: {traj.acts.shape}")
    print(f"  Terminal: {traj.terminal}")
    print(f"  First obs: {traj.obs[60:80]}")
    print(f"  First act: {traj.acts[60:80]}")
    print()
