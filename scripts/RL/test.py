import sys
import os
import time
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../..')))
from scripts.RL.environment import GymWrapper as Env
from stable_baselines3 import SAC

checkpoint_path = os.path.join(os.getcwd(), "scripts/IRL/logs_gail/checkpoints/checkpoint_800000.zip") # best scripts/IRL/logs_gail/checkpoints/checkpoint_1350000

env = Env(vis=True)

seed = 1111
env.seed(seed)

model = SAC.load(checkpoint_path, env=env)

num_episodes = 10

for episode in range(num_episodes):
    observation = env.reset()
    done = False
    total_reward = 0
    while not done:
        action, _states = model.predict(observation, deterministic=True)
        observation, reward, done, info = env.step(action)
        total_reward += reward
        # time.sleep(0.05)  # Slow down for visualization, adjust as needed
        #print(f"Observation {observation}: \n\n action = {action} reward {reward}")
        # input("Press Enter to continue...")

env.close()
