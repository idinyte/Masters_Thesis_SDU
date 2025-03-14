import sys
import os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../..')))
from scripts.RL.environment import GymWrapper as Env
from stable_baselines3 import SAC
from stable_baselines3.common.callbacks import CheckpointCallback
from stable_baselines3.common.logger import configure

log_dir = os.path.join(os.getcwd(), "scripts/RL/logs/")
os.makedirs(log_dir, exist_ok=True)

env = Env()

seed = 1111
env.seed(seed)

policy_kwargs = dict(
    net_arch=[256]
)

model = SAC("MlpPolicy", 
            env, 
            verbose=1, 
            seed=seed,
            learning_rate=0.0007,
            buffer_size=100_000_000,
            batch_size=8192,
            gamma=0.9,
            tau=0.005,
            policy_kwargs=policy_kwargs)

checkpoint_callback = CheckpointCallback(save_freq=150000, save_path=os.path.join(os.getcwd(), "scripts/RL/checkpoints/"), name_prefix='sac_model')
logger = configure(log_dir, ["stdout", "csv", "tensorboard"])
model.learn(total_timesteps=150000000, callback=checkpoint_callback, log_interval=1)

model.save(os.path.join(os.getcwd(), "scripts/RL/trained_policy/sac_final_model"))
