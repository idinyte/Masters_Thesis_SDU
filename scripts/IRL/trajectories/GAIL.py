import os
import torch
import sys
import glob
import re
from stable_baselines3 import SAC
from stable_baselines3.common.vec_env import DummyVecEnv
from imitation.algorithms.adversarial.gail import GAIL
from imitation.rewards import reward_nets
from imitation.data import serialize
from imitation.util import logger as imitation_logger
from sacred import Experiment
from sacred.observers import FileStorageObserver
import shutil
import time

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../../..')))
from scripts.RL.environment import GymWrapper as Env

DEMONSTRATIONS_PATH = 'scripts/IRL/trajectories/processed_for_imitation_serialized'

# --- Sacred Experiment Setup ---
ex = Experiment("gail_experiment")
ex.observers.append(FileStorageObserver('scripts/IRL/logs_gail/sacred_runs'))

@ex.config
def my_config():
    log_format_strs = ["tensorboard", "csv"]
    source = "file"
    path = None
    checkpoint_path = None

@ex.main
def run(_run, checkpoint_path):
    # Configuration
    SEED = 1111
    TOTAL_TIMESTEPS = 10_000_000
    CHECKPOINT_INTERVAL = 50_000
    DEVICE = torch.device("cuda" if torch.cuda.is_available() else "cpu")

    # Paths
    LOG_DIR = os.path.abspath("scripts/IRL/logs_gail/")
    CHECKPOINT_DIR = os.path.join(LOG_DIR, "checkpoints")
    TENSORBOARD_DIR = os.path.join(LOG_DIR, "tensorboard")
    CSV_DIR = os.path.join(LOG_DIR, "csv")
    FINAL_MODEL_PATH = os.path.abspath("scripts/IRL/trained_policy_gail/gail_sac_final_model")

    # Create directories
    for d in [LOG_DIR, CHECKPOINT_DIR, TENSORBOARD_DIR, CSV_DIR, os.path.dirname(FINAL_MODEL_PATH)]:
        os.makedirs(d, exist_ok=True)

    # Configure Logging for imitation
    logger = imitation_logger.configure(
        folder=LOG_DIR,
        format_strs=["csv"],
    )

    # Load Demonstrations
    try:
        demonstrations = serialize.load(DEMONSTRATIONS_PATH)
        print(f"Loaded {len(demonstrations)} expert trajectories")
    except Exception as e:
        print(f"Failed to load demonstrations: {e}")
        sys.exit(1)

    # Environment
    def make_env():
        env = Env(vis=False)
        env.seed(SEED)
        return env
    venv = DummyVecEnv([make_env])

    # SAC Generator
    policy_kwargs = dict(net_arch=[256, 256])
    gen_algo = SAC(
        "MlpPolicy",
        venv,
        verbose=1,
        seed=SEED,
        learning_rate=0.0003,
        buffer_size=1_000_000,
        batch_size=2048,
        gamma=0.99,
        tau=0.005,
        ent_coef="auto",
        policy_kwargs=policy_kwargs,
        device=DEVICE,
    )

    # Reward Network
    reward_net = reward_nets.BasicRewardNet(
        observation_space=venv.observation_space,
        action_space=venv.action_space,
        hid_sizes=[64, 64],
    ).to(DEVICE)

    # Check for Checkpoint to Resume
    initial_step = 0
    if checkpoint_path:
        print(f"Resuming from specified checkpoint: {checkpoint_path}")
        gen_algo = SAC.load(checkpoint_path, env=venv, device=DEVICE)
        reward_net_path = checkpoint_path.replace(".zip", "_reward_net.pt")
        if os.path.exists(reward_net_path):
            reward_net.load_state_dict(torch.load(reward_net_path))
            print(f"Loaded reward network from {reward_net_path}")
        # Extract step number from checkpoint filename
        match = re.search(r"checkpoint_(\d+)\.zip", checkpoint_path)
        if match:
            initial_step = int(match.group(1))
            print(f"Resuming from step {initial_step}")
    else:
        checkpoint_files = sorted(glob.glob(os.path.join(CHECKPOINT_DIR, "checkpoint_*.zip")))
        if checkpoint_files:
            checkpoint_path = checkpoint_files[-1]
            print(f"Resuming from latest checkpoint: {checkpoint_path}")
            gen_algo = SAC.load(checkpoint_path, env=venv, device=DEVICE)
            reward_net_path = checkpoint_path.replace(".zip", "_reward_net.pt")
            if os.path.exists(reward_net_path):
                reward_net.load_state_dict(torch.load(reward_net_path))
                print(f"Loaded reward network from {reward_net_path}")
            match = re.search(r"checkpoint_(\d+)\.zip", checkpoint_path)
            if match:
                initial_step = int(match.group(1))
                print(f"Resuming from step {initial_step}")

    # GAIL Trainer
    gail_trainer = GAIL(
        demonstrations=demonstrations,
        demo_batch_size=1024,
        gen_replay_buffer_capacity=gen_algo.buffer_size,
        n_disc_updates_per_round=2,
        venv=venv,
        gen_algo=gen_algo,
        reward_net=reward_net,
        allow_variable_horizon=True,
        custom_logger=logger,
    )

    # Combined Callback for Checkpointing and SAC Logging
    class CombinedCallback:
        def __init__(self, gen_algo, reward_net, logger, checkpoint_dir, checkpoint_interval, initial_step):
            self.gen_algo = gen_algo
            self.reward_net = reward_net
            self.logger = logger
            self.checkpoint_dir = checkpoint_dir
            self.checkpoint_interval = checkpoint_interval
            self.initial_step = initial_step

        def __call__(self, n_steps):
            # Adjust step count with initial step
            adjusted_step = n_steps + self.initial_step
            # Checkpointing
            if adjusted_step % self.checkpoint_interval == 0 and adjusted_step > 0:
                self.gen_algo.save(os.path.join(self.checkpoint_dir, f"checkpoint_{adjusted_step}"))
                torch.save(self.reward_net.state_dict(), os.path.join(self.checkpoint_dir, f"checkpoint_{adjusted_step}_reward_net.pt"))
                print(f"Checkpoint saved at step {adjusted_step}")

            # Log SAC metrics safely
            if hasattr(self.gen_algo, "logger"):
                # Create a snapshot to avoid iteration errors
                metrics = dict(self.gen_algo.logger.name_to_value)
                for name, value in metrics.items():
                    self.logger.record(f"sac/{name}", value)
                self.logger.dump(step=adjusted_step)

            return True

    # Training with Combined Callback
    try:
        remaining_timesteps = TOTAL_TIMESTEPS - initial_step
        if remaining_timesteps <= 0:
            print(f"Already reached or exceeded {TOTAL_TIMESTEPS} steps. Exiting.")
            return {"final_model_path": FINAL_MODEL_PATH}
        print(f"Training for {remaining_timesteps} more timesteps from step {initial_step}")
        gail_trainer.train(
            total_timesteps=remaining_timesteps,
            callback=CombinedCallback(
                gen_algo=gen_algo,
                reward_net=reward_net,
                logger=logger,
                checkpoint_dir=CHECKPOINT_DIR,
                checkpoint_interval=CHECKPOINT_INTERVAL,
                initial_step=initial_step,
            ),
        )
    except KeyboardInterrupt:
        print("Training interrupted, saving current state...")
        gen_algo.save(os.path.join(CHECKPOINT_DIR, "interrupted_checkpoint"))
        torch.save(reward_net.state_dict(), os.path.join(CHECKPOINT_DIR, "interrupted_checkpoint_reward_net.pt"))
        print("Saved interrupted checkpoint")
        shutil.copy("scripts/IRL/logs_gail/progress.csv", f"scripts/IRL/logs_gail/progress_{time.time()}.csv")
        sys.exit(0)

    # Save Final Policy
    print("Saving final trained policy...")
    gen_algo.save(FINAL_MODEL_PATH)
    torch.save(reward_net.state_dict(), f"{FINAL_MODEL_PATH}_reward_net.pt")
    print(f"Saved policy to {FINAL_MODEL_PATH}")
    venv.close()
    return {"final_model_path": FINAL_MODEL_PATH}

if __name__ == "__main__":
    ex.run()