import sys
import os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../..')))
from scripts.IRL.environment import GymWrapper as Env
import time
from stable_baselines3.common.env_checker import check_env

env = Env()
