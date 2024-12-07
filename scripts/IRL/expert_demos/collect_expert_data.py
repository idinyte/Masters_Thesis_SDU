import sys
import os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../../..')))
from scripts.ANN.TestSortBallsANN import TestSortBallsANN
from scripts.environments.sortBallsEnv import BALLS_MAP
import numpy as np

def collect_expert_demonstrations(demos = 10):
    backup = os.path.join(os.getcwd(), "scripts/IRL/expert_demos/demos_backup.txt")
    if not os.path.exists(backup):
        with open(backup, "w") as file:
            pass

    demonstrations = []
    ball_classes = list(BALLS_MAP.keys())
    testANN = TestSortBallsANN(record_trajectory=True)
    for ball_class in ball_classes:
        testANN.ball_class_name = ball_class
        i = 0
        while i < demos:
            success = testANN.run_once()
            if not success:
                continue
            
            demonstration = testANN.algorithm.trajectory.recorded_trajectory
            demonstrations.append(demonstration)
            with open(backup, "a") as file:
                file.write(f"{demonstration}\n")
            i += 1
    return demonstrations

expert_demos = collect_expert_demonstrations()
save_path = os.path.join(os.getcwd(), "scripts/IRL/expert_demos/expert_demos.npy")
np.save(save_path, np.array(expert_demos, dtype=object))
print(f"Demonstrations saved to {save_path}")