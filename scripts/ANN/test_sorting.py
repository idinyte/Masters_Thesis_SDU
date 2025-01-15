import sys
import os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../..')))
from scripts.ANN.TestSortBallsANN import TestSortBallsANN

for name in ["1", "2", "3", "4"]:
  test_env = TestSortBallsANN(ball_class_name=name)
  for _ in range(100):
    test_env.run_once()