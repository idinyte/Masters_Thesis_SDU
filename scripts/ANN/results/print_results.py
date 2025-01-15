import pandas as pd

def analyze_data(filepath):
  df = pd.read_csv(filepath, sep=' ', header=None)
  class1 = df[0]
  class2 = df[1]
  counts = pd.crosstab(class1, class2, margins=True, margins_name="Total")
  return counts

filepath = "scripts/ANN/results/results.txt"
results = analyze_data(filepath)

if results is not None:
    print(results)