import numpy as np
from scipy.stats import chi2_contingency, kruskal

successes_grouped_in_10 = {1: [], 2: [], 3: [], 4: []}

with open('scripts/ANN/results/results.txt', 'r') as file:
    lines = file.readlines()


for i in range(0, len(lines), 1):
    group = lines[i:i+1]
    class_counts = {1: 0, 2: 0, 3: 0, 4: 0}
    

    for line in group:
        parts = line.split()
        class_id = int(parts[0])
        result = float(parts[1])

        if result == 2:
            class_counts[class_id] += 1

    successes_grouped_in_10[class_id].append(class_counts[class_id])

# Print results
for class_id, counts in successes_grouped_in_10.items():
    print(f"Class {class_id}: {counts}")
    
stat, p_value = kruskal(
  successes_grouped_in_10[2],
  successes_grouped_in_10[3],
  successes_grouped_in_10[4]
)

print(f"Kruskal-Wallis Test Statistic: {stat}")
print(f"P-Value: {p_value}")