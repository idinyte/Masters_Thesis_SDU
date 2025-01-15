import matplotlib.pyplot as plt

file_path = "scripts/ANN/data3/train_history.txt"
x = []
y = []

with open(file_path, "r") as file:
    for line in file:
        if line.strip():
            values = line.split()
            x.append(int(values[0]))
            y.append(float(values[1]))


plt.figure(figsize=(12, 6))
plt.plot(x, y, color='b', label='Loss')
plt.xlabel('Iteration')
plt.ylabel('Loss')
plt.title('Neural Network: Loss vs Iteration')
plt.legend()
plt.grid(True)
plt.show()
