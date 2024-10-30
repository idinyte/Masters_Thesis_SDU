import os
import torch
import torch.nn as nn
import torch.optim as optim
from torch.utils.data import DataLoader, Dataset, random_split
from ANN import ANN

class BallDataset(Dataset):
    def __init__(self, file_path):
        self.data = []
        self.labels = []

        with open(file_path, 'r') as file:
            for line in file:
                parts = line.strip().split()
                ball_name = parts[0]
                data = [float(x) for x in parts[1:]]
                self.data.append(data)
                self.labels.append(int(ball_name) - 1)

    def __len__(self):
        return len(self.data)

    def __getitem__(self, idx):
        return torch.tensor(self.data[idx], dtype=torch.float32), torch.tensor(self.labels[idx])

data_file_path = os.path.join(os.path.dirname(__file__), 'data', 'data.txt')
train_file_path = os.path.join(os.path.dirname(__file__), 'data', 'train.txt')
test_file_path = os.path.join(os.path.dirname(__file__), 'data', 'test.txt')
train_history = os.path.join(os.path.dirname(__file__), 'data', 'train_history.txt')

# Split dataset into train and test sets and save to separate files
dataset = BallDataset(data_file_path)
train_size = int(0.8 * len(dataset))
test_size = len(dataset) - train_size
train_dataset, test_dataset = random_split(dataset, [train_size, test_size])

with open(train_file_path, 'w') as train_file:
    for data, label in train_dataset:
        line = f"{label + 1} " + " ".join(map(str, data.tolist())) + "\n"
        train_file.write(line)

with open(test_file_path, 'w') as test_file:
    for data, label in test_dataset:
        line = f"{label + 1} " + " ".join(map(str, data.tolist())) + "\n"
        test_file.write(line)

input_dim = 8   # 4 pairs of (diameter, force)
output_dim = 4  # 4 types of balls
num_epochs = 5000
batch_size = 32
learning_rate = 0.001
model_archtecture = "8_64_4"

train_loader = DataLoader(train_dataset, batch_size=batch_size, shuffle=True)
test_loader = DataLoader(test_dataset, batch_size=batch_size, shuffle=False)

model = ANN(input_dim=input_dim, output_dim=output_dim)
criterion = nn.CrossEntropyLoss()
optimizer = optim.Adam(model.parameters(), lr=learning_rate)

# Continue training if checkpoint exists
checkpoint_path = os.path.join(os.path.dirname(__file__), 'checkpoint.pth')
if os.path.isfile(checkpoint_path):
    checkpoint = torch.load(checkpoint_path)
    model.load_state_dict(checkpoint['model_state_dict'])
    optimizer.load_state_dict(checkpoint['optimizer_state_dict'])
    start_epoch = checkpoint['epoch'] + 1
else:
    start_epoch = 1
    with open(train_history, 'w') as _:
        pass

# Training loop
try:
    for epoch in range(start_epoch, num_epochs + 1):
        model.train()
        for batch_data, batch_labels in train_loader:
            optimizer.zero_grad()
            outputs = model(batch_data)
            loss = criterion(outputs, batch_labels)
            loss.backward()
            optimizer.step()
        
        with open(train_history, 'a') as file:
            file.write(f"{epoch} {loss.item():.4f}\n")

        if epoch % 10 == 0:
            print(f'Epoch [{epoch}/{num_epochs}], Loss: {loss.item():.4f}')

except KeyboardInterrupt:
    print("Training interrupted. Saving model...")

finally:
    # Save model and optimizer state
    torch.save({
        'epoch': epoch,
        'model_state_dict': model.state_dict(),
        'optimizer_state_dict': optimizer.state_dict(),
    }, checkpoint_path)
    print(f'Model and optimizer state saved to {checkpoint_path}')

# Evaluation
model.eval()
correct = 0
total = 0
with torch.no_grad():
    for batch_data, batch_labels in test_loader:
        outputs = model(batch_data)
        _, predicted = torch.max(outputs, 1)
        total += batch_labels.size(0)
        correct += (predicted == batch_labels).sum().item()

accuracy = 100 * correct / total
print(f'Accuracy on test set: {accuracy:.2f}%')

# Save final model after training
torch.save(model, os.path.join(os.path.dirname(__file__), f"ann_weights_architecture_{model_archtecture}_epochs_{num_epochs}_acc_{int(accuracy * 10)}.pth"))
