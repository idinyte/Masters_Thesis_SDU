from ANN import ANN
import torch
from torch.utils.data import DataLoader, Dataset

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

test_file = "scripts/ANN/data2/test.txt"
test_dataset = BallDataset(test_file)

batch_size = 1
test_loader = DataLoader(test_dataset, batch_size=batch_size, shuffle=False)


input_dim = 8
output_dim = 4
model = ANN(input_dim=input_dim, output_dim=output_dim)
weights_path = 'scripts/ANN/ann_weights_architecture_8_32_4_epochs_5000_acc_996.pth'
model.load_state_dict(torch.load(weights_path))
model.eval()

model.eval()
correct = 0
total = 0
with torch.no_grad():
    for batch_data, batch_labels in test_loader:
        outputs = model(batch_data)
        _, predicted = torch.max(outputs, 1)
        total += batch_labels.size(0)
        correct += (predicted == batch_labels).sum().item()
        print(batch_data)
        break

accuracy = 100 * correct / total
print(f'Accuracy on test set: {accuracy:.2f}%')