from ANN import ANN
import torch

path = "scripts/ANN/ann_weights_architecture_8_32_4_epochs_5000_acc_901"
model = torch.load(f"{path}.pth")
model.eval()

torch.save(model.state_dict(), f"{path}_state_dict.pth")