import pandas as pd
import matplotlib.pyplot as plt
import os

csv_path = os.path.join(os.getcwd(), 'scripts/IRL/logs_gail/progress.csv')

# Preview columns first
with open(csv_path, 'r') as f:
    header = f.readline().strip().split(',')

# Columns of interest (strip to match actual names)
important_cols = ['mean/disc/global_step',
                  'mean/disc/disc_loss',
                  'mean/disc/disc_acc_expert',
                  'mean/disc/disc_acc_gen',
                  'mean/disc/disc_proportion_expert_pred',
                  'mean/disc/disc_entropy']

# Match actual column names from file
usecols = [col for col in header if col.strip() in important_cols]

# Load only those columns
df = pd.read_csv(csv_path, usecols=usecols)

# Drop rows with any NaNs
df.dropna(inplace=True)

# Sort by step in case it's unordered
df.sort_values(by='mean/disc/global_step', inplace=True)

# Plotting
plt.figure(figsize=(15, 10))

plt.subplot(2, 2, 1)
plt.plot(df['mean/disc/global_step'], df['mean/disc/disc_loss'])
plt.title('Discriminator Loss')
plt.xlabel('Step')
plt.ylabel('Loss')

plt.subplot(2, 2, 2)
plt.plot(df['mean/disc/global_step'], df['mean/disc/disc_acc_expert'], label='Expert')
plt.plot(df['mean/disc/global_step'], df['mean/disc/disc_acc_gen'], label='Generator')
plt.title('Discriminator Accuracy')
plt.xlabel('Step')
plt.ylabel('Accuracy')
plt.legend()

plt.subplot(2, 2, 3)
plt.plot(df['mean/disc/global_step'], df['mean/disc/disc_proportion_expert_pred'])
plt.title('Predicted as Expert (Proportion)')
plt.xlabel('Step')
plt.ylabel('Proportion')

plt.subplot(2, 2, 4)
plt.plot(df['mean/disc/global_step'], df['mean/disc/disc_entropy'])
plt.title('Discriminator Entropy')
plt.xlabel('Step')
plt.ylabel('Entropy')

plt.tight_layout()
plt.show()
