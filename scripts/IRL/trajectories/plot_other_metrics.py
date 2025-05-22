import pandas as pd
import matplotlib.pyplot as plt
import os

csv_path = os.path.join(os.getcwd(), 'scripts/IRL/logs_gail/progress.csv')

# Read full header
with open(csv_path, 'r') as f:
    header = f.readline().strip().split(',')

# Generator-related important columns
gen_cols = [
    'mean/disc/global_step',  # for X-axis
    'mean/gen/rollout/ep_rew_wrapped_mean',
    'mean/gen/train/actor_loss',
    'mean/gen/train/critic_loss',
    'mean/gen/train/ent_coef',
    'mean/gen/train/ent_coef_loss',
    'mean/gen/train/learning_rate'
]

# Match available columns
usecols = [col for col in header if col.strip() in gen_cols]

# Load data
df = pd.read_csv(csv_path, usecols=usecols)
df.dropna(inplace=True)
df.sort_values(by='mean/disc/global_step', inplace=True)

# Plot
plt.figure(figsize=(15, 12))

plt.subplot(3, 2, 1)
plt.plot(df['mean/disc/global_step'], df['mean/gen/rollout/ep_rew_wrapped_mean'])
plt.title('Episode Reward')
plt.xlabel('Step')
plt.ylabel('Reward')

plt.subplot(3, 2, 2)
plt.plot(df['mean/disc/global_step'], df['mean/gen/train/actor_loss'])
plt.title('Actor Loss')
plt.xlabel('Step')
plt.ylabel('Loss')

plt.subplot(3, 2, 3)
plt.plot(df['mean/disc/global_step'], df['mean/gen/train/critic_loss'])
plt.title('Critic Loss')
plt.xlabel('Step')
plt.ylabel('Loss')

plt.subplot(3, 2, 4)
plt.plot(df['mean/disc/global_step'], df['mean/gen/train/ent_coef'])
plt.title('Entropy Coefficient')
plt.xlabel('Step')
plt.ylabel('Ent Coef')

plt.subplot(3, 2, 5)
plt.plot(df['mean/disc/global_step'], df['mean/gen/train/ent_coef_loss'])
plt.title('Entropy Coef Loss')
plt.xlabel('Step')
plt.ylabel('Loss')

plt.subplot(3, 2, 6)
plt.plot(df['mean/disc/global_step'], df['mean/gen/train/learning_rate'])
plt.title('Learning Rate')
plt.xlabel('Step')
plt.ylabel('LR')

plt.tight_layout()
plt.show()
