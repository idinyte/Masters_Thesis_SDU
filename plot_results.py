import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# Load data
data = np.loadtxt('results.txt')
l_forces = data[:, 0]
r_forces = data[:, 1]
target_lc = data[:, 2]
target_rc = data[:, 3]
present_lc = data[:, 4]
present_rc = data[:, 5]
time = data[:, 6]

# Normalize time to start from 0
time -= time[0]

# Create figure and subplots
fig, axs = plt.subplots(2, 2, figsize=(19.2, 5.4), dpi=100)
lines = []

# Left force
axs[0, 0].set_title("Left Force vs Time")
axs[0, 0].set_xlim(time[0], time[-1])
axs[0, 0].set_ylim(min(l_forces), max(l_forces))
line1, = axs[0, 0].plot([], [], lw=2)
lines.append(line1)

# Right force
axs[0, 1].set_title("Right Force vs Time")
axs[0, 1].set_xlim(time[0], time[-1])
axs[0, 1].set_ylim(min(r_forces), max(r_forces))
line2, = axs[0, 1].plot([], [], lw=2)
lines.append(line2)

# Left currents
axs[1, 0].set_title("Left Pad Currents vs Time")
axs[1, 0].set_xlim(time[0], time[-1])
axs[1, 0].set_ylim(min(min(target_lc), min(present_lc)), max(max(target_lc), max(present_lc)))
line3, = axs[1, 0].plot([], [], label='Target')
line4, = axs[1, 0].plot([], [], label='Present')
lines += [line3, line4]
axs[1, 0].legend()

# Right currents
axs[1, 1].set_title("Right Pad Currents vs Time")
axs[1, 1].set_xlim(time[0], time[-1])
axs[1, 1].set_ylim(min(min(target_rc), min(present_rc)), max(max(target_rc), max(present_rc)))
line5, = axs[1, 1].plot([], [], label='Target')
line6, = axs[1, 1].plot([], [], label='Present')
lines += [line5, line6]
axs[1, 1].legend()

# Animation update function
def update(i):
    lines[0].set_data(time[:i], l_forces[:i])
    lines[1].set_data(time[:i], r_forces[:i])
    lines[2].set_data(time[:i], target_lc[:i])
    lines[3].set_data(time[:i], present_lc[:i])
    lines[4].set_data(time[:i], target_rc[:i])
    lines[5].set_data(time[:i], present_rc[:i])
    return lines

def get_results_interval():
  data = np.loadtxt('results_exo1.txt')
  time = data[:, -1]
  time_diffs = np.diff(time)
  mean_interval_s = np.mean(time_diffs)
  interval_ms = mean_interval_s * 1000  # convert to milliseconds

  print(f"Recommended interval: {interval_ms:.2f} ms")
  return interval_ms

interval_ms = get_results_interval()
fps = 1000 / interval_ms

ani = FuncAnimation(fig, update, frames=len(time), interval=interval_ms, blit=True)
plt.tight_layout()

save_animation = True
if save_animation: 
  ani.save('animation.mp4', writer='ffmpeg', fps=fps, dpi=100) 
else:
  plt.show()
