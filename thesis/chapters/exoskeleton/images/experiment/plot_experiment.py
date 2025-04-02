import matplotlib.pyplot as plt
import numpy as np

# X-axis values in different units
target_current = np.array([0, -1, -2, -3, -4, -5, -6, -7, -8, -9, -10, -11, -12, -13, -14])
current_mA = np.array([0.0, -2.69, -5.38, -8.07, -10.76, -13.45, -16.14, -18.83, -21.52, -24.21, -26.9, -29.59, -32.28, -34.97, -37.66])
torque_Nm = np.array([0.0, -0.005, -0.01, -0.014, -0.019, -0.024, -0.029, -0.034, -0.038, -0.043, -0.048, -0.053, -0.058, -0.062, -0.067])

# Y-axis values
experimental_force = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0, -2.943, -3.335, -3.728, -4.12, -4.513, -4.905, -5.297, -5.69, -6.082])
theoretical_force = np.array([0.0, -0.833, -1.667, -2.333, -3.167, -4.0, -4.833, -5.667, -6.333, -7.167, -8.0, -8.833, -9.667, -10.333, -11.167])

fig, ax1 = plt.subplots()
ax2 = ax1.twiny()
ax3 = ax1.twiny()

# Offset the third x-axis to prevent overlap
ax3.spines['top'].set_position(('outward', 40))

# Plot experimental and theoretical data
ax1.plot(target_current, experimental_force, 'ro-', label='Experimental')
ax1.plot(target_current, theoretical_force, 'bo--', label='Theoretical')

# Set axis labels
ax1.set_xlabel("Target Current Motor Setting")
ax1.set_ylabel("Force (N)")
ax2.set_xlabel("Current (mA)")
ax3.set_xlabel("Torque (Nm)")

# Invert the x-axis to read left to right
ax1.invert_xaxis()
ax1.invert_yaxis()
ax2.invert_xaxis()
ax2.invert_yaxis()
ax3.invert_xaxis()
ax3.invert_yaxis()

# Match the scale of additional x-axes to the primary x-axis
ax2.set_xlim(ax1.get_xlim())
ax3.set_xlim(ax1.get_xlim())
ax2.set_xticks(target_current)
ax2.set_xticklabels(current_mA)
ax3.set_xticks(target_current)
ax3.set_xticklabels(torque_Nm)

# Add legend
ax1.legend()
plt.title("Experimental vs Theoretical Force")
plt.show()