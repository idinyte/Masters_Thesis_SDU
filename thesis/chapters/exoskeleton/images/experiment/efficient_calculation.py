import numpy as np

experimental_force = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0, -2.943, -3.335, -3.728, -4.12, -4.513, -4.905, -5.297, -5.69, -6.082])
theoretical_force = np.array([0.0, -0.833, -1.667, -2.333, -3.167, -4.0, -4.833, -5.667, -6.333, -7.167, -8.0, -8.833, -9.667, -10.333, -11.167])


start_index = np.where(experimental_force == -2.943)[0][0]
efficiency_values = (experimental_force[start_index:] / theoretical_force[start_index:]) * 100

print(start_index)
average_efficiency = np.mean(efficiency_values)
print(efficiency_values)
print(average_efficiency)
