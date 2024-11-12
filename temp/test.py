import numpy as np

def continuous_rotation_quaternions(angle_increment, steps):
    Q_y_axis = np.array([np.sqrt(2) / 2, 0, np.sqrt(2) / 2, 0])  # [w, x, y, z]
    
    quaternions = []
    
    for i in range(steps):
        theta = i * angle_increment
        Q_z_axis = np.array([np.cos(theta / 2), 0, 0, np.sin(theta / 2)])  # [w, x, y, z]
        
        current_orientation = quaternion_multiply(Q_z_axis, Q_y_axis)  # Multiply Z-axis by Y-axis
        
        quaternions.append(current_orientation)
    
    return quaternions

def quaternion_multiply(q1, q2):
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    return np.array([
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2
    ])

angle_increment = np.pi / 8
steps = 16

target_orientations = continuous_rotation_quaternions(angle_increment, steps)

for i, quat in enumerate(target_orientations):
    print(f"{list(quat)}")
