import pybullet as p

class Ball():
    def __init__(self, stiffness, base_position, radius):
        self.stiffness = stiffness
        self.position = base_position
        self.radius = radius

    def instantiate(self):
        # Load a soft body mesh from a file (replace 'soft_ball.obj' with your mesh file)
        # Alternatively, create a simple sphere as a placeholder
        self.body_id = p.loadURDF("sphere.urdf", basePosition=self.position, globalScaling=self.radius)

        # Set soft body parameters
        p.changeDynamics(self.body_id, -1, lateralFriction=0.5, spinningFriction=0.5, 
                         rollingFriction=0.5, restitution=0.6)

        # Configure soft body properties
        p.changeVisualShape(self.body_id, -1, rgbaColor=[1, 0, 0, 1])  # Red color
        p.changeDynamics(self.body_id, -1, linearStiffness=self.stiffness)


