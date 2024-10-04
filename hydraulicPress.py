import pybullet as p
import pybullet_data
import os
import time
import matplotlib.pyplot as plt
import numpy as np

# Connect to PyBullet
physicsClient = p.connect(p.DIRECT)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

# Reset the simulation
p.resetSimulation(p.RESET_USE_DEFORMABLE_WORLD)
p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)
p.setGravity(0, 0, -10)
p.setTimeStep(0.001)
p.setPhysicsEngineParameter(sparseSdfVoxelSize=0.25)

# Load ground plane
planeId = p.loadURDF("plane.urdf", [0, 0, 0])

# Load a soft ball
ball_radius_m = 0.045
ballId = p.loadSoftBody(os.path.join(os.getcwd(), "assets/objects/softBall/ball.obj"), 
                         simFileName=os.path.join(os.getcwd(), "assets/objects/softBall/ball.vtk"), 
                         basePosition=[0, 0, ball_radius_m], 
                         scale=ball_radius_m, 
                         mass=0.05, 
                         useNeoHookean=1, 
                         NeoHookeanMu=600, 
                         NeoHookeanLambda=600, 
                         NeoHookeanDamping=0.001, 
                         useSelfCollision=1, 
                         frictionCoeff=0.5, 
                         collisionMargin=0.001)

# Hydraulic press parameters
initial_height = 0.1
final_height = 0.01
lowering_speed = 0.001

hydraulic_press_height = 0.5
hydraulic_press_id = p.loadURDF("cube.urdf", [0, 0, initial_height + hydraulic_press_height / 2], globalScaling=hydraulic_press_height)
p.changeDynamics(hydraulic_press_id, -1, mass=0)

press_frozen = False

# Plot parameters
press_heights = []
normal_forces = []

# Simulation loop
while not press_frozen:
    p.stepSimulation()
    
    # Get the current position of the hydraulic press
    plate_pos = p.getBasePositionAndOrientation(hydraulic_press_id)[0]
    
    # Lower the plate until it reaches the final height
    if plate_pos[2] > final_height + hydraulic_press_height / 2 and not press_frozen:
        new_height = plate_pos[2] - lowering_speed
        p.resetBasePositionAndOrientation(hydraulic_press_id, [0, 0, new_height], [0, 0, 0, 1])
    elif not press_frozen:
        # Freeze the hydraulic press by setting its mass to 0 (only once)
        p.changeDynamics(hydraulic_press_id, -1, mass=0)
        press_frozen = True  # Ensure this is done only once

    temp_total_force = []
    for i in range(100):
        p.stepSimulation()

        # Get the contact points between the hydraulic press and the ball
        contact_points = p.getContactPoints(bodyA=hydraulic_press_id, bodyB=ballId)
        
        total_force = 0
        # Sum the forces from all contact points
        for contact in contact_points:
            # contact[9] is the normal force at the contact point
            total_force += contact[9]

    if total_force > 0.001:
      temp_total_force.append(total_force)
      press_heights.append(plate_pos[2] - hydraulic_press_height / 2)  # Store current press height
      normal_forces.append(np.mean(temp_total_force))  # Store the corresponding normal force

    


# Create the plot
plt.figure()
plt.plot(normal_forces, press_heights)
plt.xlabel('Normal Force (N)')
plt.ylabel('Press Height (m)')
plt.title('Hydraulic Press Height vs. Normal Force')
plt.grid(True)
plt.show()

# Disconnect the simulation
p.disconnect()
