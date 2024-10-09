import pybullet as p
import pybullet_data
import os
import matplotlib.pyplot as plt
import numpy as np
import sys

# Connect to PyBullet
physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

# Reset the simulation
p.resetSimulation(p.RESET_USE_DEFORMABLE_WORLD)
p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)
p.setGravity(0, 0, -10)
p.setTimeStep(1/240)
p.resetDebugVisualizerCamera(cameraDistance=0.5, 
                              cameraYaw=0.0, 
                              cameraPitch=-5.0, 
                              cameraTargetPosition=[0, 0, 0])

# Load ground plane
planeId = p.loadURDF("plane.urdf", [0, 0, 0])

# Load a soft ball
density = 300
ball_radius_m = 0.045
volume = 4*np.pi*ball_radius_m**3/3
mass = density*volume

youngs_modulus = 1440 # kPa
poson_ratio = 0.45 # highly elastic materials

neo_mu = youngs_modulus / (2 * (1 + poson_ratio))
neo_lambda = youngs_modulus * poson_ratio / ((1 + poson_ratio)*(1 - 2 * poson_ratio))

ballId = p.loadSoftBody(os.path.join(os.getcwd(), "assets/objects/softBall/ball.obj"), 
                         simFileName=os.path.join(os.getcwd(), "assets/objects/softBall/ball.vtk"), 
                         basePosition=[0, 0, ball_radius_m], 
                         scale=ball_radius_m, 
                         mass=mass, 
                         useNeoHookean=1, 
                         NeoHookeanMu=neo_mu, 
                         NeoHookeanLambda=neo_lambda, 
                         NeoHookeanDamping=0.03, 
                         useSelfCollision=1, 
                         frictionCoeff=0.5, 
                         collisionMargin=0.001)

# Load the hydraulic press URDF
start_height = 0.3
hydraulic_press_id = p.loadURDF("assets/objects/hydraulic_press/press.urdf", [0, 0, start_height], useFixedBase=True)
p.enableJointForceTorqueSensor(hydraulic_press_id, 0, enableSensor=True)
p.setJointMotorControl2(hydraulic_press_id, 0, p.POSITION_CONTROL, force=0)
p.setJointMotorControl2(hydraulic_press_id, 0, p.VELOCITY_CONTROL, force=0)
for _ in range(100):
    p.stepSimulation()




# child_link_index = 1  # Assuming 'child_link' is index 1
# force = [0, 0, -5]  # Downward force of 5 N
# position = p.getBasePositionAndOrientation(child_link_id)[0]  # Current position



# _link_name_to_index = {p.getBodyInfo(hydraulic_press_id)[0].decode('UTF-8'):-1,}
        
# for _id in range(p.getNumJoints(hydraulic_press_id)):
# 	_name = p.getJointInfo(hydraulic_press_id, _id)[12].decode('UTF-8')
# 	_link_name_to_index[_name] = _id

# print(_link_name_to_index)

# Hydraulic press parameters
final_height=0.07
lowering_speed=-0.2
target_pos = -(start_height - 0.2 - final_height)

press_frozen = False

# Plot parameters
press_heights = []
normal_forces = []

# Simulation loop 
while not press_frozen:
    p.stepSimulation()
    p.setJointMotorControl2(hydraulic_press_id, 0, p.TORQUE_CONTROL, force=-500)
    # link_index = 0
    # p.applyExternalForce(
    #     objectUniqueId=hydraulic_press_id,
    #     linkIndex=0,
    #     forceObj=[0, 0, -5000],
    #     posObj=[0, 0, 0],
    #     flags=p.LINK_FRAME
    # )
    
    # joint_state = p.getJointState(hydraulic_press_id, 0)
    # joint_position = joint_state[0]
    # joint_velocity = joint_state[1]
    # reaction_force = joint_state[2]  # [Fx, Fy, Fz]
    # applied_torque = joint_state[3]

    # # Print the reaction force
    # print(f"Reaction Force: Fx={reaction_force[0]:.2f} N, Fy={reaction_force[1]:.2f} N, Fz={reaction_force[2]:.2f} N")

    # # Optionally, print joint position and velocity
    # print(f"Joint Position: {joint_position:.4f} m, Joint Velocity: {joint_velocity:.4f} m/s")
    # Set the joint position to the final height directly
   

    # Get the joint position of the prismatic joint (joint index 0)
    # joint_position = p.getJointState(hydraulic_press_id, 0)[0]
    # current_height = start_height - 0.21 + joint_position
    
    # # Get contact forces between press and ball
    # contact_points = p.getContactPoints(bodyA=hydraulic_press_id, bodyB=ballId, linkIndexA=0)
    # if len(contact_points) > 0:
    #     print(f"heights {current_height} {contact_points[0][5][2]}")
    
    # total_force = sum(contact[9] * np.linalg.norm(contact[8]) for contact in contact_points)

    # if current_height <= ball_radius_m*2:
    #     press_heights.append(current_height)  # Store current press height
    #     normal_forces.append(total_force)  # Store the corresponding normal force

    # # Check if the press is at the final position
    # if current_height - 0.001 <= final_height:
    #     # for i, contact in enumerate(contact_points):
    #     #     print(f"Contact {i}: Position: {contact[5]}, Positionb: {contact[6]}, Normal: {contact[8]}, Force: {contact[9]}")
    #     #  # Visualize contact forces
    #     # for contact in contact_points:
    #     #     contact_pos = contact[5]  # The position of the contact point
    #     #     contact_force = contact[9] * np.linalg.norm(contact[8])  # Magnitude of the contact force
            
    #     #     # Normalize the contact force vector
    #     #     force_direction = np.array(contact[8])  # Contact force vector
    #     #     force_magnitude = np.linalg.norm(force_direction)
    #     #     if force_magnitude > 0:
    #     #         force_direction /= force_magnitude  # Normalize

    #     #     # Scale the force vector for visualization
    #     #     scale_factor = 1  # Adjust this value for better visualization
    #     #     end_point = contact_pos + force_direction * scale_factor
    #     #     # Draw the force vector as a line
    #     #     p.addUserDebugLine([contact_pos[0], contact_pos[1], contact_pos[2]], end_point, lineColorRGB=[1, 0, 0], lifeTime=100)  # Red color for contact force
    #     #     p.stepSimulation()
    #     #     print([contact_pos[0], contact_pos[1], contact_pos[2]])
    #     #     print(end_point)
    #     #     while True:
    #     #         continue
    #     p.setJointMotorControl2(hydraulic_press_id, 0, p.VELOCITY_CONTROL, targetVelocity=0)
    #     press_frozen = True

# Create the plot
plt.figure()

# Plot normal force vs press height
plt.plot(normal_forces, press_heights, label='Normal Force (N)', color='blue')

plt.xlabel('Force (N)')
plt.ylabel('Press Height (m)')
plt.title('Hydraulic Press: Height vs Force')
plt.legend()
plt.grid(True)

# Show the plot
plt.show()

# Disconnect the simulation
p.disconnect()
