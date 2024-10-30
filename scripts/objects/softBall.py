import pybullet as p
import os
import numpy as np
import time

class SoftBall():
    def __init__(self, youngs_modulus, poisson_ration, radius, density = 400, name = "", robot_base_pos = [0, 0, 0]):
        self.youngs_modulus = youngs_modulus
        self.poisson_ration = poisson_ration
        self.radius = radius
        self.density = density
        self.id = None
        self.name = name
        self.aabb_volume = (radius*2)**3
        self.robot_base_pos = robot_base_pos
        self.dead = False

    def instantiate(self, base_position):
        # ball model optained from https://github.com/bulletphysics/bullet3/blob/master/data/ball.obj
        volume = 4*np.pi*self.radius**3/3
        mass = self.density*volume
        
        neo_mu = self.youngs_modulus / (2 * (1 + self.poisson_ration))
        neo_lambda = self.youngs_modulus * self.poisson_ration / ((1 + self.poisson_ration)*(1 - 2 * self.poisson_ration))
        
        self.id = p.loadSoftBody(os.path.join(os.getcwd(), "assets/objects/softBall/ball_regular.obj"), 
                          simFileName=os.path.join(os.getcwd(), "assets/objects/softBall/ball_regular.vtk"), 
                          basePosition=base_position, 
                          mass=mass, 
                          useNeoHookean=1, 
                          NeoHookeanMu=neo_mu, 
                          NeoHookeanLambda=neo_lambda,
                          NeoHookeanDamping=0.001,
                          useSelfCollision=1,
                          repulsionStiffness=800,
                          frictionCoeff=1, 
                          collisionMargin=0.0001)
        
        p.stepSimulation()

        return self.id
    
    def calculate_volume(self):
        aabb_min, aabb_max = p.getAABB(self.id)
        width = aabb_max[0] - aabb_min[0]
        height = aabb_max[1] - aabb_min[1]
        thickness = aabb_max[2] - aabb_min[2]

        volume = width * height * thickness

        return volume
    
    def euclidean_distance(self, pos1, pos2):
        return np.linalg.norm(np.array(pos1) - np.array(pos2))
    
    def update_pos(self):
        self.ball_position = p.getBasePositionAndOrientation(self.id)[0]

    def is_ball_within_robot_reach(self):
        radius_min = 0.2
        radius_max = 1.5
        distance = self.euclidean_distance(self.robot_base_pos, self.ball_position)

        return radius_min <= distance <= radius_max
    
    def should_self_destruct(self):
        if self.dead:
            return False

        self.update_pos()

        # The soft body simulation has exploded
        if self.calculate_volume() > 2*self.aabb_volume:
            print("Self destruct: ball exploded!")
            self.dead = True
        
        # Unreachable
        if not self.is_ball_within_robot_reach():
            print("Self destruct: ball too far!")
            self.dead = True
        
        # Reached ground
        if self.ball_position[2] <= self.radius:
            print("Self destruct: ball on ground!")
            self.dead = True
        
        return self.dead


