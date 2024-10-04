import pybullet as p
import os

class SoftBall():
    def __init__(self, stiffness, base_position, radius):
        self.stiffness = stiffness
        self.base_position = base_position
        self.radius = radius
        self.id = None

    def instantiate(self):
        # ball model optained from https://github.com/bulletphysics/bullet3/blob/master/data/ball.obj
        self.id = p.loadSoftBody(os.path.join(os.getcwd(), "assets/objects/softBall/ball.obj"), simFileName = os.path.join(os.getcwd(), "assets/objects/softBall/ball.vtk"), basePosition = self.base_position, scale = self.radius, mass = 4, useNeoHookean = 1, NeoHookeanMu = 400, NeoHookeanLambda = self.stiffness, NeoHookeanDamping = 0.001, useSelfCollision = 1, frictionCoeff = .5, collisionMargin = 0.001)

        return self.id


