#!/usr/bin/env python

import math

# Parameters for the potential fields algorithm
KATT = 0.5
KREP = 0.4
Q = 0.35	# Radius of action around objects

class PotentialFields:

    # return the heuristic for a given point (pX, pY) if the goal is situated at (goalX, goalY)
    @staticmethod
    def heuristic(pX, pY, goalX, goalY, obst_list):

        # start with attractive only
        fX, fY = PotentialFields.attractive_force(pX, pY, goalX, goalY)

        # for each obstacle, add the repulsive force
        for obst in obst_list:
            rX, rY = PotentialFields.repulsive_force(pX, pY, obst)
            fX, fY = fX + rX, fY + rY

        total_force = math.sqrt(fX**2 + fY**2)

        return total_force
    
    # return the attractive force from the robot towards the goal.
    @staticmethod
    def attractive_force(pX, pY, goalX, goalY):
        return (KATT*(goalX - pX), KATT*(goalY - pY))

    # return the repulsive force caused by every object to the robot.
    @staticmethod
    def repulsive_force(pX, pY, obst):
        
        # start with the actual error
        fX, fY = pX - obst.x, pY - obst.y

        # find Euclidean distance    
        D = math.sqrt(fX**2 + fY**2)

        if D > Q:
            return (0, 0)
        
        # find scaling factor according to eqn
        scale_factor = KREP * (1/(D**3)) * ((1/D) - (1/Q))

        return (fX * scale_factor, fY * scale_factor)
