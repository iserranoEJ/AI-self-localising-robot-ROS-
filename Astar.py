#!/usr/bin/env python
from __future__ import print_function

from math import floor
from PotentialFields import *
from Grid import *
from PriorityQueue import *

# some parameters: the bottom left of the square
GRID_CORNER_X = -0.875
GRID_CORNER_Y = -6.0
# the sidelength of the whole square (in metres)
GRID_SIDELENGTH = 6
# and the number of grid squares to divide this into
NUM_GRID_SQUARES = 24



# This class houses all the A* functionality except for the heuristic, which is handled by the
# potential fields class. It creates a grid, and finds an optimal path through that grid.
class Astar:


    # this method instantiates a grid and marks inaccessible those squares corresponding
    # to obstacles, according to the list it is given.    
    @staticmethod
    def createGrid(obst_list):

	    grid = Grid(NUM_GRID_SQUARES, GRID_SIDELENGTH, GRID_CORNER_X, GRID_CORNER_Y)
	    
	    for obst in obst_list:
	        grid.setInaccessible(obst.x, obst.y)

	    return grid


    
    # computes the transition cost between two nodes; for orthogonal movements, that's 1, 
    # but diagonally that's root 2, approximately 1.4
    @staticmethod
    def transitionCost(nodeA, nodeB):
	    if nodeA.x == nodeB.x or nodeA.y == nodeB.y: 
	        return 1
	    else:
	        return 1.4


    # Perform an A* search generating a route from the start node to
    # the goal node. Returns a path from the start node to the goal node.
    # Assumes that the goal is always reachable from the start node.
    @staticmethod
    def a_star_search(start_x, start_y, goal_x, goal_y, obst_list):

        # initialise the grid and the start/goal nodes
        grid = Astar.createGrid(obst_list)
        startNode = grid.gridsquare(start_x, start_y)
        goalNode = grid.gridsquare(goal_x, goal_y)

        # initialise the open and closed lists
        openList = PriorityQueue(startNode)
        closedList = []
        currentNode = None

        # main loop: continue until we find the goal or have explored the whole map
        while openList != [] and currentNode != goalNode:
	    
            currentNode = openList.dequeue()
            closedList.append(currentNode)

            # for each reachable node
            for succ in grid.reachable(currentNode):

                transCost = Astar.transitionCost(currentNode, succ)

                # if it hasn't been seen before add it to the list
                if succ not in closedList and not openList.contains(succ):
                    
                    # mark info on the successor
                    succ.predecessor = currentNode
                    succ.pathCost = currentNode.pathCost + transCost
                    
                    # calculate the heuristic value using potential fields
                    sX, sY = grid.cartesian(succ.x, succ.y)
                    gX, gY = grid.cartesian(goalNode.x, goalNode.y)
                    succ.hDistance = PotentialFields.heuristic(sX, sY, gX, gY, obst_list)
                    
                    # and put it on the open list
                    openList.enqueue(succ)

                # or if we need to, lower the cost to reach it
                elif openList.contains(succ) and succ.pathCost > currentNode.pathCost + transCost:
                    succ.pathCost = currentNode.pathCost + transCost
                    openList.decreaseKey(succ)

        # once the goal is found, reconstruct the route taken
        path = []
        pointer = goalNode
        # by continuously appending the predecessor
        while pointer != startNode:	
            path.append(pointer)
            pointer = pointer.predecessor
        # and finally start node
        path.append(pointer)

        # print out for the viewer
        grid.printPath(path)

        # take cartesian points instead of node objects
        waypoints = [grid.cartesian(n.x, n.y) for n in path]
        # start at the start
        waypoints.reverse()
        # not interested in the turtlebots current position
        waypoints.remove(waypoints[0])
        # and at the end, go exactly to the goal
        waypoints.append((goal_x, goal_y))
        
        return waypoints


