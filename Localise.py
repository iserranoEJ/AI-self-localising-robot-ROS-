#!/usr/bin/env python
from __future__ import print_function
import time
from math import pi

# enum for the different colours scanned
COLOUR_CODE = ["Background", "White", "Grey"]
# an array representing the actual colours of the grids
LINE_COLOURS = [2, 2, 1, 2, 2, 2, 1, 1, 2, 2, 
				1, 2, 2, 2, 1, 1, 2, 2, 2, 1, 
				1, 2, 2, 1]
				
# some cartesian information
BOX_SIZE = 0.1
LINE_START_X = 1.9
LINE_START_Y = -2.7

# the likelihood that each move is successful
MOVE_SUCCESS_RATE = 0.9
MOVE_FAILURE_RATE = 1 - MOVE_SUCCESS_RATE

# the required confidence threshold
CONFIDENCE_REQUIRED = 0.9



# This class houses all the localisation techniques. The main functionality is provided by the
# Localise method.
class BayesLoc:


    # Update the current belief according to a new colour which has been sensed
    @staticmethod
    def bayes_update_by_sensor(old_bayes, colour):
	    
	    # create a new array to hold the updated beliefs
	    new_bayes = [0 for _ in range(len(old_bayes))]
	    
	    # and update each one
	    for i in range(len(old_bayes)):
		    if colour == LINE_COLOURS[i]:   # if it matches the real colour
			    new_bayes[i] = old_bayes[i] * MOVE_SUCCESS_RATE
		    else:                           # or if it doesn't match
			    new_bayes[i] = old_bayes[i] * MOVE_FAILURE_RATE

        # normalise the array; first compute the sum
	    eta = 0;
	    for x in new_bayes:
		    eta += x
	    
	    # and then divide each entry by it
	    for i in range(len(new_bayes)):
		    new_bayes[i] = new_bayes[i]/eta

	    return new_bayes


    # Update the current belief according to the movement we've made (prediction phase)
    @staticmethod
    def bayes_update_by_move(old_bayes, movement):
	    
	    # create a new array to hold the updated beliefs
	    new_bayes = [0 for _ in range(len(old_bayes))]

        # and update each one
	    for i in range(len(old_bayes)):
	        # if it couldn't be the result of the move (end of the line)
		    if i - movement < 0 or i - movement >= len(old_bayes):
			    new_bayes[i] = MOVE_FAILURE_RATE * old_bayes[i]
		    # or if it could be
		    else:
			    new_bayes[i] = MOVE_FAILURE_RATE * old_bayes[i] \
						    + MOVE_SUCCESS_RATE * old_bayes[i - movement]

	    return new_bayes


    # returns the grid square which we currently believe to be the most likely
    # this is done by simply returning the index of the maximum value
    @staticmethod
    def best_guess(array):
	    val = -1
	    ind = -1	
	    for i in range(len(array)):
		    if array[i] >= val:
			    val = array[i]
			    ind = i
	    return ind


    # Returns the position of the centre of the square in cartesian (x, y)
    @staticmethod
    def cartesian_coords(grid_index):
	    return (LINE_START_X, LINE_START_Y - 0.1*grid_index)


    # returns True if we should move forward, False otherwise
    @staticmethod
    def best_direction(array, current_direction, delta):
        
        # take the max
        maxval = 0
        for x in array:
            if x > maxval:
                maxval = x

        # if we're not confident, check the delta only
        if maxval < 0.3:
            if delta >= 10:
                return False # move backward
            if delta <= -10:
                return True # move forward
            return current_direction

        # take the nearest index to either end having this value
        lowind = len(array)
        highind = 0
        for i in range(len(array)):
            if array[i] == maxval:
                lowind = min(lowind, i)
                highind = max(highind, i)


        # check if we could be dangerously close to either end
        if lowind < 4 and highind >= len(array) - 4:
            return lowind > len(array) - highind - 1

        # or if we are only close to one end
        if lowind < 4:
            return False # move backward
        if highind >= len(array) - 4:
            return True # move forward

        # if close to neither, keep same heading
        return current_direction


    # localise the robot. This explores the line maintaining the current belief after each step
    # until it is very confident. It returns the cartesian location of where it thinks it is.
    @staticmethod
    def localise(nav, sensors):

        # initialise the array of probabilities for each square
        n = len(LINE_COLOURS)
        bayes = [1.0 / n for _ in range(n)]

        # initial print out
        BayesLoc.print_bayes(bayes, COLOUR_CODE[sensors.read_colour()])

        # read the first colour and update
        bayes = BayesLoc.bayes_update_by_sensor(bayes, sensors.read_colour())
	    
	    # loop until we've reached the required confidence, and then return
        move_fwd = True # current direction
        delta = 0 # balance of total squares forward or backward that we've moved so far
        while True:
		    
            if sensors.read_colour() == 0: #background
                move_fwd = not move_fwd
            else:
			    # determine whether we have reached the confidence threshold
                guess = BayesLoc.best_guess(bayes)
                if bayes[guess] > CONFIDENCE_REQUIRED:
                    x, y = BayesLoc.cartesian_coords(guess)
                    print("I am at square {0} with prob {1}!".format(guess, 
				                                                     round(bayes[guess], 2)))
                    return BayesLoc.cartesian_coords(guess)
			    
			    # determine best direction to drive
                move_fwd = BayesLoc.best_direction(bayes, move_fwd, delta)
		    
		    # set movement and speed according to our direction
            if move_fwd:
                lin = BOX_SIZE
                move = -1
            else:
                lin = -BOX_SIZE
                move = 1

		    # update delta
            delta += move

		    # try to keep pointing straight
            nav.turnTo(sensors, pi/2)
            nav.forward(2, lin, sleep_factor=0)
            nav.change_speed(0, 0)

		    # adjust probabilities (prediction phase)
            bayes = BayesLoc.bayes_update_by_move(bayes, move)

		    # adjust probabilities (correction phase)
            bayes = BayesLoc.bayes_update_by_sensor(bayes, sensors.read_colour())
		    
		    # and print out the updated beliefs
            BayesLoc.print_bayes(bayes, COLOUR_CODE[sensors.read_colour()])
		    
    
	    
    # Print out an array representing the current belief. Shows the actual floats as a single
    # digit 0-9 for ease of display
    @staticmethod
    def print_bayes(bayes, colour):

        print(colour, end=":")
        for _ in range(7 - len(colour)):
            print(" ", end="")

        print("[ ", end="")
        for x in bayes:
            print("{0} ".format(int(10*x)), end="")
        print("]")

