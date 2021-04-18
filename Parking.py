from math import pi
import time

# Parking parameters

START_EDGE = 0.6 # Approx expected distance to left corner
STOP_EDGE = 0.7 # Approx expected distance to right corner
EDGE_THRESHOLD = 0.1 # Accuracy threshold
PARKING_SPACE_LENGTH = 1.1 # Approximate length of the parking space

PARK_DIST = 0.3 # How close we want to get to the end
PARK_SPEED = 0.1 # How quickly we drive
TURN_SPEED = 0.25 # How quickly we turn

KP = 0.8 # Proportional gain
KD = 0.3 # Derivative gain



# This class encapsulates all parking behaviour, and wraps it in a high level interface, with
# methods such as drive in, drive out, and identifying the angle needed.
class Parking:


    # returns the required heading angle, by scanning left and right and averaging those
    # two angles. 
    @staticmethod
    def detect_parking_angle(sensors, nav):
        
        # turn to face straight left
        nav.turnTo(sensors, pi/2)

        # rotate right until START EDGE
        dist = sensors.full_scan.ranges[0]
        starting_phi = sensors.phi
        min_dist = dist # recording a closest point
        while dist == float("inf") or dist < 1.0:
            nav.change_speed(0, -TURN_SPEED)
            if dist < min_dist:
                starting_phi = sensors.phi
                min_dist = dist
            dist = sensors.full_scan.ranges[0]
                
        nav.change_speed(0, 0)

        print("LEFT: {0}".format(starting_phi))

        # rotate through back panel
        while abs(sensors.full_scan.ranges[0] - PARKING_SPACE_LENGTH) \
                     > PARKING_SPACE_LENGTH*EDGE_THRESHOLD:
              nav.change_speed(0, -TURN_SPEED)
        nav.change_speed(0, 0)
        
        # rotate right until STOP EDGE
        dist = sensors.full_scan.ranges[0]
        stopping_phi = sensors.phi
        min_dist = dist # recording a closest point
        while dist != float("inf"):
            nav.change_speed(0, -TURN_SPEED)
            if dist < min_dist:
                stopping_phi = sensors.phi
                min_dist = dist
            dist = sensors.full_scan.ranges[0]

        nav.change_speed(0, 0)

        print("RIGHT: {0}".format(stopping_phi))

        # compute average heading
        heading = (starting_phi + stopping_phi) / 2.0
        
        print("MIDDLE: {0}".format(heading))

        return heading


    # Compute the difference between distances measured to the left and right,
    # i.e. how far off central it is in the parking space.
    @staticmethod
    def sideways_error(scan):

        left_ind = len(scan.ranges) // 4
        right_ind = 3*len(scan.ranges) // 4

        if scan.ranges[left_ind] == float("inf") \
                or scan.ranges[right_ind] == float("inf"):
            return 0

        return scan.ranges[left_ind] - scan.ranges[right_ind]


    # the first step of parking: drive into the space
    @staticmethod
    def drive_in(sensors, nav, heading, dist_threshold):
        
        # turn to the right angle
        nav.turnTo(sensors, heading)

        print("Driving forward")

        # take an initial reading of time and error
        prev_err = Parking.sideways_error(sensors.full_scan)
        prev_time = time.time()

        # navigate using PD control until we are near enough to the back of the space
        while sensors.full_scan.ranges[0] > dist_threshold:
            
            # take a new snapshot of time and error
            error = Parking.sideways_error(sensors.full_scan)
            deriv = (error - prev_err) / (time.time() - prev_time)
            # and use that for the respective gains
            a = KP*error + KD*deriv
            
            # select either fast or slow speed depending on distance
            if sensors.full_scan.ranges[0] > 0.4:
                l = PARK_SPEED
            else:
                l = PARK_SPEED / 2
                
            # and publish
            nav.change_speed(l, a)

        nav.change_speed(0, 0)


    # the second part of parking: reverse back out of the space
    @staticmethod
    def drive_out(sensors, nav, heading, dist_threshold):

        # reset the heading angle, in case we've drifted
        nav.turnTo(sensors, heading)

        print("Driving backward")

        # take an initial snapshot of the time and error
        prev_err = Parking.sideways_error(sensors.full_scan)
        prev_time = time.time()

        # navigate using PD control until we've reached the required threshold
        while sensors.full_scan.ranges[0] < dist_threshold:
            
            # record new time and error values
            error = Parking.sideways_error(sensors.full_scan)
            deriv = (error - prev_err) / (time.time() - prev_time)
            # use for the proportional and derivative gains, respectively
            a = KP*error + KD*deriv
            
            # choose either the fast or the slow speed
            if sensors.full_scan.ranges[0] < dist_threshold - 0.2:
                l = PARK_SPEED
            else:
                l = PARK_SPEED / 2
                
            # publish
            nav.change_speed(-l, -a)

        nav.change_speed(0, 0)


    # the overall high level goal of this class: park the turtlebot.
    # returns the reading of the light sensor
    @staticmethod
    def park(sensors, nav):

        print("Commence parking!")

        heading = Parking.detect_parking_angle(sensors, nav)

        # turn to parking angle
        nav.turnTo(sensors, heading)
        dist = sensors.full_scan.ranges[0] # record original distance, to use again
        
        # drive into space
        Parking.drive_in(sensors, nav, heading, PARK_DIST)

	    # then record the colour of the blob
        nav.sleep(0.5)  
        colour = sensors.light_sensor

        # reverse out of space
        Parking.drive_out(sensors, nav, heading, dist)
        
        # return the colour of the blob
        return colour

