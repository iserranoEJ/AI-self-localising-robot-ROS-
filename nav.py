#!/usr/bin/env python
from __future__ import print_function
import time
import rospy
from sensor_msgs.msg import Illuminance
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovariance
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from math import atan2
from math import floor
from math import pi

from Astar import *
from Localise import *
from Parking import *


# Wrapper class for storing points
class Point:
    def __init__(self, xIn, yIn):
        self.x = xIn
        self.y = yIn

#List of all coordinates that contain obstacles
OBSTACLES = {"RED_CYL1" :  [Point(1.23, -2.7)],
             "RED_CYL2" :  [Point(0.83, -2.7)],
             "RED_CYL3" :  [Point(0.41, -2.7)],
             "YELLOW" :    [Point(2.899, -2.7)],
             "PURPLE" :    [Point(3.34, -2.7)],
             "BAR" :       [Point(2.53, -2.7),
                            Point(2.38, -2.7),
                            Point(2.18, -2.7),
                            Point(1.98, -2.7),
                            Point(1.78, -2.7),
                            Point(1.58, -2.7)],
             "BLOCK_LOW" : [Point(1.50, -2.7),
                            Point(1.25, -2.7),
                            Point(1.00, -2.7),
                            Point(0.75, -2.7),
                            Point(0.50, -2.7),
                            Point(0.25, -2.7),
                            Point(0.00, -2.7),
                            Point(-0.25, -2.7),
                            Point(-0.50, -2.7),
                            Point(-0.75, -2.7),
                            Point(-1.00, -2.7)],
	    "PARK_BLOCK" : [Point(1.34, -1.62),
			    Point(1.51, -1.53)]}

# turn a list of string descriptors into the list of all
# corresponding obstacle points
def assemble_obstacles(keys):

    obst_list = []

    for key in keys:
        for obst in OBSTACLES[key]:
            obst_list.append(obst)

    return obst_list
    

# This class encapsulates the reading of data from the ros topics, and the latest value can be
# accessed directly or through some of the helper methods provided.
class SensorInfo:

	def __init__(self):
		self.light_sensor = None
		self.scan_dist = None
		self.full_scan = None
		self.x = None
		self.y = None
		self.phi = None
		
		# initialise ropsy subscribers		
		rospy.Subscriber("light_sensor_plugin/lightSensor", Illuminance, self.read_light_sensor)
		rospy.Subscriber("odom", Odometry, self.read_odom)
		rospy.Subscriber("scan", LaserScan, self.read_scan)

    # Return the coordinates and heading angle of the robot
	def read_odom(self, data):
		quat = data.pose.pose.orientation
		orientation_list = [quat.x, quat.y, quat.z, quat.w]
		(roll, pitch, yaw) = euler_from_quaternion(orientation_list)
		self.phi = yaw

		self.x = data.pose.pose.position.x
		self.y = data.pose.pose.position.y
			

	# Returns true if all sensors have meaningful readings.
	def is_init(self):
		if (not self.light_sensor
				or not self.scan_dist
				or not self.x
				or not self.y
				or not self.phi
			):
			return False
		return True
	

	def read_light_sensor(self, data):
		self.light_sensor = data.illuminance

    # Read from the laser scanner topic
	def read_scan(self, data):
		self.full_scan = data
		if not data.ranges:
			self.scan_dist = None
		else:
			minimum = float("inf")
			for x in data.ranges:
				if x < minimum:
					minimum = x
			self.scan_dist = minimum

    # Detect color through light sensor for line localisation
	def read_colour(self):
		if self.light_sensor >= 220.0:
			return 1 # White
		if self.light_sensor <= 180.0:
			return 2 # Grey
		else:
			return 0 # Background



# The Navigator class provides an easy to use interface for the rest of the code we have 
# developed. You can instruct the turtlebot to perform much higher level commands such as
# navigating to a cartesian location, or turn to a certain heading.
class Navigator:

    def __init__(self):
        self.publisher = rospy.Publisher("cmd_vel", Twist, queue_size = 10)

    # Change turtlebot's linear and angular velocities and publish them to cmd_vel topic.
    def change_speed(self, lin, ang):
        twist = Twist()
        twist.linear.x = lin
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = ang
        #print("Send {0} {1}".format(lin, ang))
        self.publisher.publish(twist)


    # Move forward for some time and distance. Optionally, choose an angular velocity and
    # a fraction of the time for which to pause after moving.
    def forward(self, time, distance, ang=0, sleep_factor=0.5):
        
        # compute the required velocity
        vel = distance / time
        a = ang / time
        # publish and sleep
        self.change_speed(vel, a)
        self.sleep(time)
        # stop and pause for a short moment
        self.change_speed(0, 0)
        self.sleep(time*sleep_factor)


    @staticmethod
    def sleep(t):
        rospy.sleep(t)


    # Checks that the turtlebot doesn't turn more than 180 degrees, but instead turns in the shorter direction.
    @staticmethod
    def shortest_rotation(rad):
        if rad > pi:
            return rad - 2*pi
        if rad < -pi:
            return rad + 2*pi
        return rad

    # Rotates robot to point to a certain heading angle
    def turnTo(self, sensors, heading):
    
        ERR_THRESHOLD = 0.01 # threshold of accuracy required
        FAST_SPEED = 0.5
        SLOW_SPEED = 0.18
        SPEED_THRESHOLD = 0.3 # point at which to start slowing down

        error = abs(sensors.phi - heading)
        while error > ERR_THRESHOLD:
            # find speed and direction
            turn_speed = FAST_SPEED if error > SPEED_THRESHOLD else SLOW_SPEED
            direction = -1 if Navigator.shortest_rotation(heading - sensors.phi) < 0 else 1
            # change speed accordingly
            self.change_speed(0, turn_speed*direction)
            # update error for next loop
            error = abs(sensors.phi - heading)

        self.change_speed(0, 0)


	# Progress towards a desired cartesian point
    def navigateTo(self, des_x, des_y, sensors, kp=0.5, lin=0.1):
		
        SHARP_TURN = 0.25
        #print("navigating with lin={0}".format(lin))
        des_phi = atan2(des_y - sensors.y, des_x - sensors.x)
        if abs(Navigator.shortest_rotation(des_phi - sensors.phi)) > SHARP_TURN:
            self.turnTo(sensors, des_phi)

        # main control loop
        while abs(des_x - sensors.x) > 0.02 or abs(des_y - sensors.y) > 0.02:
            # calculate deviation from required heading
            des_phi = atan2(des_y - sensors.y, des_x - sensors.x)
            error = Navigator.shortest_rotation(des_phi - sensors.phi)
			
            # and use for the angular component
            a = kp * error
			
			# dampen linear component slightly, depending on angular
            l = lin if abs(a) < SHARP_TURN else 0
            self.change_speed(l, a)
            

        self.change_speed(0, 0)
        

# Main loop that coordinates the whole localisation, A*, navigation and parking algorithms.
def mainloop():

    # prompt user for which case we have
    case = int(input("Enter the case number: "))

    # Set obstacles according to the case selected from user's input
    if case == 1:
        obst_list = assemble_obstacles(["RED_CYL1", "RED_CYL2", "BAR", "YELLOW", "PURPLE"])
    elif case == 2:
        obst_list = assemble_obstacles(["RED_CYL1", "RED_CYL3", "BAR", "YELLOW", "PURPLE"])
    elif case == 3:
        obst_list = assemble_obstacles(["RED_CYL2", "RED_CYL3", "BAR", "YELLOW", "PURPLE"])

    # initialise sensor object and navigator
    sensors = SensorInfo()
    nav = Navigator()

    # Set up subscribers and publisher
    rospy.init_node("turtlenav", anonymous=True)

    # Do nothing if the sensors are not active
    while not sensors.is_init():
    	print("Light: {0}".format(sensors.light_sensor))
    	rospy.sleep(0.5)

    print("Navigation system up and ready!")
    nav.change_speed(0, 0)

    ########### Navigate to the first goal, near the parking spot
    goal = (0.895494, -1.741872)

    start = BayesLoc.localise(nav, sensors)
    #start = (sensors.x, sensors.y)	

    route = Astar.a_star_search(start[0], start[1],
    			goal[0], goal[1], obst_list)

    for waypoint in route:
    	print("Navigating to waypoint: ({0}, {1})".format(round(waypoint[0], 2), round(waypoint[1], 2)))
    	nav.navigateTo(waypoint[0], waypoint[1], sensors)

    ########### Park in the spot (returns the colour detected)
    colour = Parking.park(sensors, nav)	
    
    if colour >= 143 and colour < 170:
        print("Purple rain")
        obst_list = assemble_obstacles(["PURPLE", "BAR", "BLOCK_LOW", "PARK_BLOCK"])

    elif colour >= 170 and colour < 200:
        print("Yellow mellow")
        obst_list = assemble_obstacles(["YELLOW", "BAR", "BLOCK_LOW", "PARK_BLOCK"])
	
    else:
        print("Yellow mellow...")
        obst_list = assemble_obstacles(["YELLOW", "BAR", "BLOCK_LOW", "PARK_BLOCK"])
    
    ############ Navigate home!
    second_goal = (2.03630784842, -5.19129478774)
    
    start = (sensors.x, sensors.y)

    # A* search
    route = Astar.a_star_search(start[0], start[1],
                          second_goal[0], second_goal[1], obst_list)

    # Navigate to the cheapest nodes given by A*
    for waypoint in route:
        print("Navigating to waypoint: ({0}, {1})".format(round(waypoint[0], 2), round(waypoint[1], 2)))
        nav.navigateTo(waypoint[0], waypoint[1], sensors)

    print("Mission accomplished")
    exit(0)

if __name__ == "__main__":
	try:
		mainloop()
	except rospy.ROSInterruptException:
		pass
