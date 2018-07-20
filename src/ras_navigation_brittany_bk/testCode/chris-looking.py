#!/usr/bin/env python
import rospy
import math
import time

#Include data structure for handling map
from turtlebot3_msgs.msg import SensorState
from ras_msgs.srv import Goto_xy
import dynamic_reconfigure.client
import csv



time = 0

def callback(data):
	nav_to_dock(data)

def nav_to_dock():
	rospy.wait_for_service('Goto_xy') #waits until Goto_xy function is up

	nav = rospy.serviceProxy('Goto_xy', Goto_xy) #sets nav as funtion Goto_xy defined elsewhere?
	nav_results = nav(x, y) #inputs xy coordinates into nav, into Goto_xy and carries out

def finesse_nav():
	recon = dynamic_reconfigure.client.Client('move_base') #reconfigures node from  dynamic
	config = client.update_configuration({}) #gets current configuration without changes for future reference
	params =  ('max_vel_x': 0.05, 'max_trans_vel': 0.05, 'accel_lim_x': 0.5, 'xy_goal_tolerance': 0.025) #sets max velocities/acceleration low and goal tolerance to radius of 1 inch
	recon.update_configuration(params) #updates recon with above
	nav

def connect():

def listener():
	rospy.init_node('docking_service', anonymous=True) #creates a node the file (only 1 per file needed)
	rospy.Subscriber('/sensor_state', SensorState, callback) #subscribes to sensor state, calls sensor state in  
	
	while not rospy.is_shutdown(): #basically has check every results 10 seconds
		rospy.sleep(10)


if __name__ == '__main__': #main body of the function. starts everything going
	listener()
