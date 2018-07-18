#!/usr/bin/env python
import rospy
import math
import time

#Include data structure for handling map
from turtlebot3_msgs.msg import SensorState
from ras_msgs.srv import Goto_xywz
import dynamic_reconfigure.client
import csv
from ras_msgs.srv import Goto_human


class Dock_Service: #necessary for letting ros see the code by making the service public on ros
	def __init__(self):
		dock_instance = Dock()
		rospy.init_node('Dock_Service', anonymous = False) #sets node type
		s = rospy.Service('Dock', Goto_human, dock_instance.begin) #extend service/make visible. the use of Dock starts that class 
		rospy.loginfo("beginning docking service") #setting name/basic info
		rospy.spin() #setting better format while true/keeps main thread alive

class Dock:

	def __init__(self): #sets variables to be referenced later in the class
		rospy.loginfo("service declared") #gives prompts to user
		rospy.Subscriber('/sensor_state', SensorState, self.callback) #subscribes to sensor state, calls sensor state in
		self.data = None #sets variable of self.data up
		self.start_time = time.time() #when class initialized, time will start.
		self.loop_time = time.time() 
		self.t1 = 3 #sets time variables. 
		self.t2 = 20 #time until robot re-sets to docking posion
		self.t3 = 120 #time until robot aborts entirely
		self.x1 = 2.408 #sets pre-docking positions
		self.y1 = -3.380
		self.w = 0.0348631 #orientation
		self.z = 0.999392
		self.x2 = 2.024 #sets where the docking position is
		self.y2 = -3.397
		self.succeed = False #priming variable

	def begin(self,data):
		rospy.loginfo("Service requested") #give prompts to user
		self.nav_to_dock()
		self.finesse_nav()
		return self.connect()

	def callback(self,data): #pulls in data and assigns it in parallel
		self.data = data

	def nav_to_dock(self):
		rospy.wait_for_service('goto_xy') #waits until Goto_xy function is up
		rospy.loginfo("go to xy found") #informs user of success
		nav = rospy.ServiceProxy('goto_xy', Goto_xywz) #sets nav as funtion Goto_xy defined elsewhere?
		nav_results = nav(self.x1, self.y1, self.w, self.z) #inputs xy coordinates into nav, into Goto_xy and carries out

	def finesse_nav(self):
		recon = dynamic_reconfigure.client.Client('move_base/DWAPlannerROS') #reconfigures node from  dynamic
		config = recon.update_configuration({}) #gets current configuration without changes for future reference
		params = {'max_vel_x': 0.05, 'max_vel_y': 0.05, 'xy_goal_tolerance': 0.038, 'acc_lim_x': .5} #sets max velocities/acceleration low and goal tolerance to radius of 1 inch
		print config
		recon.update_configuration(params) #updates recon with above
		nav = rospy.ServiceProxy('goto_xy', Goto_xywz)
		nav_results = nav(self.x2, self.y2, self.w, self.z) #sends to final destination
		recon.update_configuration(config) #resets parameters

	def connect(self):
		self.loop_time = time.time()
		self.check_voltage()
		while not self.succeed and self.t2 >= time.time() - self.loop_time and self.t3 >= time.time() - self.start_time: #continues loop until one of condition described below is met 
			self.wiggle()
			self.check_voltage()
		if self.succeed: #if we connect will return successful docking
			return "We have docked!"
		if self.t2 <= time.time() - self.loop_time: #if we have exceeded time t2 will restart the navigation process
			rospy.loginfo("naving naving")
			self.nav_to_dock()
			self.finesse_nav()
			return self.connect() #will recall connect to effectively restart it
		if self.t3 <= time.time() - self.start_time: #if we have exceeded time t3 will announce failure
			return "We failed to dock"
	def check_voltage(self):
		if self.data.battery > 19: #checks if voltage is above 11.8. theoretically this means its charging
			self.succeed = True
		time.sleep(self.t1) #waits for period of time
		if self.data.battery > 19: #checks if voltage is above 11.8. theoretically this means its charging
			self.succeed = True


	def wiggle(self): #will make the robot wiggle in the future
		rospy.loginfo("wiggling")
		return None


if __name__ == '__main__': #main body of the function. starts everything going
	Dock_Service()
