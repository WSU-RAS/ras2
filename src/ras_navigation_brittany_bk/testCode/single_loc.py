#!/usr/bin/env python
'''
The idea of this is to let RAS know when he is localized or not. 
This is a service you want to call and return a T/F message. 
Currently having issues with tf not working to receive the information
for his current location. Going to try nested class.  
'''





import rospy, math, time, sys, os
from tf import TransformListener
from ras_msgs.srv import Localize
from std_msgs.msg import String
from tf.msg import tfMessage


class LocIt:
	def __init__(self):


		rospy.init_node('nav_test_localize2', anonymous=False)
		s = rospy.Service('localize', Localize, self.start_loc)
		rospy.loginfo("Reporting localized service")
		rospy.spin()
		rospy.on_shutdown(rospy.loginfo("Ending Service"))

	def start_loc(self, notdatainput):



		# print(service_input)
		original = self.RAS()
		original.getRAS()
		starting_pos = original.pos
		starting_time = original.thyme
		loc = self.Local(starting_pos[0], starting_pos[1], starting_time) #set to now
		finished = False #are we localized?
		counter = -1

		while not finished:
			counter += 1
			current_RAS = self.RAS()
			current_RAS.getRAS()
			current_pos = current_RAS.pos
			current_time = current_RAS.thyme
			loc.newLocal(current_pos[0], current_pos[1], current_time)
			dis = loc.getDistance()
			time_dif = loc.getTime()
			scale = 1/time_dif
			adjusted_dis = dis * scale
			time.sleep(2.5)
			if adjusted_dis > 10:
				rospy.loginfo("Localized, moving faster than 10m/s")
				finished = True
				rospy.loginfo("Hooray, we've localized")
				self.publishIt("True")
				return "Success"
			#elif counter > 15:
			#	rospy.loginfo("Not localized and past interation point")
			#	self.publishIt("False")
			#	return "Failure"
			else: 
				log_it = "Haven't localized yet, going again " + str(int(round(adjusted_dis))) + "m/s " + str(counter)			
				
				rospy.loginfo(log_it)


	def publishIt(self, mesg):
		pub = rospy.Publisher('localized', String, queue_size=10)
		pub.publish(mesg)


	class Local:
		def __init__(self, x, y, t):
			self.current_local = self.last_local= (x, y)
			self.current_time = self.last_time = t


 		def newLocal(self, x, y, t):
			before = "Current local: " + str(self.current_local) + ". Current time: " + str(self.current_time)
			print(before)
			self.last_local = self.current_local
			self.current_local = (x, y)
			self.last_time = self.current_time
			self.current_time = t
			after = "Updated local: " + str(self.current_local) + ". Updated time: " + str(self.current_time)
			print(after)

		def getDistance(self):
			x = self.current_local[0] - self.last_local[0]
			y = self.current_local[1] - self.last_local[1]
			return math.hypot(x, y)
        

		def getTime(self):
			if(self.current_time > self.last_time): #Should always be this way
 				return self.current_time - self.last_time

			else:
				rospy.loginfo("Unexpected Time")
				return abs(self.current_time - self.last_time)


	class RAS:
 		def __init__(self):
			self.pos = None
			self.thyme = None


		def getRAS(self):

			sub = rospy.Subscriber('tf', tfMessage, self.getPOSThyme)

		def getPOSThyme(self, tf_msg):
			print(tf_msg[0])

	
			#for pose in tf_msg:
			#if tf_msg.transforms.child_frame_id == "base_link":
			#	(trans, rot) = pose.transform
			#	print ("Yay we found it")
			#	self.pos = (trans[0], trans[1])
			#	self.thyme = time.time()
				


			

			# tf = TransformListener()
			# tf.waitForTransform("/base_link", "/map", rospy.Time(), rospy.Duration(1.0))			
			# finding = True
			# count = 0
			# while finding:
			# 	count += 1
			# 	tf.waitForTransform("/base_link", "/map", rospy.Time(), rospy.Duration(1.0))
			# 	(trans, rot)=tf.lookupTransform("/base_link", "/map", rospy.Time())
			# 	self.pos = (trans[0], trans[1])
			# 	self.thyme = time.time()
			# 	if self.pos is not None:
			# 		finding = False


			


if __name__ == '__main__':
	try:

		LocIt()
	except rospy.ROSInterruptException:
		rospy.loginfo("Exception thrown")
