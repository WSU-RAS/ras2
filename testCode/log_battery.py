#!/usr/bin/env python
import rospy
import math
import time

#Include data structure for handling map
from turtlebot3_msgs.msg import SensorState


import csv



time = 0

def callback(data):
	global time

	if time + 10 < data.header.stamp.secs:
		time = data.header.stamp.secs

		with open('battery_log.csv', 'a') as csvfile:
			spamwriter = csv.writer(csvfile, delimiter=',')
			spamwriter.writerow((data.battery, data.header.stamp.secs))

		print(data.battery)

def listener():
	rospy.init_node('listener', anonymous=True)
	rospy.Subscriber('/sensor_state', SensorState, callback)
	
	while not rospy.is_shutdown():
		rospy.sleep(10)


if __name__ == '__main__':
	listener()
