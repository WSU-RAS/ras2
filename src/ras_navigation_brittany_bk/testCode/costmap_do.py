#!/usr/bin/env python
import rospy
import math
import time

#Include data structure for handling map
from nav_msgs.msg import OccupancyGrid

#Global publisher
pub = rospy.Publisher('/move_base/global_costmap/costmap', OccupancyGrid, queue_size=10)
#pub2 = rospy.Publisher('/move_base/local_costmap/costmap', OccupancyGrid, queue_size=10)
	
def callback(data):

	grid = list(data.data)

	for ind,val in enumerate(grid):
		if val < 10:
			grid[ind] = 0
		if val > 60:
			grid[ind] = 100

	data.data = tuple(grid)
	print("publishing data!")
	pub.publish(data)
	#pub2.publish(data)

def listener():
	rospy.init_node('listener', anonymous=True)
	rospy.Subscriber('/map', OccupancyGrid, callback)
	
	while not rospy.is_shutdown():
		# do whatever you want here
		#pub.publish(foo)
		rospy.sleep(1)  # sleep for one second


if __name__ == '__main__':
	listener()
