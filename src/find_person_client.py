#! /usr/bin/env python

# import the libraries

import rospy
import roslib
import actionlib
roslib.load_manifest('find_person')

from find_person.msg import FindPersonAction,  FindPersonGoal

if __name__ == '__main__':
	rospy.init_node('find_person_client')
	client = actionlib.SimpleActionClient('find_person', FindPersonAction)
	client.wait_for_server()

	goal = FindPersonGoal()

	client.send_goal(goal)
	client.wait_for_result(rospy.Duration.from_sec(5.0))


