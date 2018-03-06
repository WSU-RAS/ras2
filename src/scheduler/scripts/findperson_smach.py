#!/usr/bin/env python

import rospy
import smach
import smach_ros
# roslib.load_manifest('smach_tutorials')
roslib.load_manifest('FindPerson')

from actionlib import SimpleActionServer
from ras_msgs.msg import GotoAction, GotoFeedback, GotoResult


# class to find the person
# this state sends the task number and error step to the 
# actionlib server in the 
# upon executing this the robot finds tries to find the person 
# with the current field of view
# it has two options, it either finds it or not finds it.
class FindPerson(smach.State):

	def __init__(self):	
		smach.State.__init__(self, outcomes = ['found', 'error'])
		self.counter = 0

	def execute(self, userdata):
		pass
	
		# if self.counter < 3:
		# 	self.counter += 1
		# 	return 'not_found'
		# else:
		# 	return 'found'
		# need to trigger the find person node

# this class is is the state which is triggered once the camera finds the human,
# it triggers the gotoxy node which navigates the robot to the given xy location.
class GotoXY(smach.State):

	def __init__(self):
		smach.State.__init__(self, outcomes = {'done', 'error'})

	def execute(self, userdata):
		rospy.loginfo('Executing state GOTO XY')
		return 'done'
		pass


		# here is the part where we trigger Chris' part where we have the coordinates and
		# just need to navigate the robot through the map

def main():
	rospy.init_node("find_person")
	
	# SMACH State Machine
	sm = smach.StateMachine(outcomes = ['finish', 'error'])

	# container having all the states
	with sm:
		smach.StateMachine.add('FINDPERSON', FindPerson(), transitions = {'found': 'GOTOXY', 'error': 'error'})
		smach.StateMachine.add('GOTOXY', GotoXY(), transitions = {'done': 'finish', 'error': 'error'})

	outcome =  sm.execute()
	rospy.signal_shutdown('All done.')


if __name__ == '__main__':
	main()



