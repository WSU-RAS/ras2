#!/usr/bin/env python

import rospy
import smach
import smach_ros
import actionlib
from smach_ros import SimpleActionState

# roslib.load_manifest('smach_tutorials')
# roslib.load_manifest('FindPerson')

from actionlib import SimpleActionServer
# from ras_msgs.msg import GotoAction, GotoFeedback, GotoResult
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import *
from actionlib_msgs.msg import GoalStatus

from find_person.msg import FindPersonAction, FindPersonResult, FindPersonGoal


# class to find the person
# this state sends the task number and error step to the 
# actionlib server 
# upon executing this the robot finds tries to find the person 
# with the current field of view
# it has two options, it either finds it or there is an error

# input should  Goal: task number and error step
# output should be Result: Found, location[X, Y]

class FindPerson(smach.State):

	def __init__(self):	
		smach.State.__init__(self, outcomes = ['found', 'error'],\
							output_keys = ['find_person_output'])
		# self.counter = 0

	def execute(self, userdata):
		pass
	
# this class is is the state which is triggered once the camera finds the human,
# it triggers the gotoxy node which navigates the robot to the given xy location.
class GotoXY(smach.State):

	def __init__(self):
		smach.State.__init__(self, outcomes = {'done', 'error'}, \
			input_keys = ['goto_input'], output_keys = ['goto_output'])

	def execute(self, userdata):
		rospy.loginfo('Executing state GOTO XY')
		return 'done'
		pass

def main():
	# find_person_goal.task_number = 1
	# findperson_goal.error_step = 4
	rospy.init_node("find_person")
	
	# SMACH State Machine
	sm = smach.StateMachine(outcomes = ['finish', 'error'], \
		input_keys = ['sm_input'], \
		output_keys = ['sm_output'])
	
	# dummy goal set for testing
	find_person_goal = FindPersonGoal()
	find_person_goal.task_number = 2
	find_person_goal.error_step = 3

	# container having all the states
	with sm:
		# alter the x, y coordinates to prevent the robot from running into human
		def findperson_result_cb (self, result):
			result.X -= 12.0
			result.Y -= 0.0
			return result 

		smach.StateMachine.add('FINDPERSON', smach_ros.SimpleActionState('find_person', FindPersonAction,\
			goal = find_person_goal),  \
			# result_cb = findperson_result_cb) \
			transitions = {'succeeded': 'GOTOXY', 'aborted': 'error', 'preempted': 'error'}, \
			remapping = {'find_person_output' : 'goto_input'})
		
		def goto_goal_cb(x, y):
			goal = MoveBaseGoal()
			goal.target_pose.header.frame_id = "map"
			goal.target_pose.header.stamp = rospy.Time.now()
			goal.target_pose.pose.position.x = x
			goal.target_pose.pose.position.y = y
			goal.target_pose.pose.orientation.w = 1.0 # Go forward
			return 'done'


		smach.StateMachine.add('GOTOXY', smach_ros.SimpleActionState('move_base', MoveBaseAction, goal_cb = goto_goal_cb), \
			transitions = {'succeeded': 'finish', 'aborted': 'error', 'preempted': 'error'}, \
			remapping = {'goto_input' : 'find_person_output'})
		
		
	sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
	sis.start()
	outcome =  sm.execute()
	rospy.signal_shutdown('All done.')


if __name__ == '__main__':
	main()



