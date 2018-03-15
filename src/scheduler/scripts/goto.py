#! /usr/bin/env python

import rospy

from actionlib import SimpleActionServer
from ras_msgs.msg import GotoAction, GotoFeedback, GotoResult
from adl.util import Goal, Task

import smach_ros
import smach

from findperson_smach import FindPersonSMACH
from goto_object_smach import GotoObjectSMACH

class GotoServer:

    def __init__(self):
        self.rate = rospy.Rate(10)
        self.goto_server = SimpleActionServer(
            'goto', GotoAction,
            execute_cb=self.goto_execute,
            auto_start=False)
        self.goto_server.start()

    def goto_execute(self, goal):
        x = 20
        y = -20
        z = 0

        rospy.loginfo("Executing {} for {}".format(
            Goal.types[goal.type], Task.types[goal.task_number]))
        rospy.loginfo("error_step={}  error_object={}".format(goal.error_step, goal.error_object))
        # if the goal is to go to base
        if goal.type == 0:
            rospy.loginfo("Initiating movement to base")
            sm_gotoobject = GotoObjectSMACH()
            sm_gotoobject.execute(task_number = goal.task_number, error_step = goal.error_step, base = True )
        # if the goal is to goto human
        elif goal.type == 1:
            rospy.loginfo("Initiating FindPerson State Machine")
            sm_findperson = FindPersonSMACH()
            sm_findperson.execute(task_number = goal.task_number, error_step = goal.error_step)
        elif goal.type == 2 :
            rospy.loginfo("Initiating GotoObject State Machine")
            sm_gotoobject = GotoObjectSMACH()
            sm_gotoobject.execute(task_number = goal.task_number, error_step = goal.error_step, base = False )

	"""
        goto_feedback = GotoFeedback()
        while True:
            goto_feedback.x = x
            goto_feedback.y = y
            goto_feedback.z = z
            goto_feedback.status = 1
            goto_feedback.text = 'ROBOT MOVING'
            self.goto_server.publish_feedback(goto_feedback)
            self.rate.sleep()
            x -= 1
            y += 1
            if x == 0 and y == 0 and z == 0:
                break

        goto_feedback.x = x
        goto_feedback.y = y
        goto_feedback.z = z
        goto_feedback.status = 2
        goto_feedback.text = 'TASK COMPLETED'
	"""

        goto_result = GotoResult()
        goto_result.status = 2
        goto_result.is_complete = True
        self.goto_server.set_succeeded(goto_result)


if __name__ == '__main__':
    try:
        rospy.init_node('goto_server')
        server = GotoServer()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
