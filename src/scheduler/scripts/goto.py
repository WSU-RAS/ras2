#! /usr/bin/env python

import rospy

from actionlib import SimpleActionServer
from ras_msgs.msg import GotoAction, GotoFeedback, GotoResult
from adl.util import Goal, Task

import smach_ros
import smach

from findperson_smach import FindPersonSMACH
from goto_object_smach import GotoObjectSMACH

from adl.util import Goal, Status


class GotoServer:

    def __init__(self):
        self.rate = rospy.Rate(10)
        self.goto_server = SimpleActionServer(
            'goto', GotoAction,
            execute_cb=self.goto_execute,
            auto_start=False)
        self.goto_server.start()

    def goto_execute(self, goal):
        rospy.loginfo("Executing {} for {}".format(
            Goal.types[goal.type], Task.types[goal.task_number]))
        rospy.loginfo("error_step={}  error_object={}".format(
            goal.error_step, goal.error_object))

        outcome == "error"
        if goal.type == Goal.BASE:
            self.goto_feedback(Status.STARTED, "GO TO BASE SMACH STARTED")
            rospy.loginfo("Initiating GotoBase State Machine")
            sm_gotoobject = GotoObjectSMACH()
            outcome = sm_gotoobject.execute(
                task_number=goal.task_number,
                error_step=goal.error_step, base=True)

        elif goal.type == Goal.HUMAN:
            self.goto_feedback(Status.STARTED, "FIND PERSON SMACH STARTED")
            rospy.loginfo("Initiating FindPerson State Machine")
            sm_findperson = FindPersonSMACH()
            outcome = sm_findperson.execute(
                task_number=goal.task_number, error_step=goal.error_step)

        elif goal.type == Goal.OBJECT:
            self.goto_feedback(Status.STARTED, "GO TO OBJECT SMACH STARTED")
            rospy.loginfo("Initiating GotoObject State Machine")
            sm_gotoobject = GotoObjectSMACH()
            outcome = sm_gotoobject.execute(
                task_number=goal.task_number,
                error_step=goal.error_step, base=False)

        is_success = True if outcome == "finish" else False

        if is_success:
            self.goto_feedback(Status.COMPLETED, "SMACH SUCCESSFUL")
            rospy.loginfo("State machine successful")
        else:
            self.goto_feedback(Status.FAILED, "SMACH FAILED")
            rospy.logwarn("State machine failed")

        goto_result = GotoResult()
        goto_result.status = Status.COMPLETED if is_success else Status.FAILED
        goto_result.is_complete = is_success
        self.goto_server.set_succeeded(goto_result)

    def goto_feedback(self, status, text):
        feedback = GotoFeedback()
        feedback.status = status
        feedback.text = text
        self.goto_server.publish_feedback(feedback)


if __name__ == '__main__':
    try:
        rospy.init_node('goto_server')
        server = GotoServer()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
