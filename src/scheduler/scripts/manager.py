#! /usr/bin/env python

import rospy

from actionlib_msgs.msg import GoalStatus
from actionlib import SimpleActionServer, SimpleActionClient
from scheduler.msg import GotoAction, GotoGoal
from scheduler.msg import DoErrorAction, DoErrorFeedback, DoErrorResult


class SchedulerServer:

    def __init__(self):
        self.is_goto_active = False
        self.rate = rospy.Rate(10)

        self.doerror_server = SimpleActionServer(
            'doerror_server', DoErrorAction,
            execute_cb=self.doerror_execute,
            auto_start=False)
        self.doerror_server.start()

        self.goto_client = SimpleActionClient(
            'goto', GotoAction)
        self.doerror_server.start()
        self.goto_client.wait_for_server()

    def doerror_execute(self, goal):
        rospy.loginfo("Executing Do Error")

        goto_goal = GotoGoal()
        goto_goal.task_number = goal.task_number
        goto_goal.error_step = goal.error_step
        goto_goal.error_object = goal.error_step
        self.goto_client.send_goal(
            goto_goal,
            done_cb=self.goto_done_cb,
            active_cb=self.goto_active_cb,
            feedback_cb=self.goto_feedback_cb)

        while not self.is_goto_active:
            self.rate.sleep()

        doerror_feedback = DoErrorFeedback()
        doerror_feedback.status = 1
        doerror_feedback.text = "SENT GOTO GOAL TO TURTLEBOT"
        self.doerror_server.publish_feedback(doerror_feedback)
        self.rate.sleep()

        while self.is_goto_active:
            doerror_feedback.status = 2
            doerror_feedback.text = "TURTLEBOT NAVIGATING"
            self.doerror_server.publish_feedback(doerror_feedback)
            self.rate.sleep()

        doerror_feedback.status = 3
        doerror_feedback.text = "TURTLEBOT COMPLETED TASK"
        self.doerror_server.publish_feedback(doerror_feedback)

        doerror_result = DoErrorResult()
        doerror_result.status = 3
        doerror_result.is_complete = True
        self.doerror_server.set_succeeded(doerror_result)

    def goto_done_cb(self, terminal_state, result):
        self.is_goto_active = False
        status = "UNKNOWN"
        if terminal_state == GoalStatus.RECALLED:
            status = "RECALLED"
        elif terminal_state == GoalStatus.REJECTED:
            status = "REJECTED"
        elif terminal_state == GoalStatus.PREEMPTED:
            status = "PREMEPTED"
        elif terminal_state == GoalStatus.ABORTED:
            status = "ABORTED"
        elif terminal_state == GoalStatus.SUCCEEDED:
            status = "SUCCEEDED"
        elif terminal_state == GoalStatus.LOST:
            status = "LOST"
        rospy.loginfo("terminal state: {}  result: ({}, {})".format(status, result.status, result.is_complete))

    def goto_active_cb(self):
        self.is_goto_active = True

    def goto_feedback_cb(self, feedback):
        rospy.loginfo("goto feedback: x={}, y={}, z={}, status={}".format(feedback.x, feedback.y, feedback.z, feedback.text))

if __name__ == '__main__':
    rospy.init_node('scheduler_server')
    server = SchedulerServer()
    rospy.spin()

        
