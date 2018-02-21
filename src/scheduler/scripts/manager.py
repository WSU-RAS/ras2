#! /usr/bin/env python

import rospy

from actionlib_msgs.msg import GoalStatus
from actionlib import SimpleActionServer, SimpleActionClient
from scheduler.msg import GotoAction, GotoGoal
from scheduler.msg import DoErrorAction, DoErrorFeedback, DoErrorResult

'''
GoalStatus = [PENDING, ACTIVE, RECALLED, REJECTED, PREEMPTED, ABORTED, SUCCEEDED, LOST]
'''

class SchedulerServer:

    __doerror_feedback = DoErrorFeedback()
    __doerror_result = DoErrorResult()
    __goto_goal = GotoGoal()


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
        rospy.loginfo(goal.error)

        self.__goto_goal.goal = 20
        self.goto_client.send_goal(
            self.__goto_goal,
            done_cb=self.goto_done_cb,
            active_cb=self.goto_active_cb,
            feedback_cb=self.goto_feedback_cb)
        self.rate.sleep()

        while self.is_goto_active:
            self.rate.sleep()
            self.__doerror_feedback = 1
            self.doerror_server.publish_feedback(self.__doerror_feedback)
        self.__doerror_feedback = 0
        self.doerror_server.publish_feedback(self.__doerror_feedback)

        self.__doerror_result.result = 6000
        self.__doerror_result.is_complete = True
        self.doerror_server.set_succeeded(self.__doerror_result)

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
        rospy.loginfo("terminal state: {}  result: ({}, {})".format(status, result.result, result.is_complete))

    def goto_active_cb(self):
        self.is_goto_active = True

    def goto_feedback_cb(self, feedback):
        rospy.loginfo("current x={}, y={}, z={}".format(feedback.x, feedback.y, feedback.z))

if __name__ == '__main__':
    rospy.init_node('scheduler_server')
    server = SchedulerServer()
    rospy.spin()

        
