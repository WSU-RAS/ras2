#!/usr/bin/env python

import rospy
import smach
import actionlib

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus
from object_detection_msgs.srv import ObjectQuery, ObjectQueryResponse
from ras_msgs.srv import Goto_xywz

class Goto():

    def __init__(self):

        rospy.init_node('goto')

        self.rate = rospy.Rate(10) #Hertz

        rospy.loginfo("Waiting for the move_base action server")
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)        
        self.move_base.wait_for_server(rospy.Duration(2))
        rospy.loginfo("Move_base action server is live!")

    
        self.create_service()

        rospy.loginfo('Goto service is live!')
        rospy.spin()

    def create_service(self):
        rospy.Service('goto', Goto_xywz, self.execute)

    def done_cb(self, state, data):
        self.is_running = False


    def execute(self, data):

        rospy.loginfo("Goto x={} y={} z={} w={}".format(data.x, data.y, data.z, data.w))
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = data.x
        goal.target_pose.pose.position.y = data.y
        goal.target_pose.pose.orientation.z = data.z
        goal.target_pose.pose.orientation.w = data.w

        self.is_running = True

        self.move_base.send_goal(goal, done_cb=self.done_cb)

        start_time = rospy.Time.now()
        timeout = rospy.Duration(secs=60, nsecs=0)
        while self.is_running and rospy.Time.now() - start_time < timeout:
            self.rate.sleep()


        self.move_base.cancel_goal()

        print(self.move_base.get_state())
        if self.move_base.get_state() == GoalStatus.SUCCEEDED:
            rospy.loginfo("GotoXY succeeded")
            return 'success'
        elif self.move_base.get_state() == GoalStatus.PREEMPTED:
            rospy.loginfo("GotoXYState Preempted")
            return 'preempted'
        else:
            rospy.loginfo("GotoXY failed")
            return 'fail'


if __name__ == '__main__':
    try:
        Goto()
    except rospy.ROSInterruptException:
        rospy.loginfo("Exception thrown in goto_service")


'''
uint8 status
uint8 PENDING         = 0   # The goal has yet to be processed by the action server
uint8 ACTIVE          = 1   # The goal is currently being processed by the action server
uint8 PREEMPTED       = 2   # The goal received a cancel request after it started executing
                            #   and has since completed its execution (Terminal State)
uint8 SUCCEEDED       = 3   # The goal was achieved successfully by the action server (Terminal State)
uint8 ABORTED         = 4   # The goal was aborted during execution by the action server due
                            #    to some failure (Terminal State)
uint8 REJECTED        = 5   # The goal was rejected by the action server without being processed,
                            #    because the goal was unattainable or invalid (Terminal State)
uint8 PREEMPTING      = 6   # The goal received a cancel request after it started executing
                            #    and has not yet completed execution
uint8 RECALLING       = 7   # The goal received a cancel request before it started executing,
                            #    but the action server has not yet confirmed that the goal is canceled
uint8 RECALLED        = 8   # The goal received a cancel request before it started executing
                            #    and was successfully cancelled (Terminal State)
uint8 LOST            = 9   # An action client can determine that a goal is LOST. This should not be
                            #    sent over the wire by an action server
'''

