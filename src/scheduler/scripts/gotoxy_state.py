#!/usr/bin/env python
"""
Used in both findperson_smach.py and goto_object_smach.py
"""
import rospy
import smach
import actionlib

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus

class GotoXYState(smach.State):

    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=['success', 'fail', 'preempted'],
            input_keys=['position_x_in', 'position_y_in', 'orientation_z_in', 'orientation_w_in'])

        self.rate = rospy.Rate(10)
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)

        rospy.loginfo("Waiting for the move_base action server")
        self.move_base.wait_for_server(rospy.Duration(2))

    def done_cb(self, terminal_state, result):
        if terminal_state == GoalStatus.SUCCEEDED:
            self.success = True
        self.is_running = False

    def execute(self, userdata):
        rospy.loginfo("Executing state GotoXY")

        rospy.loginfo(
            "Goto x={} y={} z={} w={}".format(userdata.position_x_in, userdata.position_y_in,
                userdata.orientation_z_in, userdata.orientation_w_in))
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = userdata.position_x_in
        goal.target_pose.pose.position.y = userdata.position_y_in
        goal.target_pose.pose.orientation.z = userdata.orientation_z_in
        goal.target_pose.pose.orientation.w = userdata.orientation_w_in

        self.success = False
        self.is_running = True
        self.move_base.send_goal(goal, done_cb=self.done_cb)

        start_time = rospy.Time.now()
        timeout = rospy.Duration(secs=60, nsecs=0)
        while self.is_running and rospy.Time.now() - start_time < timeout:
            if self.preempt_requested():
                self.service_preempt()
                return 'preempted'
            self.rate.sleep()

        if not self.success:
            self.move_base.cancel_goal()
            rospy.loginfo("GotoXY failed")
            return "fail"

        rospy.loginfo("GotoXY succeeded")
        return "success"

    def request_preempt(self):
        smach.State.request_preempt(self)
        rospy.logwarn("GotoXYState Preempted")
