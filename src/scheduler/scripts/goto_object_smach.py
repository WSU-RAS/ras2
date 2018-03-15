#!/usr/bin/env python

import rospy
import actionlib_msgs
import actionlib
import smach
import smach_ros
import time

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus

from object_detection_msgs.srv import ObjectQuery, ObjectQueryResponse
from adl.util import Task, WaterPlantsDag, WalkDogDag, TakeMedicationDag


class FindObjectState(smach.State):

    def __init__(self):
        smach.State.__init__(
            self,
            outcomes = ['success', 'fail'],
            input_keys = ['task_number_in', 'error_step_in', 'base_in'],
            output_keys = ['position_x_out', 'position_y_out']
        )
        self.rate = rospy.Rate(10)
        self.task_to_object = {
            Task.WATER_PLANTS: WaterPlantsDag.subtask_info,
            Task.TAKE_MEDS: TakeMedicationDag.subtask_info,
            Task.WALK_DOG: WalkDogDag.subtask_info
        }

    def execute(self, userdata):
        rospy.loginfo("Executing state Findobject")
        self.success = False
        self.is_running = True

        # according to the task number and error step the object needs to be queried
        if userdata.base_in == True:
            object_to_find = 'base'
        else:
            object_to_find = self.task_to_object[userdata.task_number_in][userdata.error_step_in][1]

        data = self.get_object_location(object_to_find)
        if data is not None and len(data) != 0:
            userdata.position_x_out = data[0].x
            userdata.position_y_out = data[0].y
            rospy.loginfo("Object {} found at x = {} y = {}".format(object_to_find, data[0].x, data[0].y))
            return "success"
        else:
            rospy.loginfo("Cannot retrieve {} location".format(object_to_find))
            return "fail"

    def get_object_location(self, name):
            rospy.wait_for_service("query_objects")

            try:
                query = rospy.ServiceProxy("query_objects", ObjectQuery)
                result = query(name)
                return result.locations
            except rospy.ServiceException, e:
                rospy.logerr("Service call failed: %s" % e)
            return None


class GotoXYState(smach.State):

    def __init__(self):
        smach.State.__init__(
            self,
            outcomes = ['success', 'fail', 'preempted'],
            input_keys = ['position_x_in', 'position_y_in'])

        self.rate = rospy.Rate(10)
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)

        rospy.loginfo("Waiting for the move_base server to be connected")
        self.move_base.wait_for_server(rospy.Duration(2))

    def done_cb(self, terminal_state, result):
        if terminal_state == GoalStatus.SUCCEEDED:
            self.success = True
        self.is_running = False

    def execute(self, userdata):
        rospy.loginfo("Executing State GotoXY")

        rospy.loginfo("Goto x = {} y = {}".format(userdata.position_x_in, userdata.position_y_in))
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = userdata.position_x_in
        goal.target_pose.pose.position.y = userdata.position_y_in
        # at some point we need to change this or may be depending upon the task number we can do it
        goal.target_pose.pose.orientation.w = 1.0

        self.success = False
        self.is_running = True
        self.move_base.send_goal(goal)

        start_time = rospy.Time.now()
        timeout = rospy.Duration(secs = 60)
        while self.is_running and rospy.Time.now() - start_time < timeout:
            if self.preempt_requested():
                self.service_preempt()
                return 'preempted'
            self.rate.sleep()

        if not self.success:
            self.move_base.cancel_goal()
            rospy.loginfo("GOtoXY failed")
            return 'fail'

        rospy.loginfo("GotoXY succeeded")
        return "success"

    def request_preempt(self):
        smach.State.request_preempt(self)
        rospy.logwarn("GotoXYState Preempted")


class GotoObjectSMACH():

    def __init__(self):
        pass

    def execute(self, task_number, error_step, base):
        sm = smach.StateMachine(outcomes = ['finish', 'error'])
        sm.userdata.task_number = task_number
        sm.userdata.error_step = error_step
        sm.userdata.sm_pose_x = 0
        sm.userdata.sm_pose_y = 0
        sm.userdata.base = base

        with sm:
            smach.StateMachine.add(
                'FIND_OBJECT',
                FindObjectState(),
                transitions = {
                    'success': 'GOTO_XY',
                    'fail' : 'error'},
                    remapping = {
                        'task_number_in' : 'task_number',
                        'error_step_in' : 'error_step',
                        'base_in' : 'base',
                        'position_x_out' : 'sm_pose_x',
                        'position_y_out' : 'sm_pose_y'}
            )

            smach.StateMachine.add(
                'GOTO_XY',
                GotoXYState(),
                transitions = {
                    'success': 'finish',
                    'fail' : 'error',
                    'preempted' : 'error'},
                remapping = {
                    'position_x_in' : 'sm_pose_x',
                    'position_y_in' : 'sm_pose_y'})

        outcome = sm.execute()
        return outcome


if __name__ == '__main__':
    rospy.init_node("goto_object_state_machine")
    sm_gotoobject = GotoObjectSMACH()
    sm_gotoobject.execute()
    rospy.signal_shutdown('All Done.')
