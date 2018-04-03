#!/usr/bin/env python

import rospy
import smach
import smach_ros
import time
import actionlib_msgs
import actionlib

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus

from adl.util import Task, TaskToDag
#from gotoxy_state import GotoXYState, get_object_location
from gotoxy_state_seq import GotoXYState, get_object_location, multi_path


class FindObjectState(smach.State):

    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=['success', 'fail'],
            input_keys=['task_number_in', 'error_step_in', 'base_in'],
            output_keys=['position_x_out', 'position_y_out',
                         'orientation_z_out', 'orientation_w_out', 'points_out']
        )
        self.rate = rospy.Rate(10)

    def execute(self, userdata):
        rospy.loginfo("Executing state Findobject")
        self.success = False
        self.is_running = True

        # according to the task number and error step the object needs to be queried
        if userdata.base_in == True:
            # if the task is water plants and take meds then the base is BASE2
            if userdata.task_number_in in [0, 1]:
                object_to_find = 'base2'

            # if the task is walk the dog then the base is BASE1
            elif userdata.task_number_in == 2:
                object_to_find = 'base1'

            # if the task is walk the dog and error step is take the leash then the base is BASE3
            elif userdata.task_number_in == 2 and userdata.error_step_in == 6:
                object_to_find = 'base3'


        else:
            object_to_find = TaskToDag.mapping[userdata.task_number_in].subtask_info[userdata.error_step_in][1]

        path_goals = multi_path(GotoObjectSMACH.last_object, object_to_find)
        # Tracking last object
        GotoObjectSMACH.last_object = object_to_find
        userdata.points = path_goals

        return "success"


class GotoObjectSMACH():
    last_object = "base1"

    def __init__(self, last_object):
        GotoObjectSMACH.last_object = last_object

    def execute(self, task_number, error_step, base):
        sm = smach.StateMachine(outcomes=['finish', 'error'])
        sm.userdata.task_number = task_number
        sm.userdata.error_step = error_step
        sm.userdata.sm_pose_x = 0
        sm.userdata.sm_pose_y = 0
        sm.userdata.sm_orient_z = 0
        sm.userdata.sm_orient_w = 0
        sm.userdata.base = base
        # Chris' points
        sm.userdata.sm_points = []

        with sm:
            smach.StateMachine.add(
                'FIND_OBJECT',
                FindObjectState(),
                transitions={
                    'success': 'GOTO_XY',
                    'fail': 'error'},
                remapping={
                    'task_number_in': 'task_number',
                    'error_step_in': 'error_step',
                    'base_in': 'base',
                    'position_x_out': 'sm_pose_x',
                    'position_y_out': 'sm_pose_y',
                    'orientation_z_out': 'sm_orient_z',
                    'orientation_w_out': 'sm_orient_w',
                    'points_out': 'sm_points'}
            )

            smach.StateMachine.add(
                'GOTO_XY',
                GotoXYState(),
                transitions={
                    'success': 'finish',
                    'fail': 'error',
                    'preempted': 'error'},
                remapping={
                    'position_x_in': 'sm_pose_x',
                    'position_y_in': 'sm_pose_y',
                    'orientation_z_in': 'sm_orient_z',
                    'orientation_w_in': 'sm_orient_w',
                    'points_in': 'sm_points'}
            )

        outcome = sm.execute()
        return outcome


if __name__ == '__main__':
    rospy.init_node("goto_object_state_machine")
    sm_gotoobject = GotoObjectSMACH()
    sm_gotoobject.execute()
    rospy.signal_shutdown('All Done.')
