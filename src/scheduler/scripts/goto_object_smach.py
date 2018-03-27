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
from gotoxy_state import GotoXYState, get_object_location


class FindObjectState(smach.State):

    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=['success', 'fail'],
            input_keys=['task_number_in', 'error_step_in', 'base_in'],
            output_keys=['position_x_out', 'position_y_out',
                         'orientation_z_out', 'orientation_w_out']
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

        data = get_object_location(object_to_find)
        if data is not None and len(data) != 0:
            userdata.position_x_out = data[0].x
            userdata.position_y_out = data[0].y
            userdata.orientation_z_out = data[0].z
            userdata.orientation_w_out = data[0].w
            rospy.loginfo("Object {} found at x = {} y = {} z = {} w = {}".format(object_to_find, data[0].x, data[0].y,
                                                                                  data[0].z, data[0].w))
            return "success"
        else:
            rospy.loginfo("Cannot retrieve {} location".format(object_to_find))
            return "fail"

class GotoObjectSMACH():

    def __init__(self):
        pass

    def execute(self, task_number, error_step, base):
        sm = smach.StateMachine(outcomes=['finish', 'error'])
        sm.userdata.task_number = task_number
        sm.userdata.error_step = error_step
        sm.userdata.sm_pose_x = 0
        sm.userdata.sm_pose_y = 0
        sm.userdata.sm_orient_z = 0
        sm.userdata.sm_orient_w = 0
        sm.userdata.base = base

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
                    'orientation_w_out': 'sm_orient_w'}
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
                    'orientation_w_in': 'sm_orient_w'})

        outcome = sm.execute()
        return outcome


if __name__ == '__main__':
    rospy.init_node("goto_object_state_machine")
    sm_gotoobject = GotoObjectSMACH()
    sm_gotoobject.execute()
    rospy.signal_shutdown('All Done.')
