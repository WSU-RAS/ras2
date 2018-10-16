#!/usr/bin/env python

import rospy
import smach
import smach_ros
import time
import actionlib_msgs
import actionlib

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus

from adl.util import TaskToDag
from gotoxy_state_seq import GotoXYState, get_object_location, multi_path


class FindObjectState(smach.State):

    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=['success', 'fail'],
            input_keys=['task_number_in', 'error_step_in', 'base_in', 'human_in'],
            output_keys=['position_x_out', 'position_y_out',
                         'orientation_z_out', 'orientation_w_out', 'points_out',
                         'object_name_out']
        )
        self.use_location = False
        if rospy.has_param("adl"):
            adl = rospy.get_param("adl")
            self.use_location = adl['use_location']
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

        elif userdata.human_in == True:
            object_to_find = 'h_' + str(userdata.task_number_in) + '_' + str(userdata.error_step_in)
        else:
            object_to_find = TaskToDag.mapping[userdata.task_number_in][1 if self.use_location else 0].subtask_info[userdata.error_step_in][1]

        # TODO we don't even use these points but re-run this in the
        # GotoXYState based on object_to_find?
        path_goals = multi_path(GotoObjectSMACH.last_object, object_to_find)
        # Tracking last object
        GotoObjectSMACH.last_object = object_to_find
        userdata.points_out = path_goals
        userdata.object_name_out = object_to_find

        return "success"


class GotoObjectSMACH():
    last_object = "base1"

    def __init__(self, last_object):
        GotoObjectSMACH.last_object = last_object

    def execute(self, task_number, error_step, base,human):
        sm = smach.StateMachine(outcomes=['finish', 'error'])
        sm.userdata.task_number = task_number
        sm.userdata.error_step = error_step
        sm.userdata.base = base
        sm.userdata.human = human
        sm.userdata.sm_last_object = GotoObjectSMACH.last_object
        sm.userdata.sm_points = []
        sm.userdata.sm_object_name = ""

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
                    'human_in': 'human',
                    'points_out': 'sm_points',
                    'object_name_out' : 'sm_object_name'
                    }
            )

            smach.StateMachine.add(
                'GOTO_XY',
                GotoXYState(),
                transitions={
                    'success': 'finish',
                    'fail': 'error',
                    'preempted': 'error'},
                remapping={
                    'last_object_in' : 'sm_last_object',
                    'task_number_in' : 'task_number',
                    'error_step_in' : 'error_step',
                    'points_in': 'sm_points',
                    'object_name_in' : 'sm_object_name',
                    'last_object_out' : 'sm_last_object',
                    'points_in': 'sm_points'
                    }
            )
        sis = smach_ros.IntrospectionServer('GotoObjectSMACH', sm, '/GotoObjectSMACH')
        sis.start()

        outcome = sm.execute()
        GotoObjectSMACH.last_object = sm.userdata.sm_last_object
        return outcome


if __name__ == '__main__':
    rospy.init_node("goto_object_state_machine")
    sm_gotoobject = GotoObjectSMACH()
    sm_gotoobject.execute()
    rospy.signal_shutdown('All Done.')
