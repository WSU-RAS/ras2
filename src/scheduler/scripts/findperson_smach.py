#!/usr/bin/env python

import rospy
import smach
import smach_ros
import actionlib

from smach import StateMachine
from smach_ros import SimpleActionState

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus
from find_person.msg import FindPersonAction, FindPersonGoal
from gotoxy_state import GotoXYState, get_object_location


class FindPersonState(smach.State):

    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=['success', 'fail', 'preempted'],
            input_keys=['task_number_in', 'error_step_in'],
            output_keys=['position_x_out', 'position_y_out', 'orientation_z_out', 'orientation_w_out']
        )
        self.rate = rospy.Rate(10)
        self.find_person = actionlib.SimpleActionClient(
            "find_person_server", FindPersonAction)

        rospy.loginfo("Waiting for the find_person_server action server")
        self.find_person.wait_for_server(rospy.Duration(2))

    def done_cb(self, terminal_state, result):
        if terminal_state == GoalStatus.SUCCEEDED and result.found:
            self.success = True
        self.is_running = False

    def execute(self, userdata):
        rospy.loginfo('Executing state FindPerson')

        goal = FindPersonGoal()
        goal.task_number = userdata.task_number_in
        goal.error_step = userdata.error_step_in

        self.success = False
        self.is_running = True
        self.find_person.send_goal(goal, done_cb=self.done_cb)

        start_time = rospy.Time.now()
        timeout = rospy.Duration(secs=60, nsecs=0)
        while self.is_running and rospy.Time.now() - start_time < timeout:
            if self.preempt_requested():
                self.service_preempt()
                return 'preempted'
            self.rate.sleep()

        if not self.success:
            self.find_person.cancel_goal()
            rospy.logwarn("FindPerson failed")
            return "fail"

        result = self.find_person.get_result()
        userdata.position_x_out = result.x
        userdata.position_y_out = result.y
        userdata.orientation_z_out = result.z
        userdata.orientation_w_out = result.w
        rospy.loginfo("Person found at x={} y={} z={} w={}".format(result.x, result.y,
            result.z, result.w))
        return "success"

    def request_preempt(self):
        smach.State.request_preempt(self)
        rospy.logwarn("FindPersonState Preempted!")


class GotoNewBaseState(smach.State):
    def __init__(self):
        smach.State.__init__(self,
            outcomes = ['success', 'fail', 'preempted'],
            input_keys = ['task_number_in', 'error_step_in']
        )
        self.rate = rospy.Rate(10)
        self.find_person = actionlib.SimpleActionClient(
            "find_person_server", FindPersonAction)
        rospy.loginfo("Waiting for the find_person_server action server")
        self.find_person.wait_for_server(rospy.Duration(2))

    def done_cb(self, terminal_state, result):

        if terminal_state == GoalStatus.SUCCEEDED:
            self.success = True
        self.is_running = False

    def execute(self, userdata):
        rospy.loginfo("Executing state Goto New Base")

        goal = FindPersonGoal()
        goal.task_number = userdata.task_number_in
        goal.error_step = userdata.error_step_in

        self.rate = rospy.Rate(10)
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)

        rospy.loginfo("Waiting for the move_base action server")
        self.move_base.wait_for_server(rospy.Duration(2))
        # walk dog task
        data = None
        if userdata.task_number_in == 2:
            if userdata.error_step_in in [1, 2, 3, 4]:
                object_to_find = 'entryway'
                # watering the plants
        elif userdata.task_number_in == 0:
            # error step in filling
            if userdata.error_step_in == 1:
                object_to_find = 'kitchen'

        data = get_object_location(object_to_find)

        if data is not None and len(data) != 0:
            rospy.loginfo("Moving Robot to new base to locate person")
            rospy.loginfo("Moving Robot to {} x = {} y = {} z = {} w = {}".format(object_to_find, data[0].x, data[0].y, data[0].z, data[0].w))
            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = "map"
            goal.target_pose.header.stamp = rospy.Time.now()
            goal.target_pose.pose.position.x = data[0].x
            goal.target_pose.pose.position.y = data[0].y
            goal.target_pose.pose.orientation.z = data[0].z
            goal.target_pose.pose.orientation.w = data[0].w

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
                    rospy.loginfo("GotoNewBase failed")
                    return "fail"

                rospy.loginfo("GotoNewBase succeeded")
                return "success"
            else:
                rospy.loginfo("Cannot retreive the location of base")
                return "fail"


class FindPersonSMACH():

    def __init__(self):
        pass

    def execute(self, task_number=2, error_step=3):
        # SMACH State Machine
        sm = smach.StateMachine(outcomes = ['finish', 'error'])
        sm.userdata.task_number = task_number
        sm.userdata.error_step = error_step
        sm.userdata.sm_pose_x = 0
        sm.userdata.sm_pose_y = 0
        sm.userdata.sm_orient_z = 0
        sm.userdata.sm_orient_w = 0

        # container having all the states
        with sm:
            StateMachine.add(
                'FIND_PERSON',
                FindPersonState(),
                transitions={
                    'success': 'GOTO_XY',
                    'fail': 'GOTONEWBASE',
                    'preempted': 'error'},
                remapping={
                    'task_number_in': 'task_number',
                    'error_step_in': 'error_step',
                    'position_x_out': 'sm_pose_x',
                    'position_y_out': 'sm_pose_y',
                    'orientation_z_out' : 'sm_orient_z',
                    'orientation_w_out' : 'sm_orient_w'
                })
            sm.add(
                'GOTONEWBASE',
                GotoNewBaseState(),
                transitions = {
                        'success' : 'FIND_PERSON',
                        'fail' : 'error',
                        'preempted' : 'error'},
                remapping = {
                        'task_number_in' : 'task_number',
                        'error_step_in' : 'error_step'
                })

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
                    'orientation_z_in' : 'sm_orient_z',
                    'orientation_w_in' : 'sm_orient_w'
                })

        outcome =  sm.execute()
        return outcome


if __name__ == '__main__':
    rospy.init_node("find_person_state_machine")
    sm_findperson = FindPersonSMACH()
    sm_findperson.execute()
    rospy.signal_shutdown('All done.')
