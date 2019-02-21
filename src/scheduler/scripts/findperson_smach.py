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
#from gotoxy_state import GotoXYState, get_object_location
from gotoxy_state_seq import GotoXYState, get_object_location, multi_path, Goto_points
from geometry_msgs.msg import Pose, Point, Quaternion
from adl.util import Task

class FindPersonState(smach.State):

    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=['success', 'fail', 'preempted'],
            input_keys=['task_number_in', 'error_step_in'],
            output_keys=['position_x_out', 'position_y_out', 'orientation_z_out', 'orientation_w_out', 'points_out', 'object_name_out']
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
        userdata.object_name_out = 'human_found'

        goal = FindPersonGoal()
        goal.task_number = userdata.task_number_in
        goal.error_step = userdata.error_step_in

        self.success = False
        self.is_running = True
        self.find_person.send_goal(goal, done_cb=self.done_cb)

        start_time = rospy.Time.now()
        timeout = rospy.Duration(secs=5, nsecs=0)
        while self.is_running and rospy.Time.now() - start_time < timeout:
            if self.preempt_requested():
                self.service_preempt()
                userdata.object_name_out = 'human_found' # Not sure?
                return 'preempted'
            self.rate.sleep()

        if not self.success:
            self.find_person.cancel_goal()
            rospy.logwarn("FindPerson failed")
            userdata.object_name_out = 'human_notfound'
            return "fail"

        # Find intermediate points based on previous location
        points = multi_path(FindPersonSMACH.last_object, "")

        # Result of querying find_person, add person coords to goal list
        result = self.find_person.get_result()
        points.append((result.x, result.y, result.z, result.w))

        rospy.loginfo("Last object: {}, Points: {}".format(FindPersonSMACH.last_object, points))

        userdata.points_out = points
        FindPersonSMACH.last_object = ""

        '''
        userdata.position_x_out = result.x
        userdata.position_y_out = result.y
        userdata.orientation_z_out = result.z
        userdata.orientation_w_out = result.w
        userdata.points_out = []
        rospy.loginfo("Person found at x={} y={} z={} w={}".format(result.x, result.y,
            result.z, result.w))
        '''
        return "success"

    def request_preempt(self):
        smach.State.request_preempt(self)
        rospy.logwarn("FindPersonState Preempted!")



class FindPersonSMACH():
    #This is probably wrong, so commented out -Chris
    #last_object = "base1"

    def __init__(self, last_object):
        FindPersonSMACH.last_object = last_object

    def execute(self, task_number=2, error_step=3):
        # SMACH State Machine
        sm = smach.StateMachine(outcomes = ['finish', 'error'])
        sm.userdata.task_number = task_number
        sm.userdata.error_step = error_step
        sm.userdata.sm_pose_x = 0
        sm.userdata.sm_pose_y = 0
        sm.userdata.sm_orient_z = 0
        sm.userdata.sm_orient_w = 0

        sm.userdata.sm_last_object = FindPersonSMACH.last_object
        sm.userdata.sm_points = []
        sm.userdata.sm_object_name = "" # if "human", look up via task number / error step

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
                    'object_name_out': 'sm_object_name',
                    'points_out': 'sm_points'
                })
            StateMachine.add(
                'FIND_PERSON2',
                FindPersonState(),
                transitions={
                    'success': 'GOTO_XY',
                    'fail': 'error',
                    'preempted': 'error'},
                remapping={
                    'task_number_in': 'task_number',
                    'error_step_in': 'error_step',
                    'object_name_out': 'sm_object_name',
                    'points_out': 'sm_points'
                })
            StateMachine.add(
                'GOTONEWBASE',
                GotoXYState(),
                transitions = {
                        'success' : 'FIND_PERSON2',
                        'fail' : 'error',
                        'preempted' : 'error'},
                remapping = {
                        'last_object_in' : 'sm_last_object',
                        'task_number_in' : 'task_number',
                        'error_step_in' : 'error_step',
                        'object_name_in' : 'sm_object_name',
                        'last_object_out' : 'sm_last_object',
                        'points_in': 'sm_points'
                })
            StateMachine.add(
                'GOTO_XY',
                GotoXYState(),
                transitions = {
                        'success' : 'finish',
                        'fail' : 'error',
                        'preempted' : 'error'},
                remapping = {
                        'last_object_in' : 'sm_last_object',
                        'task_number_in' : 'task_number',
                        'error_step_in' : 'error_step',
                        'object_name_in' : 'sm_object_name',
                        'last_object_out' : 'sm_last_object',
                        'points_in': 'sm_points'
                })

        sis = smach_ros.IntrospectionServer('FindPersonSMACH', sm, '/FindPersonSMACH')
        sis.start()
        outcome =  sm.execute()
        FindPersonSMACH.last_object = sm.userdata.sm_last_object
        return outcome

if __name__ == '__main__':
    rospy.init_node("find_person_state_machine")
    sm_findperson = FindPersonSMACH()
    sm_findperson.execute()
    rospy.signal_shutdown('All done.')
