#!/usr/bin/env python

import rospy
import actionlib_msgs
import actionlib
import smach
import smach_ros


from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus

from object_detection_msgs.srv import ObjectQuery, ObjectQueryResponse
import time


class FindObjectState(smach.State):

    def getObjectLocation(name):
        try:
            query = rospy.ServiceProxy("query_object", ObjectQuery)
            result = query(name)
            return result.locations
        except rospy.ServiceException, e:
            rospy.roserr("Service call failed: %s" % e)
            return None


    def __init__(self):
        smach.State.__init__(
            self,
            outcomes = ['success', 'fail']
            input_keys = ['task_number_in', 'error_step_in']
            output_keys = ['position_x_out', 'position_y_out']
        )
        self.rate = rospy.Rate(10)
    #
    # def done_cb(self, terminal_state, result):
    #     if terminal_state = GoalStatus.SUCCEEDED and result.found:
    #         self.success = True
    #     self.is_running = False

    def execute(self, userdata):
        rospy.loginfo("Executing state Findobject")
        self.success = False
        self.is_running = True

        # define the goal
        # according to the task number and error step the object needs to be queried
        # to map these task numbers with the objects to find

        # Water plants
        if userdata.task_number_in == 0:
            if userdata.error_step_in == 0 or 1 or 4 or 5:
                object_to_find = 'watercan'
            elif userdata.error_step_in == 2:
                object_to_find = 'plantcoffee'
            elif userdata.error_step_in == 3:
                object_to_find = "plantside"
        # Take Meds
        elif userdata.task_number_in == 1:
            if userdata.error_step_in == 0 or 5 or 11:
                object_to_find = 'food'
            elif userdata.error_step_in == 1 or 2 or 7 or 10:
                object_to_find = 'glass'
            elif userdata.error_step_in == 3 or 6 or 9:
                object_to_find = 'pillbottle'
        # Walk the dog
        elif userdata.task_number_in == 2:
            if userdata.error_step_in == 0:
                object_to_find = 'umbrella'
            elif userdata.error_step_in == 1:
                object_to_find = 'leash'
            elif userdata.error_step_in == 2:
                object_to_find = 'keys'
            elif userdata.error_step_in == 3:
                object_to_find = 'dog'


        data = self.getObjectLocation(object_to_find)

        if (data.locations != 0):
            userdata.position_x_out = data[0].x
            userdata.position_y_out = data[0].y
            rospy.loginfo("Object {} found at x = {} y = {}".format(object_to_find, data[0].x, data[0].y))
            return "success"
        else:
            rospy.loginfo("Cannot retrieve {} location".format(object_to_find))
            return "fail"

class GotoXYState(smach.State):

    def __init__(self):
        smach.State.__init__(
            self,
            outcomes = ['success', 'fail', 'preempted'],
            input_keys = ['position_x_in', 'position_y_in'])

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
        goal = MoveBaseGoal
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = userdata.position_x_in
        goal.target_pose.pose.position.y = userdata.position_y_in
        # at some point we need to change this or may be depending upon the task number we can do it
        goal.target_pose.pose.orientation.w = 1.0

        self.success = False
        self.is_running = True
        self.move_base.send_goal(goal)

        start_time = rospy,Time.now()
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

    def execute(self, task_number, error_step):

        sm = smach.StateMachine(outcomes = ['finish', 'error'])
        sm.userdata.task_number = task_number
        sm.userdata.error_step = error_step
        sm.userdata.sm_pose_x = 0
        sm.userdata.sm_pose_y = 0

        with sm:
            StateMachine.add(
                'FIND_OBJECT',
                FindObjectState(),
                transitions = {
                    'success': 'GOTO_XY',
                    'fail' : 'error'},
                    remapping = {
                        'task_number_in' : 'task_number'
                        'error_step_in' : 'error_step'
                        'position_x_out' : 'sm_pose_x',
                        'position_y_out' : 'sm_pose_y'}
            )

            smach.StateMachine.add(
                'GOTO_XY',
                GotoXYState(),
                transitions = {
                    'success': 'finish'
                    'fail' : 'error'
                    'preempted' : 'error'},
                remapping = {
                    'position_x_in' : 'sm_pose_x'
                    'position_y_in' : 'sm_pose_y'})

                    outcome = sm.execute()
                    return outcome

if __name__ == '__main__':
    rospy.init_node("goto_object_state_machine")
    sm_gotoobject = GotoObjectSMACH()
    sm_gotoobject.execute()
    rospy.signal_shutdown('All Done.')
