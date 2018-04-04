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
            output_keys=['position_x_out', 'position_y_out', 'orientation_z_out', 'orientation_w_out', 'points_out']
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


class GotoNewBaseState(smach.State):
    def __init__(self):
        smach.State.__init__(self,
            outcomes = ['success', 'fail', 'preempted'],
            input_keys = ['task_number_in', 'error_step_in'],
            output_keys = ['task_number_out', 'error_step_out']
        )
        self.rate = rospy.Rate(10)

        rospy.loginfo("GotoNewBaseState has been initialized, Waiting for the move_base action server")       


    def active_cb(self):
        rospy.loginfo("Goal pose "+str(self.goal_cnt+1)+" is now being processed by the Action Server...")

    def feedback_cb(self, feedback):
        pass

    def done_cb(self, status, result):
        self.goal_cnt += 1

        if status == GoalStatus.PREEMPTED:
            rospy.loginfo("Goal pose "+str(self.goal_cnt)+" received a cancel request after it started executing, completed execution!")

        elif status == GoalStatus.SUCCEEDED:
            rospy.loginfo("Goal pose "+str(self.goal_cnt)+" reached")
            if self.goal_cnt < len(self.pose_seq):
                self.movebase_client()
                return
            else:
                rospy.loginfo("Final goal pose reached!")
                rospy.signal_shutdown("Final goal pose reached!")
                self.success = True

        elif status == GoalStatus.ABORTED:
            rospy.loginfo("Goal pose "+str(self.goal_cnt)+" was aborted by the Action Server")
            rospy.signal_shutdown("Goal pose "+str(self.goal_cnt)+" aborted, shutting down!")

        elif status == GoalStatus.REJECTED:
            rospy.loginfo("Goal pose "+str(self.goal_cnt)+" has been rejected by the Action Server")
            rospy.signal_shutdown("Goal pose "+str(self.goal_cnt)+" rejected, shutting down!")

        elif status == GoalStatus.RECALLED:
            rospy.loginfo("Goal pose "+str(self.goal_cnt)+" received a cancel request before it started executing, successfully cancelled!")

        self.is_running = False

    def movebase_client(self):
        rospy.loginfo("Goal cnt: {}, pose_seq: {}".format(self.goal_cnt, self.pose_seq))

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = self.pose_seq[self.goal_cnt]
        rospy.loginfo("Sending goal pose "+str(self.goal_cnt+1)+" to Action Server")
        rospy.loginfo(str(self.pose_seq[self.goal_cnt]))
        self.move_base.send_goal(goal, self.done_cb, self.active_cb, self.feedback_cb)

    def execute(self, userdata):
        ''' -Chris_test
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        self.move_base.wait_for_server(rospy.Duration(2))
        '''

        rospy.loginfo("Executing state Goto New Base")

        # walk dog task
        data = None
        if userdata.task_number_in == Task.WALK_DOG:
            if userdata.error_step_in in [1, 2, 3, 4]:
                object_to_find = 'entryway'
        # watering the plants
        elif userdata.task_number_in == Task.WATER_PLANTS:
            # error step in filling
            if userdata.error_step_in == 1:
                object_to_find = 'kitchen'

        data = multi_path(settings.last_object, object_to_find)
        settings.last_object = object_to_find

        ''' -Chris_test
        # multi-points
        self.pose_seq = list()
        self.goal_cnt = 0

        for point in data:
            self.pose_seq.append(Pose(Point(point[0],point[1],0), Quaternion(0,0,point[2],point[3])))

        # run it
        self.success = False
        self.is_running = True
        self.movebase_client()
        ''' 

        #Chris_test replacement code
        args = [self.is_running, self.success, data]
        Goto_points(args)

        start_time = rospy.Time.now()
        timeout = rospy.Duration(secs=120, nsecs=0)
        while self.is_running and rospy.Time.now() - start_time < timeout:
            if self.preempt_requested():
                self.service_preempt()
                return 'preempted'
            self.rate.sleep()

        if not self.success:
            self.move_base.cancel_goal()
            rospy.loginfo("GotoXYNewBase failed")
            return "fail"

        userdata.task_number_out = userdata.task_number_in
        userdata.error_step_out = userdata.error_step_in
        rospy.loginfo("GotoXYNewBase succeeded")
        return "success"


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

        sm.userdata.sm_points = []

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
                    'orientation_w_out' : 'sm_orient_w',
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
                    'position_x_out': 'sm_pose_x',
                    'position_y_out': 'sm_pose_y',
                    'orientation_z_out' : 'sm_orient_z',
                    'orientation_w_out' : 'sm_orient_w',
                    'points_out': 'sm_points'
                })
            StateMachine.add(
                'GOTONEWBASE',
                GotoNewBaseState(),
                transitions = {
                        'success' : 'FIND_PERSON2',
                        'fail' : 'error',
                        'preempted' : 'error'},
                remapping = {
                        'task_number_in' : 'task_number',
                        'error_step_in' : 'error_step',
                        'task_number_out': 'task_number',
                        'error_step_out': 'error_step'
                })

            StateMachine.add(
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
                    'orientation_w_in' : 'sm_orient_w',
                    'points_in': 'sm_points'
                })
        sis = smach_ros.IntrospectionServer('FindPersonSMACH', sm, '/FindPersonSMACH')
        sis.start()
        outcome =  sm.execute()
        return outcome


if __name__ == '__main__':
    rospy.init_node("find_person_state_machine")
    sm_findperson = FindPersonSMACH()
    sm_findperson.execute()
    rospy.signal_shutdown('All done.')
