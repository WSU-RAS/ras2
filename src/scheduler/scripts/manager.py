#!/usr/bin/env python

import rospy
import math

from actionlib_msgs.msg import GoalStatus
from actionlib import SimpleActionServer, SimpleActionClient
from ras_msgs.msg import GotoAction, GotoGoal
from ras_msgs.msg import DoErrorAction, DoErrorFeedback, DoErrorResult
from ras_msgs.msg import TabletAction, TabletFeedback, TabletResult
from tablet_interface.srv import Tablet

from adl.util import Task, Goal, Status
import tf
from tf import TransformListener
from tf.transformations import euler_from_quaternion
from collections import namedtuple

Transformation = namedtuple('Transformation', ['x', 'y', 'z'])
Rotation = namedtuple('Rotation', ['roll', 'pitch', 'yaw'])

class SchedulerServer:

    def __init__(self):
        self.rate = rospy.Rate(2)
        self.task_number = 0
        self.error_step = 0

        # Tablet data
        self.object_name = ""
        self.video_step_url = ""
        self.video_full_url = ""

        self.is_goto_active = False
        self.goto_success = False
        self.goto_type = None
        self.is_error_correction_done = False
        self.is_error_corrected = False
        self.use_robot = True
        self.use_tablet = True
        if rospy.has_param("ras"):
            ras = rospy.get_param("ras")
            self.use_robot = ras['use_robot']
            self.use_tablet = ras['use_tablet']

        self.do_error = SimpleActionServer(
            'do_error', DoErrorAction,
            execute_cb=self.do_error_execute,
            auto_start=False)

        # Called from the tablet when we want to go to a particular object
        # This then forwards it to our Go To node via the self.goto_client
        if self.use_tablet:
            self.tablet = SimpleActionServer(
                'tablet_response', TabletAction,
                execute_cb=self.tablet_execute,
                auto_start=False)
            self.tablet.start()

        if self.use_robot:
            # Forward commands through our Go To node
            self.goto_client = SimpleActionClient(
                'goto', GotoAction)
            self.goto_client.wait_for_server()
            self.tf = TransformListener()
            self.robot_loc = rospy.Timer(rospy.Duration(60), self.robot_location_cb)

            # Allow returning to base from the experimenter interface
            self.goto_base = SimpleActionServer(
                'goto_base', TabletAction, # Same as action, but ignore data in message
                execute_cb=self.goto_base_execute,
                auto_start=False)
            self.goto_base.start()

        self.do_error.start()
        rospy.on_shutdown(self.shutdown)

    def shutdown(self):
        if self.is_goto_active:
            self.goto_client.cancel_goal()

    def get_robot_location(self):
        t, r = None, None
        if self.use_robot:
            try:
                (trans, rot) = self.tf.lookupTransform("/map", "/base_link", rospy.Time(0))
                t = Transformation(*trans)
                euler = euler_from_quaternion(rot)
                r = Rotation(*euler)
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                rospy.warn("manager: Cannot find /map or /base_link")
        return (t, r)

    def log_robot_location(self):
        """
        Logs location and angle(radians) of turtlebot in the map.
        Use this to publish information to CASAS when available.
        """
        trans, rot = self.get_robot_location()
        if trans != None and rot != None:
            rospy.loginfo("Turtlebot xy=({0:.3f},{1:.3f}) radian={2:.3f} degrees={3:.3f}".format(
                trans.x, trans.y, rot.yaw, (rot.yaw * 180./math.pi)))

    def robot_location_cb(self, event):
        if (not self.is_error_correction_done) or self.is_goto_active:
            self.log_robot_location()

    def do_error_execute(self, goal):
        """
        What to do when error detection calls this action server, when an error
        is detected
        """
        self.is_error_correction_done = False
        self.is_error_corrected = False
        rospy.loginfo("manager: Executing Do Error")
        self.do_error_feedback(Status.STARTED, "ERROR CORRECTION STARTED")

        self.task_number = goal.task_number
        self.error_step = goal.error_step

        # Preprocess to get correct values for the tablet
        self.object_name, self.video_step_url, self.video_full_url = \
            TabletData.get_data(self.task_number, self.error_step)

        # Step ONE:
        # Setup tablet and find the human
        if self.use_tablet:
            self.tablet_setup("choice")
        self.do_error_feedback(Status.INPROGRESS, "TABLET SETUP COMPLETED")

        if self.use_robot:
            self.goto(Goal.HUMAN)
        self.do_error_feedback(Status.INPROGRESS, "SEND GOAL TO TURTLEBOT=>FIND/GOTO HUMAN")

        # Do not proceed until goto has started
        while not self.is_goto_active and self.use_robot:
            self.rate.sleep()

        self.do_error_feedback(
            Status.INPROGRESS, "TURTLEBOT FIND/GOTO HUMAN STARTED")
        while self.is_goto_active and self.use_robot:
            self.rate.sleep()

        # Success is determined in goto done callback
        if self.goto_success and self.goto_type == Goal.HUMAN:
            self.do_error_feedback(
                Status.INPROGRESS, "TURTLEBOT WITH HUMAN")
            rospy.loginfo("manager: Found human")
        else:
            self.do_error_feedback(
                Status.FAILED, "TURTLEBOT FIND/GOTO HUMAN FAILED")
            rospy.logwarn("manager: Did not find human")
            self.do_error_feedback(
                Status.FAILED, "EXPERIMENTER NEEDS TO TELEOP ROBOT TO HUMAN")
            rospy.logwarn("manager: Experimenter needs to teleop robot to human")

        # Step TWO:
        # Human interacts with tablet human chooses no or task completed
        while not self.is_error_correction_done and self.use_tablet:
            self.rate.sleep()

        # It ONLY reaches here if Human chooses
        # No or Task completed in the tablet interface
        if self.is_error_corrected:
            complete_status = Status.COMPLETED
            self.do_error_feedback(Status.COMPLETED, "ERROR CORRECTION COMPLETED")
            rospy.loginfo("manager: Error correction completed")
        else:
            # SHOULD NEVER GET HERE!!!
            complete_status = Status.FAILED
            self.do_error_feedback(Status.FAILED, "ERROR CORRECTION FAILED")
            rospy.logerr("manager: Error correction failed!")

        do_error_result = DoErrorResult()
        do_error_result.status = complete_status
        do_error_result.is_complete = self.is_error_corrected
        self.do_error.set_succeeded(do_error_result)

    def do_error_feedback(self, status, text):
        """
        Sends feedback to the do error action server
        """
        do_error_feedback = DoErrorFeedback()
        do_error_feedback.status = status
        do_error_feedback.text = text
        self.do_error.publish_feedback(do_error_feedback)

    def tablet_setup(self, screen, showObject=True, navigateComplete=False,
            gotoObjectComplete=False):
        """
        Command the tablet to switch to a particular screen
        """
        rospy.loginfo("manager: Waiting for /tablet service")
        rospy.wait_for_service("tablet")
        rospy.loginfo("manager: Found /tablet service")

        # We don't show the go to object button when we've already navigated to it
        if navigateComplete:
            object_name = "done"
        elif showObject and self.object_name is not None:
            object_name = self.object_name
        else:
            object_name = ""

        rospy.loginfo(
            "Commanding tablet: s: %s o: %s f: %s vs: %s vf: %s",
            screen, object_name, TabletData.face_url, self.video_step_url,
            self.video_full_url)

        try:
            query = rospy.ServiceProxy("tablet", Tablet)
            results = query(
                screen, object_name, TabletData.face_url,
                self.video_step_url, self.video_full_url)
            return results.success
        except rospy.ServiceException, e:
            rospy.logerr("manager: Service call failed: {}".format(e))

        return False

    def tablet_execute(self, goal):
        """
        Handle the response from what the user clicked on the tablet
        """
        success = True
        response = goal.response
        rospy.loginfo("manager: Got message from tablet: {}".format(response))
        self.do_error_feedback(Status.INPROGRESS, "TABLET MESSAGE {}".format(response))

        if response == "no" or response == "complete":
            self.is_error_corrected = True
            self.is_error_correction_done = True
            rospy.loginfo("manager: Sending turtlebot to base")
            if self.use_robot:
                success = self.tablet_goto_execute(Goal.BASE)
        elif response == "videodone":
            # Return to the options screen
            self.tablet_setup("options")
        elif response == "goto":
            rospy.loginfo("manager: Sending turtlebot to find object")
            if self.use_robot:
                if response.object:
                    success = self.tablet_goto_execute(Goal.OBJECT)
                else:
                    success = self.tablet_goto_execute(Goal.OBJECT)
            else:
                # Fake getting to object successfully on the tablet
                self.tablet_setup("options", navigateComplete=True)

        tablet_result = TabletResult()
        tablet_result.success = success
        self.tablet.set_succeeded(tablet_result)

    def goto_base_execute(self, goal):
        """
        Return to the base for the current task
        """
        rospy.loginfo("manager: Got goto base command")
        success = False

        # Go to object
        if self.goto(Goal.BASE):
            if self.goto_success:
                success = True
                rospy.loginfo("Back at base")
                self.goto_base_feedback(Status.COMPLETED, "GOTO BASE SUCCESS")
            else:
                self.goto_base_feedback(Status.FAILED, "GOTO BASE FAILED")

        result = TabletResult()
        result.success = success
        self.goto_base.set_succeeded(result)

    def tablet_goto_execute(self, goal_type=Goal.OBJECT):
        """
        Execute goto object and base triggered by human-tablet interaction
        """
        err_msg = "Unknown error"

        # Find the object
        if self.goto(goal_type):
            if goal_type == Goal.OBJECT:
                msg = "Found object"
                err_msg = "Did not find object"

                self.tablet_feedback(Status.STARTED, "SEND GOAL TO TURTLEBOT=>GO TO OBJECT")
                self.do_error_feedback(Status.INPROGRESS, "SEND GOAL TO TURTLEBOT=>GO TO OBJECT")

            elif goal_type == Goal.BASE:
                msg = "Back at base"
                err_msg = "Did not get back at base"
                self.tablet_feedback(Status.STARTED, "SEND GOAL TO TURTLEBOT=>GO TO BASE")
                self.do_error_feedback(Status.INPROGRESS, "SEND GOAL TO TURTLEBOT=>GO TO BASE")

                # Leave at default screen, don't show the options

            while not self.is_goto_active:
                self.rate.sleep()

            self.tablet_feedback(Status.INPROGRESS, "TURTLEBOT NAVIGATING")
            self.do_error_feedback(Status.INPROGRESS, "TURTLEBOT NAVIGATING")

            while self.is_goto_active:
                self.rate.sleep()

            if self.goto_success:
                rospy.loginfo(msg)

                self.tablet_feedback(Status.COMPLETED, "TURTLEBOT COMPLETED TASK")
                self.do_error_feedback(Status.INPROGRESS, "TURTLEBOT COMPLETED TASK")

                # After leading to object, play the "okay, here you go" sound
                if goal_type == Goal.OBJECT:
                    self.tablet_setup("options", navigateComplete=True)

                return True
            else:
                # If failure, just output the options without the go to button
                if goal_type == Goal.OBJECT:
                    self.tablet_setup("options", showObject=False)

        rospy.logerr(err_msg)

        self.tablet_feedback(Status.FAILED, "TURTLEBOT FAILED")
        self.do_error_feedback(Status.INPROGRESS, "TURTLEBOT FAILED")

        return False

    def tablet_feedback(self, status, text):
        """
        Sends feedback to the tablet action server
        """
        tablet_feedback = TabletFeedback()
        tablet_feedback.status = status
        tablet_feedback.text = text
        self.tablet.publish_feedback(tablet_feedback)

    def goto_base_feedback(self, status, text):
        """
        Sends feedback to the goto base action server
        """
        feedback = TabletFeedback()
        feedback.status = status
        feedback.text = text
        self.goto_base.publish_feedback(feedback)

    def goto(self, goto_type):
        """
        From the specified server, try going to the specified object
        """
        if self.is_goto_active:
            rospy.logerr("manager: Cannot navigate to two places at once!")
            return False

        self.goto_success = False
        self.goto_type = goto_type

        goto_goal = GotoGoal()
        goto_goal.type = goto_type
        goto_goal.task_number = self.task_number
        goto_goal.error_step = self.error_step
        goto_goal.error_object = self.error_step
        self.goto_client.send_goal(
            goto_goal,
            done_cb=self.goto_done_cb,
            active_cb=self.goto_active_cb,
            feedback_cb=self.goto_feedback_cb)

        return True

    def goto_done_cb(self, terminal_state, result):
        result_bool = result.status == Status.COMPLETED
        if terminal_state == GoalStatus.SUCCEEDED and result_bool:
            self.goto_success = True
            status = "SUCCEEDED"
        elif terminal_state == GoalStatus.PREEMPTED:
            self.goto_success = False
            status = "PREMEPTED"
        elif terminal_state == GoalStatus.ABORTED:
            self.goto_success = False
            status = "ABORTED"
        else:
            self.goto_success = False
            status = "UNKNOWN"

        self.is_goto_active = False
        rospy.loginfo("manager: terminal state: {}  result: ({}, {})".format(
            status, result.status, result.is_complete))
        self.log_robot_location()

    def goto_active_cb(self):
        self.is_goto_active = True
        rospy.loginfo("manager: goto is active")
        self.log_robot_location()

    def goto_feedback_cb(self, feedback):
        rospy.loginfo("manager: goto feedback: status={}".format(
            feedback.text))
        self.log_robot_location()


class TabletData(object):
    face_url = "happy-cartoon-face-hi.png"

    @staticmethod
    def get_data(task_number, error_step):
        """
        From the task number and error step, get the corresponding object name
        and video urls
        """
        object_to_find = ''  # No object for this error / step
        video_step_url = ''
        video_full_url = ''

        if task_number == Task.WATER_PLANTS:
            video_full_url = 'waterplants.all.mp4'
            if error_step in [0, 1, 4, 5]:
                video_step_url = 'waterplants.error1.mp4'
                object_to_find = 'watercan'
            elif error_step == 2:
                video_step_url = 'waterplants.error2.mp4'
                object_to_find = 'plantcoffee'
            elif error_step == 3:
                video_step_url = 'waterplants.errors.mp4' # Name should be error3
                object_to_find = "plantside"

        elif task_number == Task.TAKE_MEDS:
            video_full_url = 'takemedication.all.mp4'
            if error_step in [0, 5, 11]:
                video_step_url = 'takemedication.error1.mp4'
                object_to_find = 'food'
            elif error_step in [1, 2, 7, 10]:
                video_step_url = 'takemedication.error2.mp4'
                object_to_find = 'glass'
            elif error_step in [3, 6, 9]:
                video_step_url = 'takemedication.error3.mp4'
                object_to_find = 'pillbottle'

        elif task_number == Task.WALK_DOG:
            video_full_url = 'walkdog.all.mp4'
            if error_step == 0:
                video_step_url = 'walkdog.error1.mp4'
                object_to_find = 'umbrella'
            elif error_step == 1:
                video_step_url = 'walkdog.error2.mp4'
                object_to_find = 'leash'
            elif error_step == 2:
                video_step_url = 'walkdog.error3.mp4'
                object_to_find = 'keys'
            elif error_step == 3:
                video_step_url = 'walkdog.error3.mp4' # TODO error4 video does not exist...
                object_to_find = 'dog'

        return object_to_find, video_step_url, video_full_url


if __name__ == '__main__':
    try:
        rospy.init_node('scheduler_server')
        server = SchedulerServer()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
