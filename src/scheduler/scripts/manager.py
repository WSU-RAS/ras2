#!/usr/bin/env python

import rospy

from actionlib_msgs.msg import GoalStatus
from actionlib import SimpleActionServer, SimpleActionClient
from ras_msgs.msg import GotoAction, GotoGoal
from ras_msgs.msg import DoErrorAction, DoErrorFeedback, DoErrorResult
from ras_msgs.msg import TabletGotoAction, TabletGotoFeedback, TabletGotoResult
from tablet_interface.srv import Tablet

from adl.util import Task, Goal


class Status:
    STARTED = 1
    INPROGRESS = 2
    COMPLETED = 3
    FAILED = 4


class SchedulerServer:
    
    def __init__(self):
        self.is_goto_active = False
        self.goto_success = False
        self.rate = rospy.Rate(10)

        rospy.on_shutdown(self.shutdown)

        self.do_error = SimpleActionServer(
            'do_error', DoErrorAction,
            execute_cb=self.do_error_execute,
            auto_start=False)

        # Called from the tablet when we want to go to a particular object
        # This then forwards it to our Go To node via the self.goto_client
        self.tablet = SimpleActionServer(
            'tablet_response', TabletGotoAction,
            execute_cb=self.tablet_execute,
            auto_start=False)
        self.tablet.start()

        # Forward commands through our Go To node
        self.goto_client = SimpleActionClient(
            'goto', GotoAction)
        self.do_error.start()
        self.goto_client.wait_for_server()

        self.tablet.start()
        self.do_error_start()

        # Tablet data
        self.task_number = 0
        self.error_step = 0
        self.object_name = ""
        self.video_step_url = ""
        self.video_full_url = ""
        self.is_task_error_completed = True
        self.is_robot_at_base = True

    def shutdown(self):
        if self.is_goto_active:
            self.goto_client.cancel_goal()

    def do_error_execute(self, goal):
        """
        What to do when error detection calls this action server, when an error
        is detected
        """
        self.is_task_error_completed = False
        self.is_error_correction_success = True
        rospy.loginfo("Executing Do Error")
        self.do_error_feedback(Status.STARTED, "ERROR CORRECTION STARTED")

        self.task_number = goal.task_number
        self.error_step = goal.error_step

        # Preprocess to get correct values for the tablet
        self.object_name, self.video_step_url, self.video_full_url = \
            TabletData.get_data(self.task_number, self.error_step)

        while not self.is_task_error_completed:
            break

        # Step ONE: 
        # Setup tablet and find the human
        self.tablet_setup("choice")
        self.do_error_feedback(Status.INPROGRESS, "TABLET SETUP COMPLETED")
        self.goto(Goal.HUMAN)
        self.do_error_feedback(Status.INPROGRESS, "TURTLEBOT GOAL TO GOTO HUMAN")

        # Do not proceed until goto has started
        while not self.is_goto_active:
            self.rate.sleep()

        while self.is_goto_active:
            self.do_error_feedback(
                Status.INPROGRESS, "TURTLEBOT NAVIGATING TOWARDS HUMAN")
            self.rate.sleep()

        # Success is determined in goto done callback
        if self.goto_success:
            self.do_error_feedback(
                Status.INPROGRESS, "TURTLEBOT WITH HUMAN")
            rospy.loginfo("Found human")
        else:
            self.is_task_error_completed = True
            self.is_error_correction_success = False
            self.do_error_feedback(
                Status.FAILED, "TURTLEBOT GOAL TO GOTO HUMAN FAILED")
            rospy.logerr("Did not find human")

        # Step TWO:
        # Human interacts with tablet human chooses no or task completed
        while not self.is_task_error_completed:
            self.rate.sleep()

        # It ONLY reaches here if Human chooses
        # No or Task completed in the tablet interface
        if self.is_error_correction_success:
            complete_status = Status.COMPLETED
            self.do_error_feedback(Status.COMPLETED, "ERROR CORRECTION COMPLETED")
        else:
            complete_status = Status.FAILED
            self.do_error_feedback(Status.FAILED, "ERROR CORRECTIONF FAILED")

        do_error_result = DoErrorResult()
        do_error_result.status = complete_status
        do_error_result.is_complete = True
        self.do_error.set_succeeded(do_error_result)

    def do_error_feedback(self, status, text):
        """
        Sends feedback to the do error action server
        """
        do_error_feedback = DoErrorFeedback()
        do_error_feedback.status = status
        do_error_feedback.text = text 
        self.do_error.publish_feedback(do_error_feedback)

    def tablet_setup(self, screen):
        """
        Command the tablet to switch to a particular screen
        """
        rospy.wait_for_service("tablet")

        rospy.loginfo(
            "Commanding tablet: s: %s o: %s f: %s vs: %s vf: %s a: %s",
            screen, self.object_name, TabletData.face_url, self.video_step_url,
            self.video_full_url, TabletData.audio_url)

        try:
            query = rospy.ServiceProxy("tablet", Tablet)
            results = query(
                screen, self.object_name, TabletData.face_url, 
                self.video_step_url, self.video_full_url, TabletData.audio_url)
            return results.success
        except rospy.ServiceException, e:
            rospy.logerr("Service call failed: {}".format(e))

        return False

    def tablet_execute(self, goal):
        """
        Handle the response from what the user clicked on the tablet
        """
        success = True
        response = goal.response
        rospy.loginfo("Got message from tablet: {}".format(response))
        self.do_error_feedback(Status.INPROGRESS, "TABLET MESSAGE {}".format(response))

        if response == "no" or response == "complete":
            self.is_task_error_completed = True
            success = self.tablet_goto_execute(Goal.BASE)
        elif response == "videodone":
            # Return to the options screen
            self.tablet_setup("options")
        elif response == "goto":
            # Show options but without the go to object button
            self.tablet_setup("options")
            success = self.tablet_goto_execute(Goal.OBJECT)

        tablet_goto_result = TabletGotoResult()
        tablet_goto_result.success = success
        self.tablet.set_succeeded(tablet_goto_result)

    def tablet_goto_execute(self, goal_type):
        """
        Execute goto object and base triggered by human-tablet interaction
        """
        # Find the object
        if self.goto(goal_type):
            if gotal_type == Goal.OBJECT:
                msg = "Found object"
                err_msg = "Did not find object"
                rospy.loginfo("Sending turtlebot to find object")
                self.tablet_feedback(Status.STARTED, "TURTLEBOT GOAL GO TO OBJECT")
                self.do_error_feedback(Status.INPROGRESS, "TURTLEBOT GOAL GO TO OBJECT")
            elif goal_type == Goal.BASE:
                msg = "Back at base"
                err_msg = "Did not get back at base"
                rospy.loginfo("Sending turtlebot to base")
                self.tablet_feedback(Status.STARTED, "TURTLEBOT GOAL GO TO BASE")
                self.do_error_feedback(Status.INPROGRESS, "TURTLEBOT GOAL GO TO BASE")

            while not self.is_goto_active:
                self.rate.sleep()

            while self.is_goto_active:
                self.tablet_feedback(Status.INPROGRESS, "TURTLEBOT NAVIGATING")
                self.do_error_feedback(Status.INPROGRESS, "TURTLEBOT NAVIGATING")
                self.rate.sleep()

            if self.goto_success:
                rospy.loginfo(msg)
                self.tablet_feedback(Status.COMPLETED, "TURTLEBOT COMPLETED TASK")
                self.do_error_feedback(Status.INPROGRESS, "TURTLEBOT COMPLETED TASK")
                return True

        rospy.logerr(err_msg)
        self.tablet_feedback(Status.FAILED, "TURTLEBOT FAILED")
        self.do_error_feedback(Status.INPROGRESS, "TURTLEBOT FAILED")
        return False

    def tablet_feedback(self, status, text):
        """
        Sends feedback to the tablet action server
        """
        tablet_feedback = TabletGotoFeedback()
        tablet_feedback.status = status
        tablet_feedback.text = text
        self.tablet.publish_feedback(tablet_feedback)

    def goto(self, goto_type):
        """
        From the specified server, try going to the specified object
        """
        if self.is_goto_active:
            rospy.logerr("Cannot navigate to two places at once!")
            return False

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
        self.is_goto_active = False
        if terminal_state == GoalStatus.RECALLED:
            self.goto_success = False
            status = "RECALLED"
        elif terminal_state == GoalStatus.REJECTED:
            self.goto_success = False
            status = "REJECTED"
        elif terminal_state == GoalStatus.PREEMPTED:
            self.goto_success = False
            status = "PREMEPTED"
        elif terminal_state == GoalStatus.ABORTED:
            self.goto_success = False
            status = "ABORTED"
        elif terminal_state == GoalStatus.SUCCEEDED:
            self.goto_success = True
            status = "SUCCEEDED"
        elif terminal_state == GoalStatus.LOST:
            self.goto_success = False
            status = "LOST"
        else:
            self.goto_success = False
            status = "UNKNOWN"
        rospy.loginfo("terminal state: {}  result: ({}, {})".format(
            status, result.status, result.is_complete))

    def goto_active_cb(self):
        self.is_goto_active = True

    def goto_feedback_cb(self, feedback):
        rospy.loginfo("goto feedback: x={}, y={}, z={}, status={}".format(
            feedback.x, feedback.y, feedback.z, feedback.text))


class TabletData(object):
    audio_url = "" # TODO nobody recorded the audio files yet?
    face_url = "happy-cartoon-face-hi.png"
    basename = 'http://casas.wsu.edu/smarthomestats/video/'
    
    def get_data(self, task_number, error_step):
        """
        From the task number and error step, get the corresponding object name
        and video urls
        """
        object_to_find = '' # No object for this error / step
        video_step_url = ''
        video_full_url = ''

        if task_number == Task.WATER_PLANTS:
           video_full_url = self.basename + 'waterplants.all.mp4'
           if error_step in [0, 1, 4, 5]:
               video_step_url = self.basename + 'waterplants.error1.mp4'
               object_to_find = 'watercan'
           elif error_step == 2:
               video_step_url = self.basename + 'waterplants.error2.mp4'
               object_to_find = 'plantcoffee'
           elif error_step == 3:
               video_step_url = self.basename + 'waterplants.error3.mp4'
               object_to_find = "plantside"

        elif task_number == Task.TAKE_MEDS:
            video_full_url = self.basename + 'takemedication.all.mp4'
            if error_step in [0, 5, 11]:
                video_step_url = self.basename + 'takemedication.error1.mp4'
                object_to_find = 'food'
            elif error_step in [1, 2, 7, 10]:
                video_step_url = self.basename + 'takemedication.error2.mp4'
                object_to_find = 'glass'
            elif error_step in [3, 6, 9]:
                video_step_url = self.basename + 'takemedication.error3.mp4'
                object_to_find = 'pillbottle'

        elif task_number == Task.WALK_DOG:
            video_full_url = self.basename + 'walkdog.all.mp4'
            if error_step == 0:
                video_step_url = self.basename + 'walkdog.error1.mp4'
                object_to_find = 'umbrella'
            elif error_step == 1:
                video_step_url = self.basename + 'walkdog.error2.mp4'
                object_to_find = 'leash'
            elif error_step == 2:
                video_step_url = self.basename + 'walkdog.error3.mp4'
                object_to_find = 'keys'
            elif error_step == 3:
                video_step_url = self.basename + 'walkdog.error4.mp4'
                object_to_find = 'dog'

        return object_to_find, video_step_url, video_full_url


if __name__ == '__main__':
    try:
        rospy.init_node('scheduler_server')
        server = SchedulerServer()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
