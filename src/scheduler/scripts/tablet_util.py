from ras_msgs.msg import TabletAction, TabletFeedback, TabletResult
from tablet_interface.srv import Tablet

from actionlib import SimpleActionServer

from casas_util import casas_logger

import rospy

class tablet_backend():

    def __init__(self):
        self.logger = casas_logger()

        self.tablet = SimpleActionServer(
            'tablet_response', TabletAction,
            execute_cb=self.tablet_execute,
            auto_start=False)
        self.tablet.start()
        self.object_name    = ""
        self.face_url       = "" 
        self.video_step_url = ""
        self.video_full_url = ""

    def tablet_setup(self, screen, showObject=True, navigateComplete=False,
            gotoObjectComplete=False):
        """
        Command the tablet to switch to a particular screen
        """
        rospy.loginfo("manager: Waiting for /tablet service")
        rospy.wait_for_service("tablet")
        rospy.loginfo("manager: Found /tablet service")


        rospy.loginfo(
            "Commanding tablet: s: %s o: %s f: %s vs: %s vf: %s",
            screen, self.object_name, self.face_url, self.video_step_url,
            self.video_full_url)

        try:
            query = rospy.ServiceProxy("tablet", Tablet)
            results = query(
                screen, self.object_name, self.face_url,
                self.video_step_url, self.video_full_url)
            return results.success
        except rospy.ServiceException, e:
            rospy.logerr("manager: Service call failed: {}".format(e))

        return False


    # Used to wait for user input
    def bide(self):
        self.updated = False
        self.response = None
        
        while not self.updated:
            rospy.sleep(0.1)

        return self.response

    # Called when tablet screen updates 
    def tablet_execute(self, goal):
        """
        Handle the response from what the user clicked on the tablet
        """

        response = goal.response
        rospy.loginfo("Tablet_util: Got message from tablet: {}".format(response))

        if response in ['yes', 'no', 'complete', 'goto', 'watchstep', 'watchfull']:
            message = {'action':'PRESSED', 'id':response}
            self.logger.log('ROS_Tablet', 'ROS_Tablet_Button', message, 'entity')

        self.response = response
        self.updated = True

        tablet_result = TabletResult()
        tablet_result.success = True
        self.tablet.set_succeeded(tablet_result)




    
