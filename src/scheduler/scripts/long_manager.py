#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32

from ras_msgs.srv import Goto_xywz
from ras_msgs.msg import casas_sensor

# logging utils
from casas_util import *

# Tablet backend
from tablet_util import tablet_backend

# lookup tables
from Decoder import Decode

class StateM():

    def __init__(self):
        # Lookup table for stuff
        self.dec   = Decode()
        self.state = "idle"
        self.logger = casas_logger()

    def start(self, activity, step):
        # Start system logging
        self.logger.log('ROS_Manager', 'ROS_State', 'start', 'state')

        # Begin by getting activity, step
        self.activity = activity
        self.step     = step

        # Get tablet backend ready
        self.tablet = tablet_backend()

        # Begin tablet play sound + face
        self.tablet.tablet_setup("moving")
    
        # Search for human
        self.handle_human()
        
        # Do tablet interface here
        self.handle_tab()

        # Robot back to base!
        self.handle_base()

        self.logger.log('ROS_Manager', 'ROS_State', 'end', 'state')

        return 'success'
    
    def handle_human(self):
        # Next locate the human
        human = self.dec.get_human()

        # Check if human location was updated in
        #   last 30s
        start_time = rospy.Time.now()
        timeout = rospy.Duration(secs=30, nsecs=0)
        try:
            if rospy.Time.now() - human.time < timeout:
                human_found = False
            else:
                human_found = True
        except:
            human_found = False

        # If human not seen recently go to alternative point
        if not human_found:
            mes = self.send_goal(self.dec.point(self.activity, self.step, '1'))
            self.logger.log('ROS_Manager', 'ROS_Nav', mes, 'state')

        # Try find human again
        human = self.dec.get_human()

        # Check if human location was updated in
        #   last 30s    
        start_time = rospy.Time.now()
        timeout = rospy.Duration(secs=30, nsecs=0)
        try:
            if rospy.Time.now() - human.time < timeout:
                human_found = False
            else:
                human_found = True
        except:
            human_found = False

        # If not found stop
        if not human_found:
            mes = self.send_goal(self.dec.point(self.activity, self.step, '2'))
            self.logger.log('ROS_Manager', 'ROS_Nav', mes, 'state')

        # Else goto human
        else:
            mes = self.send_goal(self.dec.get_human())
            self.logger.log('ROS_Manager', 'ROS_Nav', mes, 'state')
        

    def handle_tab(self):
        object_, video_step, video_full, face_url = self.dec.get_data(self.activity, self.step)

        # Display yes/no
        self.tablet.object_name = object_
        self.tablet.video_step_url = video_step
        self.tablet.video_full_url = video_full
        self.tablet.face_url = face_url
        
        self.tablet.tablet_setup("choice")
        response = None

        while response != 'no' and response != 'complete':

            # Wait for user input
            response = self.tablet.bide()

            # Done commands
            if response == 'no' or response == 'complete':
                continue
            
            # Show options at init or after vid
            elif response == 'yes' or response == 'videodone':
                self.tablet.tablet_setup("options")

            # Play video Commands
            elif response == 'watchstep' or response == 'watchfull':
                self.tablet.tablet_setup("options")
       
            # Goto Object
            elif response == 'goto':
                mes = self.send_goal(self.dec.point(self.activity, self.step, '3'))
                self.logger.log('ROS_Manager', 'ROS_Nav', mes, 'state')
                self.tablet.tablet_setup("options")
                
            else:
                rospy.logerr('Unknown response in handle_tab!')         

    def handle_base(self):
        mes = self.send_goal(self.dec.point(self.activity, self.step, '4'))
        self.logger.log('ROS_Manager', 'ROS_Nav', mes, 'state')


    def send_goal(self, point):
        # Maybe log attempted goal while running here
        if self.state != 'idle':
            return None

        # Make Sure service is ready
        rospy.wait_for_service('goto_point')

        try:
            self.state = 'moving'
            goto_srv = rospy.ServiceProxy('goto_point', Goto_xywz)
            response = goto_srv(point.x, point.y, point.w, point.z)
            self.state = 'idle'
            
            return response.response
        except rospy.ServiceException, e:
            rospy.loginfo("Schedular --> move: Failed to call service")
            rospy.loginfo("Schedular --> move:", e)


class Mang():

    def __init__(self):
        # Log system info once on startup
        system_logger()

        # Begin battery and location loggers
        # Defined in self so scoping doesnt play a role
        self.bt = battery_logger()
        self.lo = location_logger()

        # State machine for handling overall operation
        self.state_machine = StateM()

        # Listen for casas data
        rospy.Subscriber("casas_sensor", casas_sensor, self.sensor_listener)

    def sensor_listener(self, data):

        # Filter out non-error data types
        if data.created_by != 'RAS.InHome.ErrorDetection':
            return None

        # TODO: Bryan
        # Filter by appropriate params
        try:
            activity = data.label

        except Exception as e:
            print(e)

        # Send activity and step to state machine
        self.state_machine.start(activity, '0')



if __name__ == '__main__':
    try:
        rospy.init_node('scheduler', anonymous=False)
        server = Mang()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
