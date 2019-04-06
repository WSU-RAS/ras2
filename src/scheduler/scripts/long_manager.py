#!/usr/bin/env python

import rospy
import rospkg
import math

import multiprocessing
import json
import yaml

from std_msgs.msg import String

from ras_msgs.msg import DoErrorGoal
from ras_msgs.srv import Goto_xywz

from object_detection_msgs.srv import ObjectQuery, ObjectQueryResponse

# logging utils
from casas_util import *

# Tablet backend
from tablet_util import tablet_backend

class StateM():

    def __init__(self):
        # Lookup table for stuff
        self.dec   = Decode()

    def start(self, activity, step):
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

        return 'success'
    
    def handle_human(self):
        # Next locate the human
        human = self.get_object('human')

        # Check if human location was updated in
        #   last 30s
        start_time = rospy.Time.now()
        timeout = rospy.Duration(secs=30, nsecs=0)
        if rospy.Time.now() - human.time < timeout:
            human_found = False
        else:
            human_found = True

        # If human not seen recently go to alternative point
        if not human_found:
            self.send_goal(self.dec.point(activity, step, '1'))

        # Try find human again
        human = self.get_human()

        # Check if human location was updated in
        #   last 30s    
        start_time = rospy.Time.now()
        timeout = rospy.Duration(secs=30, nsecs=0)
        if rospy.Time.now() - human.time < timeout:
            human_found = False
        else:
            human_found = True

        # If not found stop
        if not human_found:
            self.send_goal(self.dec.point(activity, step, '2'))
        else:
            self.send_goal(self.dec.point(activity, step, '3'))
        

    def handle_tab(self):

        # Display yes/no
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
                self.dec.vid_url(self.actvity, self.step)
       
            # Goto Object
            elif response == 'goto':
                obj = self.dec.obj(self.actvity, self.step)
                self.send_goal(obj)
                
            else:
                rospy.logerr('Unknown response in handle_tab!')         

    def handle_base(self):
        return None           


    def get_object(self, name):
        # Make Sure service is ready
        rospy.wait_for_service("query_objects")

        try:
            query = rospy.ServiceProxy("query_objects", ObjectQuery)
            result = query(name)
            return result.locations
        except rospy.ServiceException, e:
            rospy.logerr("Service call failed: %s" % e)
        return None
        

    def send_goal(self, activity, step):
        # Maybe log attempted goal while running here
        if self.state != 'idle':
            return None
        
        # Since we are idle and got new goal handle it
        goal = self.dec(activity, step)

    def move(self, x, y, w, z):
        # Make Sure service is ready
        rospy.wait_for_service('goto_point')

        # Format point
        pnt = Goto_xywz()
        pnt.x = x
        pnt.y = y
        pnt.w = w
        pnt.z = z
        try:
            goto_srv = rospy.ServiceProxy('goto_point', Goto_xywz)
            response = goto_srv(pnt)
            return response.sum
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
        state_machine = StateM()

        # Listen for casas data
        rospy.Subscriber("casas_sensor", String, self.sensor_listener)

    def sensor_listener(self, data):
        # Unpack string data in dict
        data = yaml.safe_load(data.data)

        #rospy.loginfo(data)
        # Filter out non-error data types
        if data['is_error'] != 'true':
            return None
    
        # Filter by appropriate params
        activity = data['activity'] 
        step     = data['step']

        # Send activity and step to state machine
        state_machine.start(activity, step)

class Decode():

    def __init__(self):
        return None

    # Lookup table for various points n stuff
    def point(self, activity, step, state):
        loc = self.get_object_location('human')
        return loc

    def get_object_location(self, name):
        rospy.wait_for_service("query_objects")
        try:
            query = rospy.ServiceProxy("query_objects", ObjectQuery)
            result = query(name)
            return result.locations
        except rospy.ServiceException, e:
            rospy.logerr("Service call failed: %s" % e)
        return None

    def get_goal(self, activity, step):
        goal = DoErrorGoal()
        goal.task_number = -1
        goal.error_step =  step

        # Error logic
        if activity == 'Eat':
            goal.task_number = 1
        elif activity == 'Work':
            goal.task_number = 2
        elif activity == 'Take_Medicine':
            goal.task_number = 0
        else:
            goal.task_number = -1

        return goal
        #    self.call_error.send_goal(goal)

    def get_data(task_number, error_step, use_location):
        """
        From the task number and error step, get the corresponding object name
        and video urls
        """
        object_to_find = ''  # No object for this error / step
        video_step_url = ''
        video_full_url = ''

        if task_number == Task.WATER_PLANTS:
            video_full_url = 'waterplants.all.mp4'
            video_step_url = 'waterplants.error{}.mp4'.format(error_step)
            if error_step == 0:
                object_to_find = 'watercan'
            elif error_step == 1:
                object_to_find = 'sink'
            elif error_step == 2:
                object_to_find = 'plantcoffee'
            elif error_step == 3:
                object_to_find = "plantside"

        elif task_number == Task.TAKE_MEDS:
            video_full_url = 'takemedication.all.mp4'
            video_step_url = 'takemedication.error{}.mp4'.format(error_step)
            #Adjust videos since we track new steps
            if use_location:
                if error_step == 3 or error_step == 5: #put_food_cup and put_med
                    video_step_url = '' #no video
                elif error_step == 4:
                    video_step_url = 'takemedication.error{}.mp4'.format(error_step - 1)
                elif error_step >= 6:
                    video_step_url = 'takemedication.error{}.mp4'.format(error_step - 2)
            if error_step == 0:
                object_to_find = 'food'
            elif error_step == 1:
                object_to_find = 'glass'
            elif error_step == 2:
                object_to_find = 'sink'
            elif error_step == (3 if not use_location else 4):
                object_to_find = 'pillbottle'
            elif error_step == (6 if not use_location else 8):
                object_to_find = 'pills'

        elif task_number == Task.WALK_DOG:
            video_full_url = 'walkdog.all.mp4'
            video_step_url = 'walkdog.error{}.mp4'.format(error_step)
            if error_step == 0:
                object_to_find = 'lappy'
            elif error_step == 1:
                object_to_find = 'leash'
            elif error_step == 2:
                object_to_find = 'keys'
            elif error_step == 3:
                object_to_find = 'dog'

        return object_to_find, video_step_url, video_full_url

        
    # New to v2
    def lookup_tablet(self, goal_class):
        obj_name = ''
        full_url = ''
        step_url = ''

        goal = goal_class.task_number

        # Eat        
        if goal == 1:
            obj_name = 'N/A'
            full_url = 'eat.all.mp4'
            step_url = 'eat.error3.mp4'
        # Work
        elif goal == 2:
            obj_name = 'lappy'
            full_url = 'work.all.mp4'
            step_url = 'work.error1.mp4'
        # Meds
        elif goal == 0:
            obj_name = 'N/A'
            full_url = 'takemedicine.all.mp4'
            step_url = 'takemedicine.error3.mp4'
               
        # Error
        else:
            print('oh gawd unknown goal x_X')

        return obj_name, step_url, full_url



if __name__ == '__main__':
    try:
        rospy.init_node('scheduler', anonymous=False)
        server = Mang()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
