#!/usr/bin/env python
"""
Used in both findperson_smach.py and goto_object_smach.py
"""
import rospy
import smach
import actionlib

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus
from object_detection_msgs.srv import ObjectQuery, ObjectQueryResponse
from geometry_msgs.msg import Pose, Point, Quaternion

from adl.util import Task

class Goto_points():


    def __init__(self, args):
        #Utilize arguments
        data = args[2]

        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        self.move_base.wait_for_server(rospy.Duration(2))

        #multi-points
        self.pose_seq = list()
        self.goal_cnt = 0

        for point in data:
            self.pose_seq.append(Pose(Point(point[0],point[1],0), Quaternion(0,0,point[2],point[3])))

        self.movebase_client(args)


    def movebase_client(self, args):
        rospy.loginfo("Goal cnt: {}, pose_seq: {}".format(self.goal_cnt, self.pose_seq))

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = self.pose_seq[self.goal_cnt]
        rospy.loginfo("Sending goal pose "+str(self.goal_cnt+1)+" to Action Server")
        rospy.loginfo(str(self.pose_seq[self.goal_cnt]))
        self.move_base.send_goal(goal, self.done_cb, self.active_cb, self.feedback_cb)


    def active_cb(self, args):
        rospy.loginfo("Goal pose "+str(self.goal_cnt+1)+" is now being processed by the Action Server...")


    #This can be used to set the goal every so often
    def feedback_cb(self, feedback, args):
        #rospy.loginfo("Feedback for goal pose "+str(self.goal_cnt+1)+" received")
        pass


    def done_cb(self, status, result, args):
        self.goal_cnt += 1

        if status == 2:
            rospy.loginfo("Goal pose "+str(self.goal_cnt)+" received a cancel request after it started executing, completed execution!")

        elif status == 3:
            rospy.loginfo("Goal pose "+str(self.goal_cnt)+" reached")
            if self.goal_cnt < len(self.pose_seq):
                self.movebase_client()
                return
            else:
                rospy.loginfo("Final goal pose reached!")
                args[1] = True

        elif status == 4:
            rospy.loginfo("Goal pose "+str(self.goal_cnt)+" was aborted by the Action Server")

        elif status == 5:
            rospy.loginfo("Goal pose "+str(self.goal_cnt)+" has been rejected by the Action Server")

        elif status == 8:
            rospy.loginfo("Goal pose "+str(self.goal_cnt)+" received a cancel request before it started executing, successfully cancelled!")

        args[0] = False


#Logic for figuring out which points to use
def multi_path(origin, object_name):
    names = []

    #  base1 -> base2
    if (origin == "base1" and object_name == "base2"):
        names.append('b1_b2_1')
        names.append('b1_b2_2')
        names.append('b1_b2_3')

    #  base1 -> base3
    elif (origin == "base1" and object_name == "base3"):
        names.append('b1_b3_1')

    #  base2 -> base1
    elif (origin == "base2" and object_name == "base1"):
        names.append('b2_b1_1')
        names.append('b2_b1_2')
        names.append('b2_b1_3')

    #  base2 -> base3
    elif (origin == "base2" and object_name == "base3"):
        names.append('b2_b1_1')
        names.append('b2_b1_2')
        names.append('b1_b3_1')

    #  base3 -> base1
    elif (origin == "base3" and object_name == "base1"):
        names.append('b2_b1_3')

    #  base3 -> base2
    elif (origin == "base3" and object_name == "base2"):
        names.append('b1_b2_1')
        names.append('b1_b2_2')
        names.append('b1_b2_3')

    #stupid plant (plant on smaller table next to base2)
    #Navigate around if it is not at base2 (since base2 is already there)
    elif ( origin != "base2" and object_name == "plantside"):
        names.append('b1_b2_2')
        names.append('b1_b2_3')
    
    # navigate to the pillbottle if the robot starts from table
    #elif (origin != "base2" and object_name == "pillbottle"):
    #    names.append('universal_kitchen')
    
    # *** REALY BAD IDEA *** #
    # Navigate to universal point if not at base2
    #elif (origin != "base2"):
    #    names.append('b1_b2_1')

    #Objects
    #elif (origin == "base2"):
        #names.append('b2_b1_1')
        #names.append('b2_b1_2')

    # *** REALY BAD IDEA DONT DO THIS *** #
    # If all else fails, send it to uni
    #else:
    #    names.append('b1_b2_1')

    #Append actual goal object to goal_names_list, unless it is empty (incase of human)
    if (object_name != ""):
        names.append(object_name)

    points = []
    for name in names:
        result = get_object_location(name)
        if result is not None:
            points.append((result[0].x,result[0].y,result[0].z,result[0].w))
        else:
            rospy.logwarn("Object "+name+" not found in YAML file or database")

    return points



def get_object_location(name):

    rospy.wait_for_service("query_objects")

    try:
        query = rospy.ServiceProxy("query_objects", ObjectQuery)
        result = query(name)
        return result.locations
    except rospy.ServiceException, e:
        rospy.logerr("Service call failed: %s" % e)
    return None

class GotoXYState(smach.State):
    def __init__(self):
        smach.State.__init__(self,
            outcomes = ['success', 'fail', 'preempted'],
            input_keys = ['task_number_in', 'error_step_in', 'object_name_in', 'last_object_in', 'points_in'],
            output_keys = ['last_object_out']
        )
        self.rate = rospy.Rate(10)

        rospy.loginfo("GotoNewBaseState has been initialized, Waiting for the move_base action server")

        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        self.move_base.wait_for_server(rospy.Duration(2))



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
                self.success = True

        elif status == GoalStatus.ABORTED:
            rospy.loginfo("Goal pose "+str(self.goal_cnt)+" was aborted by the Action Server")

        elif status == GoalStatus.REJECTED:
            rospy.loginfo("Goal pose "+str(self.goal_cnt)+" has been rejected by the Action Server")

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
        rospy.loginfo("Executing state Goto New Base: "+userdata.object_name_in)

        # Human was not found, so go to a location we should be able to find them
        if userdata.object_name_in == "human_notfound":
            
            # Eat
            if userdata.task_number_in == 1:
                object_to_find = 'walkway'

            # Work
            elif userdata.task_number_in == 2:
                object_to_find = 'walkway'

            # Meds
            elif userdata.task_number_in == 3:
                object_to_find = 'kitchen'

            # Error
            else:
                rospy.logerr('gotoxy: Unknown task sent!')
                rospy.logerr('gotoxy: task=' + str(userdata.task_number_in))

            points = multi_path(userdata.last_object_in, object_to_find)

        # Human was found, so in FindPersonSMACH we would have found the points
        # we should go to. Now navigate to those locations.
        elif userdata.object_name_in == "human_found":
            object_to_find = "human"
            points = userdata.points_in

        # If nothing to do with human, go to some other object name
        else:
            object_to_find = userdata.object_name_in
            points = multi_path(userdata.last_object_in, object_to_find)

        userdata.last_object_out = object_to_find

        # multi-points
        self.pose_seq = list()
        self.goal_cnt = 0

        for point in points:
            self.pose_seq.append(Pose(Point(point[0],point[1],0), Quaternion(0,0,point[2],point[3])))

        # run it
        self.success = False
        self.is_running = True
        self.movebase_client()


        start_time = rospy.Time.now()
        timeout = rospy.Duration(secs=120, nsecs=0)
        while self.is_running and rospy.Time.now() - start_time < timeout:
            if self.preempt_requested():
                self.service_preempt()
                return 'preempted'
            self.rate.sleep()

        if not self.success:
            self.move_base.cancel_goal()
            rospy.loginfo("GotoXYNewBase failed: "+userdata.object_name_in)
            return "fail"

        rospy.loginfo("GotoXYNewBase succeeded: "+userdata.object_name_in)
        return "success"

    def request_preempt(self):
        smach.State.request_preempt(self)
        rospy.logwarn("GotoXYNewBase Preempted!")
