#!/usr/bin/env python
import rospy
import math
import time

import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose, Point, Quaternion
from tf.transformations import quaternion_from_euler

from object_detection_msgs.srv import ObjectQuery, ObjectQueryResponse

def get_object_location(name):

    rospy.wait_for_service("query_objects")

    try:
        query = rospy.ServiceProxy("query_objects", ObjectQuery)
        result = query(name)
        return result.locations
    except rospy.ServiceException, e:
        rospy.logerr("Service call failed: %s" % e)
    return None


class MoveBaseSeq():

    def __init__(self, points):

        rospy.init_node('move_base_sequence')
        #points_seq = rospy.get_param('move_base_seq/p_seq')
        self.pose_seq = list()
        self.goal_cnt = 0
        self.time = time.time()
        self.time_n = 1

        for point in points:
            self.pose_seq.append(Pose(Point(point[0],point[1],0), Quaternion(0,0,point[2],point[3])))


        #Create action client
        self.client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server...")
        wait = self.client.wait_for_server(rospy.Duration(5.0))
        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
            return
        rospy.loginfo("Connected to move base server")
        rospy.loginfo("Starting goals achievements ...")
        self.movebase_client()

    def active_cb(self):
        rospy.loginfo("Goal pose "+str(self.goal_cnt+1)+" is now being processed by the Action Server...")

    def feedback_cb(self, feedback):
        #To print current pose at each feedback:
        #rospy.loginfo("Feedback for goal "+str(self.goal_cnt)+": "+str(feedback))
        rospy.loginfo("Feedback for goal pose "+str(self.goal_cnt+1)+" received")
        if (self.time + self.time_n*5 < time.time()):
            self.time_n = self.time_n + 1
            self.client.send_goal(self.current_goal, self.done_cb, self.active_cb, self.feedback_cb) 

    def done_cb(self, status, result):
        self.goal_cnt += 1
    # Reference for terminal status values: http://docs.ros.org/diamondback/api/actionlib_msgs/html/msg/GoalStatus.html
        if status == 2:
            rospy.loginfo("Goal pose "+str(self.goal_cnt)+" received a cancel request after it started executing, completed execution!")

        if status == 3:
            rospy.loginfo("Goal pose "+str(self.goal_cnt)+" reached") 
            if self.goal_cnt< len(self.pose_seq):
                next_goal = MoveBaseGoal()
                next_goal.target_pose.header.frame_id = "map"
                next_goal.target_pose.header.stamp = rospy.Time.now()
                next_goal.target_pose.pose = self.pose_seq[self.goal_cnt]
                rospy.loginfo("Sending goal pose "+str(self.goal_cnt+1)+" to Action Server")
                rospy.loginfo(str(self.pose_seq[self.goal_cnt]))
                self.current_goal = next_goal
                self.client.send_goal(next_goal, self.done_cb, self.active_cb, self.feedback_cb) 
            else:
                rospy.loginfo("Final goal pose reached!")
                rospy.signal_shutdown("Final goal pose reached!")
                return

        if status == 4:
            rospy.loginfo("Goal pose "+str(self.goal_cnt)+" was aborted by the Action Server")
            rospy.signal_shutdown("Goal pose "+str(self.goal_cnt)+" aborted, shutting down!")
            return

        if status == 5:
            rospy.loginfo("Goal pose "+str(self.goal_cnt)+" has been rejected by the Action Server")
            rospy.signal_shutdown("Goal pose "+str(self.goal_cnt)+" rejected, shutting down!")
            return

        if status == 8:
            rospy.loginfo("Goal pose "+str(self.goal_cnt)+" received a cancel request before it started executing, successfully cancelled!")

    def movebase_client(self):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now() 
        goal.target_pose.pose = self.pose_seq[self.goal_cnt]
        rospy.loginfo("Sending goal pose "+str(self.goal_cnt+1)+" to Action Server")
        rospy.loginfo(str(self.pose_seq[self.goal_cnt]))
        self.current_goal = goal
        self.client.send_goal(goal, self.done_cb, self.active_cb, self.feedback_cb)
        rospy.spin()

if __name__ == '__main__':
    try:
	points = []
	#points = x,y  , z,w
        '''
        loc = get_object_location("base1")[0]
        points.append((loc.x, loc.y, loc.z, loc.w))
        loc = get_object_location("b1_b2_1")[0]
        points.append((loc.x, loc.y, loc.z, loc.w))
        loc = get_object_location("b1_b2_2")[0]
        points.append((loc.x, loc.y, loc.z, loc.w))
        loc = get_object_location("base2")[0]
        points.append((loc.x, loc.y, loc.z, loc.w))
        '''
        points.append((0,0.5,1,0))
        MoveBaseSeq(points)
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation finished.")

