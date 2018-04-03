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
        names.append('b1_b2_1')
        names.append('b1_b2_2')
        names.append('b1_b2_3')

    #navigate to universal point if not at base2
    elif (origin != "base2"):
        names.append('b1_b2_1')

    #Objects
    elif (origin == "base2"):
        names.append('b2_b1_1')
        names.append('b2_b1_2')
    
    #If all else fails, send it to uni
    else:
        names.append('b1_b2_1')

    #Append actual goal object to goal_names_list
    names.append(object_name)

    points = []
    for name in names:
        result = get_object_location(name)
        points.append((result[0].x,result[0].y,result[0].z,result[0].w))

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
        smach.State.__init__(
            self,
            outcomes=['success', 'fail', 'preempted'],
            input_keys=['position_x_in', 'position_y_in', 'orientation_z_in', 'orientation_w_in', 'points_in'])

        self.rate = rospy.Rate(10)
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)

        rospy.loginfo("Waiting for the move_base action server")
        self.move_base.wait_for_server(rospy.Duration(2))

    #new func
    def active_cb(self):
        rospy.loginfo("Goal pose "+str(self.goal_cnt+1)+" is now being processed by the Action Server...")

    #new func
    def feedback_cb(self, feedback):
        #To print current pose at each feedback:
        #rospy.loginfo("Feedback for goal "+str(self.goal_cnt)+": "+str(feedback))
        #rospy.loginfo("Feedback for goal pose "+str(self.goal_cnt+1)+" received")
        pass

    #new func
    def movebase_client(self):
        rospy.logerr("Goal cnt: {}, pose_seq: {}".format(self.goal_cnt, self.pose_seq))

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = self.pose_seq[self.goal_cnt]
        rospy.loginfo("Sending goal pose "+str(self.goal_cnt+1)+" to Action Server")
        rospy.loginfo(str(self.pose_seq[self.goal_cnt]))
        self.move_base.send_goal(goal, self.done_cb, self.active_cb, self.feedback_cb)

    #edited func
    def done_cb(self, status, result):
        self.goal_cnt += 1

        if status != 3:
            self.is_running = False

        if status == 2:
            rospy.loginfo("Goal pose "+str(self.goal_cnt)+" received a cancel request after it started executing, completed execution!")

        if status == 3:
            rospy.loginfo("Goal pose "+str(self.goal_cnt)+" reached")
            if self.goal_cnt < len(self.pose_seq):
                next_goal = MoveBaseGoal()
                next_goal.target_pose.header.frame_id = "map"
                next_goal.target_pose.header.stamp = rospy.Time.now()
                next_goal.target_pose.pose = self.pose_seq[self.goal_cnt]
                rospy.loginfo("Sending goal pose "+str(self.goal_cnt+1)+" to Action Server")
                rospy.loginfo(str(self.pose_seq[self.goal_cnt]))
                self.move_base.send_goal(next_goal, self.done_cb, self.active_cb, self.feedback_cb)
            else:
                rospy.loginfo("Final goal pose reached!")
                rospy.signal_shutdown("Final goal pose reached!")
                self.success = True
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


    def execute(self, userdata):
        rospy.loginfo("Executing state GotoXY")

        #multi-points
        self.pose_seq = list()
        self.goal_cnt = 0

        for point in userdata.points_in:
            self.pose_seq.append(Pose(Point(point[0],point[1],0), Quaternion(0,0,point[2],point[3])))
            #goal_cnt = goal_cnt + 1

        #run it
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
            rospy.loginfo("GotoXY failed")
            return "fail"

        rospy.loginfo("GotoXY succeeded")
        return "success"

    def request_preempt(self):
        smach.State.request_preempt(self)
        rospy.logwarn("GotoXYState Preempted")
