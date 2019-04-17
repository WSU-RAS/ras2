#!/usr/bin/env python
import rospy
import actionlib

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose, Point, Quaternion

from tablet_interface.srv import Tablet
from ras_msgs.srv import Goto_xywz
from ras_msgs.msg import Goto_xywz_msg

class multi_points():

    def __init__(self):
        # TODO: Start service which accepts many points
        return None

class goto_point():

    def __init__(self):
        # Set rate to 10Hz
        self.rate = rospy.Rate(10)
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        self.move_base.wait_for_server(rospy.Duration(2))

        # Create service to handle single point goals
        # Send data to srv_cb when called
        self.point_srv = rospy.Service('goto_point', Goto_xywz, self.srv_cb)

        rospy.loginfo("ras_navigation: goto_point has been initialized!")

    def srv_cb(self, point):
        rospy.loginfo('ras_navigation --> goto_point: Point received!')
        return self.movebase_client(point)

    def movebase_client(self, point):
        # Prep goal for move base client
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        # We dont get full point just xywz!
        point_formatted = Pose(Point(point.x, point.y,0), Quaternion(0,0,point.z, point.w))
        goal.target_pose.pose = point_formatted

        rospy.loginfo("ras_navigation --> goto_point: Sending point to Action Server")
        self.success = False
        self.is_running = True

        # Send the goal and continue
        self.move_base.send_goal(goal,
                                self.done_cb,
                                self.active_cb,
                                self.feedback_cb)

        # Set timeout constraints
        start_time = rospy.Time.now()
        timeout = rospy.Duration(secs=120, nsecs=0)
        while self.is_running and rospy.Time.now() - start_time < timeout:
            self.rate.sleep()

        if not self.success:
            self.move_base.cancel_goal()
            rospy.loginfo("ras_navigation --> goto_point: Failed")
            return "fail"

        rospy.loginfo("ras_navigation --> goto_point: Succeeded")
        return "success"

    def active_cb(self):
        rospy.loginfo("ras_navigation --> goto_point: Now being processed by the Action Server...")

    def feedback_cb(self, feedback):
        pass

    def done_cb(self, status, result):

        if status == GoalStatus.SUCCEEDED:
            rospy.loginfo("ras_navigation --> goto_point: Goal reached")
            self.success = True

        self.is_running = False

if __name__ == '__main__':
    rospy.init_node('goto_point_srv')
    s = goto_point()
    t = multi_points()

    rospy.spin()
