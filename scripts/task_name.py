#!/usr/bin/env python2
import rospy
from tablet_interface.srv import Task

class TaskResponseService:
    def __init__(self):
        # Name this node
        rospy.init_node('taskName')

        # Listen to object locations that are published
        rospy.Service("/task", Task, self.callback_object)

    def callback_object(self, req):
        rospy.loginfo("task name: "+req.response)
        return TabletOptionResponse(True)

if __name__ == '__main__':
    try:
        service = TaskResponseService()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
