#!/usr/bin/env python2
"""
Demo service to get responses from tablet

Access via:
    /tablet_response

    rosservice call /tablet_response "your response"
"""
import rospy
from tablet_interface.srv import TabletOption, TabletOptionResponse

class TabletResponseService:
    def __init__(self):
        # Name this node
        rospy.init_node('tabletResponse')

        # Listen to object locations that are published
        rospy.Service("/tablet_response", TabletOption, self.callback_object)

    def callback_object(self, req):
        rospy.loginfo("Got message from tablet: "+req.response)
        return TabletOptionResponse(True)

if __name__ == '__main__':
    try:
        service = TabletResponseService()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
