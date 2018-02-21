#! /usr/bin/env python

import rospy

from actionlib import SimpleActionServer
from scheduler.msg import GotoAction, GotoFeedback, GotoResult


class GotoServer:

    __goto_feedback = GotoFeedback()
    __goto_result = GotoResult()


    def __init__(self):
        self.rate = rospy.Rate(10)
        self.goto_server = SimpleActionServer(
            'goto', GotoAction,
            execute_cb=self.goto_execute,
            auto_start=False)
        self.goto_server.start()

    def goto_execute(self, goal):
        x = 20
        y = -20
        z = 0
        rospy.loginfo("Executing Go To (0,0,0) from ({},{},{})".format(x,y,z))

        while True:
            self.__goto_feedback.x = x
            self.__goto_feedback.y = y
            self.__goto_feedback.z = z
            self.goto_server.publish_feedback(self.__goto_feedback)
            self.rate.sleep()
            x -= 1
            y += 1
            if x == 0 and y == 0 and z == 0:
                break

        self.__goto_result.result = 100
        self.__goto_result.is_complete = True
        self.goto_server.set_succeeded(self.__goto_result)

if __name__ == '__main__':
    rospy.init_node('goto_server')
    server = GotoServer()
    rospy.spin()

        
