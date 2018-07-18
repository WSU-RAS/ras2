#!/usr/bin/env python


import rospy
from ras_msgs.srv import Lurker_base_srv, Goto_xy


#Listening for the move command until it's triggered then move it
def lurker_move(data): #No data should be passed to the service, a lot of listeners

    rospy.init_node('Listening for base move')
    rospy.Subscriber("lurking_ai", String, move_it) #Get correct name from Yulia
    rospy.spin()


#Move the base after it is triggered
def move_it(data):

    rospy.wait_for_service('goto_xy') #wait for service to start, need to start yourself (run file)
    goto_place = rospy.ServiceProxy('goto_xy', Goto_xy)
    moved = goto_place(data.x, data.y)
    rospy.loginfo(moved.response)
    return moved.response
     

#Start services
class LurkerBase():

    def __init__(self):
        rospy.init_node('move_lurking_base', anonymous=False)
    	s = rospy.Service('Lurker_base_srv', Lurker_base_srv, lurker_move)
    	rospy.loginfo("Beginning lurker base move service")
    	rospy.spin()

    	#what to do if shut down (e.g. ctrl + C or failure)
    	rospy.on_shutdown(self.shutdown)


    def shutdown(self):
        rospy.loginfo("Ending Lurker Base Service")


if __name__ == '__main__':

    try:
        LurkerBase()
    
    except rospy.ROSInterruptException:
        rospy.loginfo("Exception thrown")