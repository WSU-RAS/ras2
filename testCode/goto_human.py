#!/usr/bin/env python

'''
Get human location from rostopic (Sim for now)
Get RAS location from rostopic (tf tree)
Do math to leave 2 foot space between RAS and human
Call goto_xy service
Return good debugging messages
'''

import rospy
import collections
from ras_msgs.srv import Goto_xy, Rotate
from tf import TransformListener


class GoToHuman():

    global lastknown_human
    global lastknown_RAS
    lastknown_human = (1.0, 1.0)
    lastknown_RAS = (0.0, 0.0)

    def __init__(self):


        rospy.init_node('nav_test_human', anonymous=False)

        #Listeners
        # rospy.Subscriber("tf", tf, setRAS(tf.lookupTransform("/base_link"))
        self.tf = TransformListener()
        self.setRAS()
        # rospy.Subscriber("human/human/human", Human, setHuman(data))

        locations = self.getLocation()
        human = locations[0]
        RAS = locations[1]


        if (human==RAS): print("Incorrect") #Shouldn't be same location, means it wasn't updated from listener

        msg = self.doMath(human[0], human[1], RAS[0], RAS[1])       
	

        #log stuff
    	rospy.loginfo("Beginning goto_human service")
    	rospy.spin()

    	#what to do if shut down (e.g. ctrl + C or failure)
    	rospy.on_shutdown(self.shutdown)

        return msg #message from moving RAS


    def doMath(self, h_x, h_y, r_x, r_y):

        #Do math, given location A and B, straight line between the two with 2ft gap between

        rospy.wait_for_service('goto_xy') #wait for service to start, need to start yourself (run file)
        goto_human = rospy.ServiceProxy('goto_xy', Goto_xy)

        rospy.wait_for_service('rotate') #wait for service to start, need to start yourself (run file)
        rotate_to_human = rospy.ServiceProxy('rotate', Rotate)

        if ((h_x == r_x) or (h_y == r_y)):


            # Simple -- Just keep it two feet away

            if (h_x == r_x): #horizontal match, change this to be more forgiving with in 5 degrees?
            
                #rosservice call goto_xy h_x, h_y-0.6096
                horizontal = goto_human(h_x, h_y-0.6096)
                rotate_to_human(r_x, r_y, h_x, h_y)
                return horizontal.response
            
            else:
            
                goto(h_x-0.6096, h_y) #vertical  match, change this to be more forgiving with in 5 degrees?
                #rosservice call goto_xy h_x-0.6096, h_y
                vert = goto_human(h_x-0.6096, h_y)
                rotate_to_human(r_x, r_y, h_x, h_y)
                return triangle.response

        else:

            # triangle -- equalatiral triangle with 0.6096m (2ft) longest size 
            # and the legs would be 0.431m (1.41ft) each.
            #rosservice call goto_xy h_x-0.431, h_y-0.431
            triangle = goto_human(h_x-0.431, h_y-0.431)
            rotate_to_human(r_x, r_y, h_x, h_y)
            return triangle.response
        

    
    def getLocation(self):
        return [lastknown_human, lastknown_RAS]


    def setRAS(self): #get data from tf listener
        #wiki.ros.org/tf/TfUsingPython
        if self.tf.frameExists("/base_link") and self.tf.frameExists("/map"):
            t = self.tf.getLatestCommonTime("/base_link", "/map")
            position, quaternion = self.tf.lookupTransform("/base_link", "/map", t) #position is tuple (x, y, z), quat (x, y, z, w)

            lastknown_RAS = (position[0], position[1])
        

    def setHuman(data): #get data from Human listener, currently incorrect
        # lastknown_human = (data.x, data.y)
        lastknown_human = (2.0, 2.0) #testing with human 2m away from origin 
    

    def shutdown(self): 
        rospy.loginfo("Editing Service")



if __name__ == '__main__':
    try:

        GoToHuman() #this returns the response not sure what to do with it right now

    except rospy.ROSInterruptException:
        rospy.loginfo("Exception thrown")
