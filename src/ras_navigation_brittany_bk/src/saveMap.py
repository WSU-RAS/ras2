#!/usr/bin/env python

#Save the map every so many seconds

import time
from datetime import datetime
import rospy
from cartographer_ros_msgs.srv import WriteState

def saveMap(nowTime, path, filename):
    rospy.wait_for_service("write_state")

    try:
        #Save to current working directory
        write = rospy.ServiceProxy("write_state", WriteState)
        write(path + filename)


        #Save to map storage directory
        newPath = path + "old_maps/" + str(nowTime) + ".pbstream"
        write(newPath)
    except rospy.ServiceException, e:
        rospy.roserr("Service call failed: %s" % e)

if __name__ == "__main__":
    #Used to hold time for map storage
    nowTime = datetime.now().strftime("%Y-%m-%d-%H-%M-%S")

    rospy.init_node('saveMap')

    #variables to hold params from launch file
    filename = rospy.get_param("~filename", None)
    path = rospy.get_param("~path", None)

    #Error if no filename is specified
    if not filename:
        rospy.loger("File path to save map is required!")

    #Do until system is turned off
    while not rospy.is_shutdown():
        #Save map every 30 seconds
        time.sleep(30)
        saveMap(nowTime, path, filename)
        rospy.loginfo("Saved map")
