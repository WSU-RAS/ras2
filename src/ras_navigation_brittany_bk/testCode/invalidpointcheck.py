#!/usr/bin/env python


'''
This service will take a point given to use in the local map.
The map of the room will be processed into numpy grid.
Given the point we will find it in the numpy grid.
We will decide if it is an acceptable point below the threshold.
Once we find that point we will return it in right coordinates.
The coordiates we return are in the same format as the given ones.
'''


import rospy, random, time
import numpy as np
from ras_msgs.srv import Check_point
import actionlib
from actionlib_msgs.msg import *
from nav_msgs.msg import OccupancyGrid


#Try class to return service, if that doesn't work do Chris' global var idea.


#The service
class NewMap():

    #Given the text file of the map we find all the needed data.
    def __init__(self, map_obj, threshold):
        self.th = threshold
        self.columns = map_obj.info.width
        self.rows = map_obj.info.height
        self.x_offset = int(round(map_obj.info.origin.position.x*20))
        self.y_offset = int(round(map_obj.info.origin.position.y*20))
        self.origin = (-self.y_offset, -self.x_offset)
        self.Map = self.filterPoints(map_obj.data)

    def filterPoints(self, mp):

        pointer = 0
        np_map = np.zeros([self.rows, self.columns], dtype=int)

        for row in range(self.rows):
            for col in range(self.columns):
                np_map[row, col] = int(mp[pointer])
                pointer += 1

        np_map = np.flipud(np_map)
	return np_map

    #Returns T/F for if the requested point is acceptable or not.

    def wantedPoint(self, point):

	return np.logical_and(self.getValue(point) >= 0, self.getValue(point) < self.th)
	#This is abnormal because numpy was having issues with the 0 < x < n being ambiguous

    #Get the value at the requested point
    def getValue(self, point):
        return self.Map[point]

    #Finds the closest free point to your requested point, using spiral pattern. https://stackoverflow.com/questions/398299/looping-in-a-spiral

    def closestPoint(self, point):

        row = col = 0
        d_row = 0
        d_col = -1

        for i in range(max(self.rows, self.columns)**2):
            if row == col or (row < 0 and row == -col) or (row > 0 and row == 1 - col):

                d_row, d_col = -d_col, d_row

            row, col = row + d_row, col + d_col
            new_row, new_col = point[0] + row, point[1] + col

            #If the points are inbounds and its the correct value for the th then return these new points.
            if ((0 <= new_row < self.rows) and (0 <= new_col < self.columns)) and self.wantedPoint((new_row, new_col)): return (new_row, new_col)

    #Convert from RAS to Numpy

    def RAStoNumpy(self, point):

        return(point[1] - self.y_offset, point[0] - self.x_offset)

    #Convert from Numpy to RAS

    def NumpytoRAS(self, point):

        return (point[1] + self.x_offset, point[0] + self.y_offset)


#Start service
class GetClosestPoint():




    def __init__(self):
	self.success = None
	self.final_point = None
	self.sub = None

        rospy.init_node('Check_point', anonymous=False)
    	s = rospy.Service('Check_point', Check_point, self.check_it)
    	rospy.loginfo("Beginning Check Point service")
    	rospy.spin()

    	#what to do if shut down (e.g. ctrl + C or failure)
    	rospy.on_shutdown(self.shutdown)

    def check_it(self, given_point):
    	x = int(round(given_point.x))
    	y = int(round(given_point.y))
    	original_point = (x, y)

        self.sub = rospy.Subscriber('map', OccupancyGrid, self.check_map, original_point)
	    #time.sleep(1)

        if not self.success:
            rospy.loginfo("Failed to find correct point")
            return "Failure", original_point[0], original_point[1]

        else: 
            rospy.loginfo("Hooray, we found the next closest point")
            return "Succes", self.final_point[0], self.final_point[1]
            

    def check_map(self, map_obj, given_point):
	
	th = 75 #=threshold
	map = NewMap(map_obj, th)
	point = given_point
        point = map.RAStoNumpy(point) 
    	acceptable = map.wantedPoint(point)
    	
	if not acceptable: point = map.closestPoint(point)

    	if map.getValue(point) < th:
		print("Success", map.getValue(point), point) 
		self.success = True
		self.final_point = map.NumpytoRAS(point) #Give back as something RAS can understand.
		self.sub.unregister() 
		return
   	else:
		self.success = False
		self.sub.unregister()
		return

    	


    def shutdown(self):
        rospy.loginfo("Ending Service")



if __name__ == '__main__':
   
    try:

        GetClosestPoint()
       
    except rospy.ROSInterruptException:

        rospy.loginfo("Exception thrown")
