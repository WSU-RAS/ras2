#!/usr/bin/env python
from sensor_msgs.msg import PointCloud2, PointField
import numpy as np
import rospy

def xyz_array_to_pointcloud2(points, stamp=None, frame_id='map'):
    '''
    Create a sensor_msgs.PointCloud2 from an array
    of points.
    '''
    msg = PointCloud2()
    if stamp:
        msg.header.stamp = stamp
    if frame_id:
        msg.header.frame_id = frame_id
    if len(points.shape) == 3:
        msg.height = points.shape[1]
        msg.width = points.shape[0]
    else:
        msg.height = 1
        msg.width = len(points)
    msg.fields = [
        PointField('x', 0, PointField.FLOAT32, 1),
        PointField('y', 4, PointField.FLOAT32, 1),
        PointField('z', 8, PointField.FLOAT32, 1)]
    msg.is_bigendian = False
    msg.point_step = 12
    msg.row_step = 12*points.shape[0]
    msg.is_dense = int(np.isfinite(points).all())
    msg.data = np.asarray(points, np.float32).tostring()

    return msg 

def talker():
    pub = rospy.Publisher('geo_bound', PointCloud2, queue_size=10)
    rospy.init_node('geo_bound_node', anonymous=False)
    rate = rospy.Rate(1) # 10hz
   
    dat = np.array([[-1.7, -1.5, 0], # Beyond chair
                    [-1.7, -1.6, 0],
                    [-1.7, -1.7, 0],
                    [-1.7, -1.8, 0],
                    [-1.7, -1.9, 0],
                    [-1.7, -2.0, 0],
                    [-1.7, -2.1, 0],
                    [-1.7, -2.2, 0],
                    [-1.7, -2.3, 0],
                    [-1.7, -2.4, 0],
                    [-1.7, -2.5, 0],
                    [-1.7, -2.6, 0],
                    [-1.7, -2.7, 0],
                    [-1.7, -2.8, 0],
                    [-1.7, -2.9, 0],
                    [-1.7, -3.0, 0],

                    [-0.1, -0.5, 0], # Sewing table
                    [-0.1, -0.6, 0],
                    [-0.1, -0.7, 0],
                    [-0.1, -0.8, 0],
                    [-0.1, -0.9, 0],
                    [-0.1, -1.0, 0],

                    [1.0, -0.1, 0],  # Dining table
                    [1.0, -0.2, 0],
                    [1.0, -0.3, 0],
                    [1.0, -0.4, 0],   
                    [1.0, -0.5, 0],
                    [1.0, -0.6, 0],
                    [1.0, -0.7, 0],
                    [1.0, -0.8, 0],  
                    [1.0, -0.9, 0],
                    [1.0, -1.0, 0],

                    [-0.7, -4.0, 0],  # Laundry room
                    [-0.7, -4.1, 0],
                    [-0.7, -4.2, 0],
                    [-0.7, -4.3, 0],
                    [-0.7, -4.4, 0],
                    [-0.7, -4.5, 0],
                    [-0.7, -4.6, 0],
                    [-0.7, -4.7, 0],
                    [-0.7, -4.8, 0],
                    [-0.7, -4.9, 0],
                    [-0.7, -5.0, 0]

                    ])
    while not rospy.is_shutdown():

        pub.publish(xyz_array_to_pointcloud2(dat))
        rate.sleep()


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
