import rospy
import rospkg

from turtlebot3_msgs.msg import SensorState
from ras_msgs.msg import casas_data
from adl.util import get_mac

import math

import tf
from tf import TransformListener
from tf.transformations import euler_from_quaternion

from collections import namedtuple

from geometry_msgs.msg import Twist
import git, giturlparse

'''
package_type='ROS',
sensor_type='ROS_Moving',
serial=self.mac_address,
target='ROS_Moving',
message='MOVING',
category='state'
'''

Transformation = namedtuple('Transformation', ['x', 'y', 'z'])
Rotation = namedtuple('Rotation', ['roll', 'pitch', 'yaw'])

class casas_logger():

    def __init__(self):
        self.mac = get_mac()
        self.pub = rospy.Publisher('to_casas', casas_data, queue_size=20)

    def log(self, sensor_type, target, message, category):
        #rospy.logerr('From: ' + str(sensor_type))
        data = casas_data()
        data.package_type = 'ROS'
        data.sensor_type  = sensor_type
        data.serial       = self.mac
        data.target       = target
        data.message      = str(message)
        data.category     = category

        self.pub.publish(data)

# Log system information to CASAS
class system_logger():

    def __init__(self):
        # Make logger
        logger = casas_logger()
        
        rospack = rospkg.RosPack()
        pkg_path = rospack.get_path('scheduler')
        repo = git.Repo(pkg_path, search_parent_directories=True)

        modules = {}
        url = repo.remote().url
        p = giturlparse.parse(url)
        target = '{}/{}'.format(p.owner, p.name)
        message = '{}'.format(repo.head.object.hexsha)
        modules[target] = message

        for sm in repo.submodules:
            p = giturlparse.parse(sm.url)
            target = '{}/{}'.format(p.owner, p.name)
            message = '{}'.format(sm.hexsha)
            modules[target] = message

        for target in modules.keys():
            logger.log('Module_version', target, modules[target], 'system')


# Following classes are misc loggings handled by manager!

class location_logger():

    def __init__(self):
        # Make logger
        self.logger = casas_logger()

        # Set initial conditions
        self.is_robot_moving = False
    
        # Make transform listener
        self.tf = TransformListener()

        # Sub to robot velocity topic
        # Will log bool of moving or not
        rospy.Subscriber('/cmd_vel', Twist, self.robot_cmd_vel_cb)

        # Thread for logging x,y, Rotation
        rospy.Timer(rospy.Duration(2), self.robot_location, oneshot=True)


    def robot_cmd_vel_cb(self, msg):
        """
        teleop and move_base publish to cmd_vel topic to move robot.
        Change to non-zero value means robot is moving and zero means still.
        """
        if self.is_robot_moving and msg.linear.x == 0 and msg.angular.z == 0:
            self.is_robot_moving = False
            self.logger.log('ROS_Moving', 'ROS_Moving', 'STILL', category='state')
        elif not self.is_robot_moving and (msg.linear.x != 0 or msg.angular.z != 0):
            self.is_robot_moving = True
            self.logger.log('ROS_Moving', 'ROS_Moving', 'MOVING', category='state')

    def robot_location(self, event):
        r = rospy.Rate(1) # 1hz
        while not rospy.is_shutdown():
            try:
                if self.is_robot_moving:
                    self.log_robot_location()
                r.sleep()
            except KeyboardInterrupt:
                break

    def get_robot_location(self):
        t, r = None, None
        try:
            (trans, rot) = self.tf.lookupTransform("/map", "/base_link", rospy.Time(0))
            t = Transformation(*trans)
            euler = euler_from_quaternion(rot)
            r = Rotation(*euler)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.warn("manager: Cannot find /map or /base_link")
        return (t, r)

    def log_robot_location(self):
        """
        Logs location and angle(radians) of turtlebot in the map.
        Use this to publish information to CASAS when available.
        """
        trans, rot = self.get_robot_location()
        if trans != None and rot != None:
            degrees = (rot.yaw * 180./math.pi)
            message =  {
                    'x':'{0:.3f}'.format(trans.x),
                    'y':'{0:.3f}'.format(trans.y),
                    'rotation':'{0:.3f}'.format(degrees)}

            self.logger.log('ROS_XYR', 'ROS_XYR', message, 'state')

class battery_logger():

    def __init__(self):
        # Make casas logger
        self.logger = casas_logger()

        # Used to hold current voltage for some time
        self.battery_voltage = -1.0

        # Called when sensor_state is updated (really fast)
        # Updates current voltage
        rospy.Subscriber('/sensor_state', SensorState, self.robot_sensor_state_cb)

        # Used to log battery every so often
        rospy.Timer(rospy.Duration(2), self.robot_battery_cb, oneshot=True)


    
    # Subscriber callback to get battery life from turtlebot 3
    def robot_sensor_state_cb(self, msg):
        self.battery_voltage = msg.battery

    # Publish battery life every 60 seconds to CASAS
    def robot_battery_cb(self, event):
        while not rospy.is_shutdown():
            try:
                message = '{0:.3f}'.format(self.battery_voltage)
                self.logger.log('ROS_Battery', 'ROS_Battery', message, 'state')
                rospy.sleep(60)
            except KeyboardInterrupt:
                break






