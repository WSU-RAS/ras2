import rospy
from turtlebot3_msgs.msg import SensorState


class BatteryLogger:

    def __init__(self, casas_publisher, mac, publish_rate):
        self.casas = casas_publisher
        self.mac_address = mac
        self.publish_rate = publish_rate
        self.battery_voltage = None
        rospy.Subscriber('/sensor_state', SensorState, self.robot_sensor_state_cb)
        rospy.Timer(rospy.Duration(2), self.robot_battery_cb, oneshot=True)

    def robot_sensor_state_cb(self, msg):
        """
        Subscriber callback to get battery life from turtlebot 3
        """
        self.battery_voltage = msg.battery

    def robot_battery_cb(self, event):
        """
        Publish battery life every 60 seconds to CASAS
        """
        while self.battery_voltage == None:
            rospy.sleep(2)

        while not rospy.is_shutdown():
            try:
                self.log_robot_battery()
                rospy.sleep(self.publish_rate)
            except KeyboardInterrupt:
                break

    def log_robot_battery(self):
        """
        Log robot's battery in voltage to CASAS
        """
        if self.battery_voltage != None:
            self.casas.publish(
                package_type='ROS',
                sensor_type='ROS_Battery',
                serial=self.mac_address,
                target='ROS_Battery',
                message='{0:.3f}'.format(self.battery_voltage),
                category='state'
            )
