# This file is to be used to send ALL information to casas
# It should subscribe to appropriate topics in this file and 
# send them to casas properly!

from casas.publish import PublishToCasas
from adl.util import get_mac

from collections import deque
import threading

def Sender():

    def __init__(self):

        # Var for holding mac address to send to casas
        self.mac_address = get_mac()

        # Queue used for holding messages to send to casas
        self.queue = deque()

        # Casas logging thread
        self.agent = threading.Thread(target=casas_logging)
        self.agent.daemon = True
        self.agent.start()

        self.subscribe()

    # Put all subscriptions here
    def subscribe():
        return None

    def casas_logging(self):
        # CASAS Logging
        agent_num = 0
        rospy.loginfo("Creating casas logger {}".format(agent_num))
        casas = PublishToCasas(
            agent_num=agent_num, node='ROS_Node_' + rospy.get_name()[1:] + '_' + agent_num,
            test=test) # use the test agent instead of kyoto if true
        try:
            t = Timer(0.01, casas.connect)
            t.start()
            rospy.sleep(1)

            while True:
                if len(self.queue) > 0:
                    casas.publish(**self.queue.popleft())
                else:
                    rospy.sleep(0.0001)
        finally:
            rospy.signal_shutdown("Cannot connect to CASAS (user {})! Need to restart.".format(agent_num))
            casas.finish()

    def casas_push(self, package_type, sensor_type, serial, 
                        target, message, category):

        self.queue.appendright(dict(
                    package_type='ROS',
                    sensor_type='ROS_manager_heartbeat_0',
                    serial=self.mac_address,
                    target='ROS_manager_heartbeat_0',
                    message='OK',
                    category='state'
                ))

    def heartbeat_cb(self, event):
        while not rospy.is_shutdown():
            try:
                self.casas_0.put(dict(
                    package_type='ROS',
                    sensor_type='ROS_manager_heartbeat_0',
                    serial=self.mac_address,
                    target='ROS_manager_heartbeat_0',
                    message='OK',
                    category='state'
                ))
                self.casas_1.put(dict(
                    package_type='ROS',
                    sensor_type='ROS_manager_heartbeat_1',
                    serial=self.mac_address,
                    target='ROS_manager_heartbeat_1',
                    message='OK',
                    category='state'
                ))
                self.casas_2.put(dict(
                    package_type='ROS',
                    sensor_type='ROS_manager_heartbeat_2',
                    serial=self.mac_address,
                    target='ROS_manager_heartbeat_2',
                    message='OK',
                    category='state'
                ))
                rospy.sleep(5)
            except KeyboardInterrupt:
                break

    def log_robot_location(self):
        """
        Logs location and angle(radians) of turtlebot in the map.
        Use this to publish information to CASAS when available.
        """
        trans, rot = self.get_robot_location()
        if trans != None and rot != None:
            degrees = (rot.yaw * 180./math.pi)
            self.casas_2.put(dict(
                package_type='ROS',
                sensor_type='ROS_XYR',
                serial=self.mac_address,
                target='ROS_XYR',
                message={
                    'x':'{0:.3f}'.format(trans.x),
                    'y':'{0:.3f}'.format(trans.y),
                    'rotation':'{0:.3f}'.format(degrees)},
                category='state'
            ))

    def robot_location_cb(self, event):
        r = rospy.Rate(1) # 1hz
        while not rospy.is_shutdown():
            try:
                if self.is_robot_moving:
                    self.log_robot_location()
                r.sleep()
            except KeyboardInterrupt:
                break


    def shutdown(self):
        if self.is_goto_active and self.use_robot:
            self.goto_client.cancel_goal()
        self.casas_log_0.terminate()
        self.casas_log_1.terminate()
        self.casas_log_2.terminate()
        self.casas_0.close()
        self.casas_1.close()
        self.casas_2.close()


