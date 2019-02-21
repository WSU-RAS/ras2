# This file is to be used to send ALL information to casas
# It should subscribe to appropriate topics in this file and 
# send them to casas properly!

import threading
import rospy

from collections import deque

from casas_lib.publish import PublishToCasas
from ras_msgs.msg import casas_data

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

        # Function to hold all subscriptions
        self.subscribe()

    # Many to one channel
    def subscribe():
        rospy.Subscriber("to_casas", casas_data, data_cb)

    # What gets called when info is sent via 'to_casas'
    def data_cb(self, data):
        self.queue.appendright(
                dict(
                    data.package_type,
                    data.sensor_type,
                    data.serial,
                    data.target,
                    data.message,
                    data.category
                    ))

    # Logs data to casas by checking a submission queue
    def casas_logging(self):

        agent_num = 0
        rospy.loginfo("Creating casas logger {}".format(agent_num))
        casas = PublishToCasas(agent_num=agent_num, 
                                node='ROS_Node_' + rospy.get_name()[1:] + '_' + agent_num,
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

    # Handles closing the casas agent
    def shutdown(self):
        self.agent.terminate()

if __name__ == '__main__':
    try:
        rospy.init_node('casas_interface', anonymous=False)
        server = Sender()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass






