#!/usr/bin/env python

import rospy
import rospkg
import logging
import configparser
import multiprocessing
import json

from adl.util import ConnectPythonLoggingToRos
from casas import objects, rabbitmq
from casas.publish import PublishToCasas

from std_msgs.msg import String

class Receiver:

    def __init__(self):

        rospack = rospkg.RosPack()
        self.pkg_path = rospack.get_path('adl_error_detection')

        self.test = False
        self.test_error = False

        self.casas = multiprocessing.Queue()
        self.casas_log = multiprocessing.Process(target=self.casas_logging, 
                                                 args=('1', self.casas, False, rospy))
        self.casas_log.daemon = True
        self.casas_log.start()

        self.pub = rospy.Publisher('casas_sensor', String, queue_size=10)

        # CASAS Sensors
        config = configparser.ConfigParser()
        config.read(self.pkg_path + "/scripts/casas.cfg")
        default = config['DEFAULT']

        self.translate = dict()
        self.rcon = rabbitmq.Connection(
            agent_name=rospy.get_name(),
            amqp_user=default['AmqpUser'],
            amqp_pass=default['AmqpPass'],
            amqp_host=default['AmqpHostDev'] if self.test_error or self.test else default['AmqpHost'],
            amqp_port=default['AmqpPort'],
            amqp_vhost=default['AmqpVHost'],
            amqp_ssl=default.getboolean('AmqpSSL'),
            translations=self.translate)
    
        rospy.loginfo('--Connection made--')

        self.rcon.l.addHandler(ConnectPythonLoggingToRos())
        rospy.loginfo('--Handler made--')
        self.casas_setup_exchange()

    def casas_setup_exchange(self):
        self.rcon.setup_subscribe_to_exchange(
            exchange_name='all.events.testbed.casas',
            exchange_type='topic',
            routing_key='#',
            exchange_durable=True,
            casas_events=True,
            callback_function=self.__casas_cb)
        rospy.loginfo('--Done config--')

    def __casas_cb(self, sensors):
        for sensor in sensors:
            sensor_str = "{}\t{}\t{}".format(
                str(sensor.stamp), str(sensor.target), str(sensor.message))
            data = {'stamp': str(sensor.stamp), 'target': str(sensor.target), 'message': str(sensor.message)}
            self.pub.publish(json.dumps(data))

    def casas_run(self):
        try:
            self.rcon.run()
        except KeyboardInterrupt:
            self.stop()

    def stop(self):
        rospy.loginfo("error_detector: disconnecting casas connection")
        self.rcon.stop()
        self.casas_log.terminate()
        self.casas.close()

    

if __name__ == "__main__":
    rospy.init_node(
        "error_detector",
        disable_signals=True,
        log_level=rospy.DEBUG if 'true' else rospy.INFO)

    ed = ErrorDetector()
    ed.casas_run()
    rospy.on_shutdown(ed.stop)
