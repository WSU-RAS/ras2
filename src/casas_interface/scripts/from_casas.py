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
from ras_msgs.msg import casas_sensor

class Receiver:

    def __init__(self):

        rospack = rospkg.RosPack()
        self.pkg_path = rospack.get_path('adl_error_detection')

        self.test = False
        self.test_error = False

        self.pub = rospy.Publisher('casas_sensor', casas_sensor, queue_size=20)

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
            data = casas_sensor()
            data.by          = str(sensor.by)
            data.site        = str(sensor.site)
            data.sensor_type = str(sensor.sensor_type)
            data.stamp       = str(sensor.stamp)
            data.target      = str(sensor.target)
            data.message     = str(sensor.message)

            self.pub.publish(data)

    def casas_run(self):
        try:
            self.rcon.run()
        except KeyboardInterrupt:
            self.stop()

    def stop(self):
        rospy.loginfo("error_detector: disconnecting casas connection")
        self.rcon.stop()

    

if __name__ == "__main__":
    rospy.init_node(
        "error_detector",
        disable_signals=True,
        log_level=rospy.DEBUG if 'true' else rospy.INFO)

    ed = Receiver()
    ed.casas_run()
    rospy.on_shutdown(ed.stop)
