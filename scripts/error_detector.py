#!/usr/bin/env python

import rospy
import rospkg
import datetime
import logging
import configparser

from actionlib import SimpleActionClient
from collections import defaultdict
from adl import check_sequence
from adl.util import Items
from adl.util import WaterPlantsDag, WalkDogDag, TakeMedicationDag
from adl.util import ConnectPythonLoggingToRos
from casas import objects, rabbitmq

from ras_msgs.msg import DoErrorAction, DoErrorActionGoal

class ErrorDetector:

    def __init__(self):
        rospy.loginfo("Initializing Error Detector")

        rospack = rospkg.RosPack()
        pkg_path = rospack.get_path('adl_error_detection')

        config = configparser.ConfigParser()
        config.read(pkg_path + "/scripts/casas.cfg")
        default = config['DEFAULT']

        self.translate = dict()
        self.rcon = rabbitmq.Connection(
            agent_name=rospy.get_name(),
            amqp_user=default['AmqpUser'],
            amqp_pass=default['AmqpPass'],
            amqp_host=default['AmqpHost'],
            amqp_port=default['AmqpPort'],
            amqp_vhost=default['AmqpVHost'],
            amqp_ssl=default.getboolean('AmqpSSL'),
            translations=self.translate)
        self.rcon.l.addHandler(ConnectPythonLoggingToRos())

        #self.do_error = SimpleActionClient("do_error", DoErrorAction)
        #self.do_error.wait_for_server(rospy.Duration(3))

    def casas_setup_exchange(self, test=False):
        self.rcon.setup_subscribe_to_exchange(
            exchange_name='all.events.testbed.casas',
            exchange_type='topic',
            routing_key='#',
            exchange_durable=True,
            casas_events=True,
            callback_function=self.casas_callback)
        if test:
            self.rcon.setup_subscribe_to_exchange(
                exchange_name='test.events.testbed.casas',
                exchange_type='topic',
                routing_key='#',
                exchange_durable=True,
                casas_events=True,
                callback_function=self.casas_callback)

    def casas_callback(self, sensors):
        for sensor in sensors:
            rospy.loginfo("{}\t{}\t{}".format(str(sensor.stamp), str(sensor.target), str(sensor.message)))

    def casas_run(self):
        try:
            self.rcon.run()
        except KeyboardInterrupt:
            self.stop()

    def start_task(self, task_number):
        pass

    def detect_error(self):
        pass

    def stop_task(self):
        pass

    def stop(self):
        rospy.loginfo("Disconnecting casas connection")
        self.rcon.stop()


if __name__ == "__main__":
    rospy.init_node("error_detector", disable_signals=True)
    ed = ErrorDetector()
    ed.casas_setup_exchange(test=True)
    ed.casas_run()
    rospy.on_shutdown(ed.stop)
