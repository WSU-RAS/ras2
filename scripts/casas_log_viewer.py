#!/usr/bin/env python

import rospy
import rospkg
import datetime
import logging
import configparser

from collections import defaultdict
from actionlib import SimpleActionClient
from actionlib_msgs.msg import GoalStatus
from adl import check_sequence
from adl.util import Items, TaskToDag
from adl.util import WaterPlantsDag, WalkDogDag, TakeMedicationDag
from adl.util import ConnectPythonLoggingToRos
from adl.util import Task, Goal, Status
from adl.util import get_mac
from casas import objects, rabbitmq
from casas.publish import PublishToCasas

from ras_msgs.msg import DoErrorAction, DoErrorGoal
from ras_msgs.msg import TaskStatus
from ras_msgs.srv import TaskController, TaskControllerResponse
from ras_msgs.srv import Pause

class CasasLogViewer:

    def __init__(self):
        rospack = rospkg.RosPack()
        self.pkg_path = rospack.get_path('adl_error_detection')

        self.mac_address = get_mac()
        self.print_casas_log = True
        self.save_fp = None
        self.save_filename = None

        self.test_error = False
        if rospy.has_param("adl"):
            adl = rospy.get_param("adl")
            self.test_error = adl['test_error']
        self.task_status = TaskStatus()

        # CASAS Sensors
        config = configparser.ConfigParser()
        config.read(self.pkg_path + "/scripts/casas.cfg")
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
        self.casas_setup_exchange()

    def casas_setup_exchange(self):
        if self.test_error:
            self.rcon.setup_subscribe_to_exchange(
                exchange_name='test.events.testbed.casas',
                exchange_type='topic',
                routing_key='#',
                exchange_durable=True,
                casas_events=True,
                callback_function=self.__casas_cb)
        else:
            self.rcon.setup_subscribe_to_exchange(
                exchange_name='all.events.testbed.casas',
                exchange_type='topic',
                routing_key='#',
                exchange_durable=True,
                casas_events=True,
                callback_function=self.__casas_cb)

    def __casas_cb(self, sensors):
        for sensor in sensors:
            sensor_str = "{}\t{}\t{}".format(
                str(sensor.stamp), str(sensor.target), str(sensor.message))
            #if self.task_status.status == TaskStatus.ACTIVE:
            rospy.loginfo(sensor_str)

    def run(self):
        try:
            self.rcon.run()
        except KeyboardInterrupt:
            self.stop()

    def stop(self):
        rospy.loginfo("casas_log_viewer: disconnecting casas connection")
        self.rcon.stop()


if __name__ == "__main__":
    rospy.init_node("casas_log_viewer", disable_signals=True)
    casas = CasasLogViewer()
    casas.run()
    rospy.on_shutdown(ed.stop)
