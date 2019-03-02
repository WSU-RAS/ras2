#!/usr/bin/env python

import rospy
import rospkg
import datetime
import logging
import configparser
import yaml
import math
import sys
import multiprocessing
import json

from threading import Timer
from datetime import datetime
from collections import deque
from actionlib import SimpleActionClient
from actionlib_msgs.msg import GoalStatus
from adl import check_sequence_gen, check_sequence_wloc_gen
from adl.util import Items, TaskToDag, Locations
from adl.util import WaterPlantsDag, WalkDogDag, TakeMedicationDag
from adl.util import ConnectPythonLoggingToRos
from adl.util import Task, Goal, Status
from adl.util import get_mac, bcolors
from adl.robot_sensor_translation import convert_point, RobotXYViz
from casas import objects, rabbitmq
from casas.publish import PublishToCasas

from ras_msgs.msg import DoErrorAction, DoErrorGoal
from ras_msgs.msg import TaskStatus
from ras_msgs.msg import RobotToMapTransformState
from ras_msgs.srv import TaskController, TaskControllerResponse
from ras_msgs.srv import Pause
from std_msgs.msg import String

class ErrorDetector:

    def __init__(self):
        rospy.loginfo("error_detector: Initializing error detector")

        rospack = rospkg.RosPack()
        self.pkg_path = rospack.get_path('adl_error_detection')

        self.test = False
        self.test_error = False

        self.casas = multiprocessing.Queue()
        self.casas_log = multiprocessing.Process(target=self.casas_logging, args=('1', self.casas, self.test or self.test_error, rospy))
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
            exchange_name='all.ai.testbed.casas',
            exchange_type='topic',
            routing_key='#',
            exchange_durable=True,
            casas_events=True,
            callback_function=self.__casas_cb)
        rospy.loginfo('--Done config--')

    def __casas_cb(self, sensors):
        for sensor in sensors:
            sensor_str = "{}\t{}\t{}\t{}\t{}\t{}".format(
                str(sensor.stamp), str(sensor.target), str(sensor.message), str(sensor.created_by), str(sensor.label), str(sensor.value))
            print(sensor_str)
            if sensor.created_by == 'RAS.InHome.ErrorDetection':
                print('RAS In-Home Error!')
                activity = sensor.label
                data = {'stamp': str(sensor.stamp), 'target': str(sensor.target), 'message': str(sensor.message), 'activity': str(activity), 'is_error': str('true')}
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

    def casas_logging(self, agent_num, queue, test, rospy):
        # CASAS Logging
        rospy.loginfo("Creating casas logger {}".format(agent_num))
        casas = PublishToCasas(
            agent_num=agent_num, node='ROS_Node_' + rospy.get_name()[1:] + '_' + agent_num,
            test=test) # use the test agent instead of kyoto if true
        try:
            t = Timer(0.01, casas.connect)
            t.start()
            rospy.sleep(1)

            while True:
                if not queue.empty():
                    casas.publish(**queue.get())
                else:
                    rospy.sleep(0.0001)
        finally:
            rospy.signal_shutdown("Cannot connect to CASAS (user {})! Need to restart.".format(agent_num))
            casas.finish()


if __name__ == "__main__":
    rospy.init_node(
        "error_detector",
        disable_signals=True,
        log_level=rospy.DEBUG if 'true' else rospy.INFO)

    ed = ErrorDetector()
    ed.casas_run()
    rospy.on_shutdown(ed.stop)
