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
from adl.util import Task, Goal
from casas import objects, rabbitmq

from ras_msgs.msg import DoErrorAction, DoErrorActionGoal
from ras_msgs.msg import TaskStatus
from ras_msgs.srv import TaskController, TaskControllerResponse

class ErrorDetector:

    def __init__(self):
        rospy.loginfo("Initializing Error Detector")

        rospack = rospkg.RosPack()
        pkg_path = rospack.get_path('adl_error_detection')

        self.task_number = None
        self.task_status = TaskStatus()
        self.task_status.status = TaskStatus.PENDING
        self.task_status.text = "PENDING"

        # start task rosservice server
        self.task_controller_srv = rospy.Service(
            'task_controller', TaskController,
            self.task_controller)

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

    def task_controller(self, request):
        response = TaskStatus(
            status=TaskStatus.SUCCESS,
            text="SUCCESS")
        if request.id.task_number < Task.size:
            if self.task_number is None \
               and request.request.status == TaskStatus.START \
               and self.task_status.status == TaskStatus.PENDING:
                self.task_number = request.id.task_number
                self.task_status.status = TaskStatus.ACTIVE
                self.task_status.text = "ACTIVE"
                rospy.loginfo("{}: START".format(Task.types[self.task_number]))
                return TaskControllerResponse(response)
            elif self.task_number == request.id.task_number \
               and request.request.status == TaskStatus.END \
               and self.task_status.status == TaskStatus.ACTIVE:
                self.task_number = None
                self.task_status.status = TaskStatus.PENDING
                self.task_status.text = "PENDING"
                rospy.loginfo("{}: END".format(Task.types[request.id.task_number]))
                return TaskControllerResponse(response)

        rospy.logwarn("Invalid task request")
        response.status = TaskStatus.FAILED
        response.text = "FAILED"        
        return TaskControllerResponse(response)

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

    def detect_error(self):
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
