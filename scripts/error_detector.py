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
from adl.util import Task, Goal
from casas import objects, rabbitmq

from ras_msgs.msg import DoErrorAction, DoErrorGoal
from ras_msgs.msg import TaskStatus
from ras_msgs.srv import TaskController, TaskControllerResponse

class ErrorDetector:

    def __init__(self, test=False):
        rospy.loginfo("Initializing Error Detector")

        rospack = rospkg.RosPack()
        pkg_path = rospack.get_path('adl_error_detection')
        self.test = test
        self.task_status = TaskStatus()

        self.task_setup()
        self.ros_setup()

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
        self.casas_setup_exchange()

    def task_setup(self, task_num=None, status=TaskStatus.PENDING, text="PENDING"):
        self.task_number = task_num
        self.task_status.status = status
        self.task_status.text = text
        self.task_sequence = []
        self.task_sequence_full = []
        self.task_no_error = True 

    def ros_setup(self):
        # start task rosservice server
        self.task_service = rospy.Service(
            'task_controller', TaskController,
             self.task_controller)
        rospy.loginfo("task_controller service running")

        self.do_error = SimpleActionClient('do_error', DoErrorAction)
        self.do_error_connected = True
        if not self.do_error.wait_for_server(rospy.Duration(3)):
            self.do_error_connected = False
            rospy.logerr("Action server failed to connect")
            return

    def start_error_correction(self, error_status):
        error_key = error_status[4]
        error_step = TaskToDag.mapping[self.task_number].subtask[error_key]
        rospy.loginfo("Initiating error correction for {} at step {}".format(error_key, error_step))
        if self.do_error_connected:
            goal = DoErrorGoal()
            goal.task_number = self.task_number
            goal.error_step = error_step
            # TODO: Uncomment
            #self.do_error.send_goal(
            #    goal,
            #    done_cb=self.__error_correction_done_cb,
            #    active_cb=self.__error_correction_active_cb,
            #    feedback_cb=self.__error_correction_feedback_cb)

    def __error_correction_feedback_cb(self, feedback):
        rospy.loginfo("Error correction status={}".format(feedback.text))

    def __error_correction_active_cb(self):
        self.task_status.status = TaskStatus.PENDING
        self.task_status.text = "ERROR CORRECTION"

    def __error_correction_done_cb(self, terminal_state, result):
        self.task_status.status = TaskStatus.ACTIVE
        if terminal_state == GoalStatus.SUCCEEDED \
           and result.status == 3 and result.is_complete:
            self.task_status.text = "ERROR CORRECTION SUCCEEDED"
            rospy.loginfo("Error correction succeeded")
            # TODO: Once error corrected, need to have check_sequence
            # check the correct dag for the next subtask
            self.task_no_error = True
        else:
            self.task_status.text = "ERROR CORRECTION FAILED"
            rospy.logerr("Error correction failed")

    def stop_error_correction(self):
        if self.do_error_connected:
            self.do_error.cancel_goal()
            rospy.logwarn("Error correction cancelled")

    def task_controller(self, request):
        response = TaskStatus(
            status=TaskStatus.SUCCESS,
            text='SUCCESS')
        if request.id.task_number < Task.size:
            if self.task_number is None \
               and request.request.status == TaskStatus.START \
               and self.task_status.status == TaskStatus.PENDING:
                self.task_setup(
                    task_num=request.id.task_number,
                    status=TaskStatus.ACTIVE,
                    text="ACTIVE")
                rospy.loginfo("{}: START".format(Task.types[self.task_number]))
                return TaskControllerResponse(response)
            elif self.task_number == request.id.task_number \
               and request.request.status == TaskStatus.END \
               and self.task_status.status == TaskStatus.ACTIVE:
                self.task_setup()
                rospy.loginfo("{}: END".format(Task.types[request.id.task_number]))
                return TaskControllerResponse(response)

        rospy.logwarn("Invalid task request")
        response.status = TaskStatus.FAILED
        response.text = "FAILED"        
        return TaskControllerResponse(response)

    def casas_setup_exchange(self):
        self.rcon.setup_subscribe_to_exchange(
            exchange_name='all.events.testbed.casas',
            exchange_type='topic',
            routing_key='#',
            exchange_durable=True,
            casas_events=True,
            callback_function=self.__casas_cb)
        if self.test:
            self.rcon.setup_subscribe_to_exchange(
                exchange_name='test.events.testbed.casas',
                exchange_type='topic',
                routing_key='#',
                exchange_durable=True,
                casas_events=True,
                callback_function=self.__casas_cb)

    def __add_sensor_to_sequence(self, sensor, sequence):
        is_estimote = sensor.sensor_type == 'Estimote-Movement' \
            and sensor.message == 'MOVED'
        if is_estimote or sensor.target in ['D001', 'D011']:
            decode_key = Items.decode[sensor.target]
            # Only include item used in the task
            if self.task_number in Items.encode[decode_key][2]:
                code = Items.encode[decode_key][1]
                sequence.append(code)
                rospy.loginfo("{}\t{}\t{}".format(
                    str(sensor.stamp), str(sensor.target), str(sensor.message)))

    def __check_error(self, new_sequence):
        task_key = TaskToDag.mapping[self.task_number].task_start['current']
        task_step = TaskToDag.mapping[self.task_number].subtask[task_key]
        result = check_sequence(
            TaskToDag.mapping[self.task_number].task_start,
            seq=self.task_sequence+new_sequence,
            task_count=task_step,
            task_num=TaskToDag.mapping[self.task_number].num_tasks)
        rospy.loginfo("status:{}".format(result))
        return result

    def __casas_cb(self, sensors):
        if self.task_status.status == TaskStatus.ACTIVE:
            new_seq = []
            for sensor in sensors:
                self.__add_sensor_to_sequence(sensor, new_seq)

            if len(new_seq) > 0:
                check_result = self.__check_error(new_seq)
                is_error_detected = not check_result[1] and not check_result[3]
                if is_error_detected and self.task_no_error:
                    self.task_sequence_full.append("*")
                    self.start_error_correction(check_result)
                    self.task_no_error = False

                self.task_sequence_full.extend(new_seq)
                self.task_sequence.extend(new_seq)
                rospy.loginfo("seq:{}".format("-".join(self.task_sequence_full)))
        else:
            for sensor in sensors:
                rospy.loginfo("{}\t{}\t{}".format(
                    str(sensor.stamp), str(sensor.target), str(sensor.message)))

    def casas_run(self):
        try:
            self.rcon.run()
        except KeyboardInterrupt:
            self.stop()

    def stop(self):
        rospy.loginfo("Disconnecting casas connection")
        self.rcon.stop()


if __name__ == "__main__":
    rospy.init_node("error_detector", disable_signals=True)
    ed = ErrorDetector(test=True)
    ed.casas_run()
    rospy.on_shutdown(ed.stop)
