#!/usr/bin/env python

import rospy
import rospkg
import datetime
import logging
import configparser

from collections import defaultdict, deque
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

class ErrorDetector:

    def __init__(self):
        rospy.loginfo("error_detector: Initializing error detector")

        rospack = rospkg.RosPack()
        self.pkg_path = rospack.get_path('adl_error_detection')

        self.mac_address = get_mac()
        self.print_casas_log = True
        self.save_fp = None
        self.save_filename = None
        self.sensors = deque()

        self.test = True
        self.save_task = True
        self.test_error = False
        if rospy.has_param("adl"):
            adl = rospy.get_param("adl")
            self.test = adl['is_test']
            self.save_task = adl['save_task']
            self.test_error = adl['test_error']
        self.task_status = TaskStatus()

        self.task_setup()
        self.ros_setup()

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

        rospy.Timer(rospy.Duration(1), self.casas_logging, oneshot=True)

    def casas_logging(self, event):
        # CASAS Logging
        self.casas = PublishToCasas(
            agent_num='2', node='ROS_Node_'+rospy.get_name()[1:],
            test=self.test) # use the test agent instead of kyoto if true
        try:
            self.casas.connect()
        finally:
            rospy.signal_shutdown("Cannot connect to CASAS! Need to restart.")
            self.casas.finish()

    def task_setup(self, task_num=-1, status=TaskStatus.PENDING, text="PENDING"):
        self.task_number = task_num
        self.task_status.status = status
        self.task_status.text = text
        self.task_sequence = []
        self.task_sequence_full = []
        self.task_pause = False
        self.task_dag = None
        self.error_key = None
        self.error_index = -1
        self.error_code = None
        self.last_key = None
        self.sensors.clear()
        if self.task_number >= 0:
            self.task_dag = TaskToDag.mapping[self.task_number].task_start

    def ros_setup(self):
        rospy.Timer(rospy.Duration(0.1), self.__error_detection_cb, oneshot=True)

        # start task rosservice server
        self.task_service = rospy.Service(
            'task_controller', TaskController,
            self.task_controller)
        rospy.loginfo("error_detector: task_controller service running")

        self.do_error = SimpleActionClient('do_error', DoErrorAction)
        self.do_error_connected = True
        if not self.do_error.wait_for_server(rospy.Duration(3)):
            self.do_error_connected = False
            rospy.logerr("error_detector: do_error client failed to connect")
            return

    def pause_dummy_data(self, request):
        rospy.wait_for_service('pause')
        try:
            pause = rospy.ServiceProxy('pause', Pause)
            response = pause(request)
            return response.success
        except rospy.ServiceException, e:
            rospy.logerr("Service call failed: {}".format(e))

    def start_error_correction(self, error_status):
        self.error_key = error_status[4]
        self.error_step = TaskToDag.mapping[self.task_number].subtask[self.error_key]
        self.error_code = TaskToDag.mapping[self.task_number].subtask_info[self.error_step][2]
        self.error_index = len(self.task_sequence)
        self.last_key = self.error_key
        rospy.loginfo("error_detector: Initiating error correction for {} at step {}".format(
            self.error_key, self.error_step))

        if self.save_task:
            time_str = datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')
            start_str = "{}\t{}\t{}\t{}".format(time_str, "ERROR", "START", self.error_key)
            self.save_fp.write("{}\n".format(start_str))

        if self.do_error_connected:
            goal = DoErrorGoal()
            goal.task_number = self.task_number
            goal.error_step = self.error_step
            self.do_error.send_goal(
                goal,
                done_cb=self.__error_correction_done_cb,
                active_cb=self.__error_correction_active_cb,
                feedback_cb=self.__error_correction_feedback_cb)

    def __error_correction_feedback_cb(self, feedback):
        rospy.loginfo("error_detector: do_error correction={}".format(feedback.text))

    def __error_correction_active_cb(self):
        self.task_pause = True
        self.task_status.text = "ERROR CORRECTION STARTED"
        self.casas.publish(
            package_type='ROS',
            sensor_type='ROS_Task_Step_Error',
            serial=self.mac_address,
            target='ROS_Task_Step_Fix_Error',
            message={
                'action':'START',
                'task':Task.types[self.task_number],
                'error_step':str(self.error_step),
                'error_id':self.error_key},
            category='state'
        )

    def __error_correction_done_cb(self, terminal_state, result):
        result_bool = (result.status == Status.COMPLETED)
        if terminal_state == GoalStatus.SUCCEEDED and result_bool:
            self.task_status.text = "ERROR CORRECTION SUCCEEDED"
            rospy.loginfo("error_detector: do_error correction succeeded")
            self.__correct_sequence()
            self.casas.publish(
                package_type='ROS',
                sensor_type='ROS_Task_Step_Error',
                serial=self.mac_address,
                target='ROS_Task_Step_Fix_Error',
                message={
                    'action':'SUCCEEDED',
                    'task':Task.types[self.task_number],
                    'error_step':str(self.error_step),
                    'error_id':self.error_key},
                category='state'
            )
            if self.test_error:
                self.pause_dummy_data(False)
        elif terminal_state == GoalStatus.SUCCEEDED and result.status == Status.FAILED:
            self.task_status.text = "ERROR CORRECTION FAILED"
            rospy.logerr("error_detector: do_error correction failed")
            self.casas.publish(
                package_type='ROS',
                sensor_type='ROS_Task_Step_Error',
                serial=self.mac_address,
                target='ROS_Task_Step_Fix_Error',
                message={
                    'action':'FAILED',
                    'error_step':str(self.error_step),
                    'task':Task.types[self.task_number]},
                category='state'
            )
        else:
            rospy.loginfo("error_detector: do_error preempted or cancelled")

        if self.save_task:
            time_str = datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')
            start_str = "{}\t{}\t{}\t{}".format(
                time_str, "ERROR", "END", self.task_status.text.lower().replace(" ", "_"))
            self.save_fp.write("{}\n".format(start_str))

        self.task_pause = False

    def __correct_sequence(self):
        self.task_dag = TaskToDag.mapping[self.task_number].subtask_info[self.error_step][4]
        self.task_sequence = []
        self.task_sequence_full.append(self.error_code + '*')
        """ OLD
        # Fix error by adding missing step in sequence
        self.task_sequence.insert(self.error_index, self.error_code)
        next_dag = TaskToDag.mapping[self.task_number].subtask_info[self.error_step][4]
        current = next_dag['current']
        next_step = TaskToDag.mapping[self.task_number].subtask[current]
        # Redo next step when next step is dependent to error step
        if (next_step - self.error_step) == 1:
            self.task_sequence = self.task_sequence[:self.error_index + 1]
        self.error_key = None
        """

    def stop_error_correction(self):
        if self.do_error_connected:
            if self.task_pause and self.do_error.get_state() == GoalStatus.ACTIVE:
                self.do_error.cancel_goal()
                rospy.logwarn("error_detector: do_error correction cancelled")
                self.casas.publish(
                    package_type='ROS',
                    sensor_type='ROS_Task_Step_Error',
                    serial=self.mac_address,
                    target='ROS_Task_Step_Fix_Error',
                    message={
                        'action':'CANCELLED',
                        'error_step':str(self.error_step),
                        'task':Task.types[self.task_number]},
                    category='state'
                )

    def task_controller(self, request):
        response = TaskStatus(
            status=TaskStatus.SUCCESS,
            text='SUCCESS')

        if request.id.task_number < Task.size:
            if self.task_number < 0 \
               and request.request.status == TaskStatus.START \
               and self.task_status.status == TaskStatus.PENDING:
                self.task_setup(
                    task_num=request.id.task_number,
                    status=TaskStatus.ACTIVE,
                    text="ACTIVE")
                if self.save_task:
                    date_str = datetime.datetime.now().strftime("%Y%m%d-%H%M%S")
                    self.save_filename = "{}-{}".format(request.id.task_number, date_str)
                    self.save_fp = open(
                        self.pkg_path + "/log/" + self.save_filename, mode='w')
                rospy.loginfo("error_detector: {}=START".format(Task.types[self.task_number]))
                self.casas.publish(
                    package_type='ROS',
                    sensor_type='ROS_Task',
                    serial=self.mac_address,
                    target='ROS_Task',
                    message={'action':'BEGIN', 'task':Task.types[self.task_number]},
                    category='state'
                )
                return TaskControllerResponse(response)

            elif self.task_number == request.id.task_number \
                    and request.request.status == TaskStatus.END \
                    and self.task_status.status == TaskStatus.ACTIVE:
                if self.save_task:
                    self.save_fp.close()
                    self.save_fp = None
                self.stop_error_correction()
                self.task_setup()
                rospy.loginfo("error_detector: {}=END".format(Task.types[request.id.task_number]))
                self.casas.publish(
                    package_type='ROS',
                    sensor_type='ROS_Task',
                    serial=self.mac_address,
                    target='ROS_Task',
                    message={'action':'END', 'task':Task.types[request.id.task_number]},
                    category='state'
                )
                return TaskControllerResponse(response)

        rospy.logwarn("task request invalid")
        response.status = TaskStatus.FAILED
        response.text = "FAILED"
        return TaskControllerResponse(response)

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

    def __add_sensor_to_sequence(self, sensor, sequence):
        is_estimote = sensor.sensor_type == 'Estimote-Movement' \
            and sensor.message == 'MOVED'
        is_door_closed = sensor.target in ['D001', 'D011'] \
            and sensor.message == 'CLOSE'
        if is_estimote or is_door_closed:
            decode_key = Items.decode[sensor.target]
            # Only include item used in the task
            if self.task_number in Items.encode[decode_key][2]:
                code = Items.encode[decode_key][1]
                sequence.append(code)
                rospy.loginfo("{}\t{}\t{}".format(
                    str(sensor.stamp), str(sensor.target), str(sensor.message)))

    def __check_error(self, new_sequence):
        task_key = self.task_dag['current']
        task_step = TaskToDag.mapping[self.task_number].subtask[task_key]
        result = check_sequence(
            self.task_dag,
            seq=self.task_sequence+new_sequence,
            task_count=task_step,
            task_num=TaskToDag.mapping[self.task_number].num_tasks)
        rospy.loginfo("error_detector: {}".format(result))
        return result

    def __error_detection(self):
        if len(self.sensors) > 0 and self.task_status.status == TaskStatus.ACTIVE and not self.task_pause:
            new_seq = self.sensors.popleft()
            if len(new_seq) > 0:
                check_result = self.__check_error(new_seq)
                is_error_detected = not check_result[1] and not check_result[3]

                # Error Detected
                if is_error_detected:
                    if self.error_key is None:
                        self.task_sequence_full.append("*")
                    self.task_pause = True
                    if self.test_error:
                        self.pause_dummy_data(True)

                    # Empty out sensors queue
                    self.sensors.clear()
                    # Start error correction
                    self.start_error_correction(check_result)

                # No Error Detected
                else:
                    # Check if a new step is completed in the sequence
                    if self.last_key != check_result[2] and check_result[1]:
                        self.current_step = TaskToDag.mapping[self.task_number].subtask[check_result[2]]
                        self.casas.publish(
                            package_type='ROS',
                            sensor_type='ROS_Task_Step',
                            serial=self.mac_address,
                            target='ROS_Task_Step',
                            message={
                                'task':Task.types[self.task_number],
                                'step':str(self.current_step),
                                'id':check_result[2]},
                            category='state'
                        )
                        rospy.loginfo('Task={} at step={}'.format(Task.types[self.task_number], self.current_step))
                    self.last_key = check_result[2]

                # Concatenate new sensor readings into task sequence
                self.task_sequence_full.extend(new_seq)
                self.task_sequence.extend(new_seq)
                rospy.loginfo("seq :{}".format("-".join(self.task_sequence)))
                rospy.loginfo("seq*:{}".format("-".join(self.task_sequence_full)))

    def __error_detection_cb(self, event):
        while not rospy.is_shutdown():
            try:
                self.__error_detection()
                rospy.sleep(0.001)
            except KeyboardInterrupt:
                break

    def __casas_cb(self, sensors):
        new_seq = []
        pause = self.task_pause
        for sensor in sensors:
            sensor_str = "{}\t{}\t{}".format(
                str(sensor.stamp), str(sensor.target), str(sensor.message))
            #if self.task_status.status == TaskStatus.ACTIVE:
            rospy.loginfo(sensor_str)
            if self.task_status.status == TaskStatus.ACTIVE and not pause:
                self.__add_sensor_to_sequence(sensor, new_seq)
            if self.save_task and self.save_fp is not None:
                self.save_fp.write("{}\n".format(sensor_str))

        # Catch some threading race issue when task is paused for
        # error detection and in the middle of the for loop task is unpaused
        # sensor readings should not be used for error detection
        if pause:
            return

        self.sensors.append(new_seq)

    def casas_run(self):
        try:
            self.rcon.run()
        except KeyboardInterrupt:
            self.stop()

    def stop(self):
        rospy.loginfo("error_detector: disconnecting casas connection")
        self.rcon.stop()
        self.casas.finish()


if __name__ == "__main__":
    rospy.init_node("error_detector", disable_signals=True)
    ed = ErrorDetector()
    ed.casas_run()
    rospy.on_shutdown(ed.stop)
