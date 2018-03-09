#!/usr/bin/env python

import datetime

from collections import defaultdict
from adl.util import Items
from adl.util import WaterPlantsDag, WalkDogDag, TakeMedicationDag
from adl import check_sequence

from ras_msgs.msg import DoErrorAction, DoErrorActionGoal

class ErrorDetector:

    def __init__(self):
        self.do_error = SimpleActionClient('do_error', DoErrorAction)
        self.do_error.wait_for_server(rospy.Duration(3))

    def start_task(self, task_number):
        pass

    def detect_error(self):
        pass

    def stop_task(self):
        pass


if __name__ == '__main__':
    rospy.init_node('error_detector')
    error_detector = ErrorDetector()
    rospy.spin()
