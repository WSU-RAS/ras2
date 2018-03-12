# Activity Daily Living (ADL) Error Detection

## Requirements
Install the following python modules:
```
sudo pip install pika
sudo pip install configparser
```

This repository has some encrypted files. You will need `git-crypt` installed and the key to decrypt the files.
```
sudo apt install git-crypt
```

### Error Detector
Run the error detection node.
```
rosrun adl_error_detection error_detector.py
```

### Start/End Task ROS Service Call
There are three task types: (0) Watering plants, (1) Taking medication with food, and (2) Walking dog. You can request a task to start (2) or end (3).

Example:
Request to start taking medication with food task
```
rosservice call /task_controller '{id: {stamp: now, task_number: 1}, request: {status: 2}}'
```

Request to end taking medication with food task
```
rosservice call /task_controller '{id: {stamp: now, task_number: 1}, request: {status: 3}}'
```

Writing a ROS Service client code
```python
#!/usr/bin/env python

import rospy
from ras_msgs.msg import TaskStatus, TaskId
from ras_msgs.srv import TaskController

def start_task(task_number):
    rospy.wait_for_service('task_controller')
    try:
        start = rospy.ServiceProxy('task_controller', TaskController)
        id = TaskId(stamp=rospy.Time.now(), task_number=task_number)
        request = TaskStatus(status=TaskStatus.START)
        response = start(id, request)
    except rospy.ServiceException, e:
        rospy.logerr("Service call failed: %s"%e)

def end_task(task_number):
    rospy.wait_for_service('task_controller')
    try:
        end = rospy.ServiceProxy('task_controller', TaskController)
        id = TaskId(stamp=rospy.Time.now(), task_number=task_number)
        request = TaskStatus(status=TaskStatus.END)
        response = end(id, request)
    except rospy.ServiceException, e:
        rospy.logerr("Service call failed: %s"%e)
```
