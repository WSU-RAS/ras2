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

### Load CASAS with Dummy data from File
Run the node below that would wait for rosservice call where you can specify the task number and dummy data file to load. This data will be read and published to CASAS. Once a service call is sent, it will also sent a rosservice call to `task_controller` to start error detection..
```
rosrun adl_error_detection test_error_detector.py
```

Run rosservice call to specify task number and file. `task_number: 2` here means walking the dog task, while `status: 2` is a request to start task. All dummy data file will be located under the `adl_error_detection/data` folder. Thus `2/0002.data` here means that there is a `0002.data` file under `adl_error_detection/data/2` folder.
```
rosservice call /test_task_controller '{id: {stamp: now, task_number: 2}, request: {status: 2}, file: 2/0002.data}'
```
