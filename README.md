# Robot Activity Support (RAS)

## Scheduler

Run the `goto` action server node.
```
rosrun scheduler goto.py
```

Run the `manager` node.
```
rosrun scheduler manager.py
```

Publish a goal to the `doerror_server` actionlib server `/doerror_server/goal`. This is a mock simulation that an error was detected and a `goal` is sent to the `manager`. Then the `manager` triggers the `goto` action.
```
rostopic pub /doerror_server/goal ras_msgs/DoErrorActionGoal '{header: auto, goal:{task_number: 1, error_step: 2}}'
```

## Git Submodules
Clone getting submodules:

    git clone --recursive https://github.com/WSU-RAS/ras.git ras

Adding a new submodule:

    cd ~/ras/src/
    git submodule add https://github.com/WSU-RAS/object_detection_msgs.git object_detection_msgs
    git submodule init

Updating all submodules to the latest commit on origin:

    git pull --recurse-submodules
    git submodule update --recursive --remote

Initializing newly added submodules (e.g. somebody else adds one):

    git submodule update --init --recursive
