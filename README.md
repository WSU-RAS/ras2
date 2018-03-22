# Robot Activity Support (RAS)

## Run RAS Experiment

Run the full RAS experiment with robot and tablet.
```
roslaunch scheduler ras_experiment
```

Run RAS experiment partially. You append arguments at the end of the roslaunch command to set options. To run without robot, append `use_robot:=false`. To run without tablet, append `use_tablet:=false`. You can also use both arguments. By default, they are both set to `true`.
```
roslaunch scheduler ras_experiment use_robot:=false use_tablet:=true
```

## Scheduler

Run the `goto` action server node.
```
rosrun scheduler goto.py
```

Run the `manager` node.
```
rosrun scheduler manager.py
```

Publish a goal to the `do_error` actionlib server `/do_error/goal`. This is a mock simulation that an error was detected and a `goal` is sent to the `manager`. Then the `manager` triggers the `goto` action.
```
rostopic pub /do_error/goal ras_msgs/DoErrorActionGoal '{header: auto, goal:{task_number: 1, error_step: 2}}'
```

## Git Submodules
Clone getting submodules:

    git clone --recursive https://github.com/WSU-RAS/ras.git ras

Adding a new submodule:

    cd ~/ras/src/
    git submodule add https://github.com/WSU-RAS/object_detection_msgs.git object_detection_msgs
    git submodule init

Updating all submodules to the latest commit on origin:

    git submodule foreach -q --recursive "branch='$(git config -f $toplevel/.gitmodules submodule.$name.branch)'; git fetch origin --tags; git checkout $branch; git pull" && git pull && git submodule update --remote --recursive

Initializing newly added submodules (e.g. somebody else adds one):

    git submodule update --init --recursive
