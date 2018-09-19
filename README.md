# Robot Activity Support (RAS)

## Requirements
Install the following python modules:
```
sudo pip install gitpython
sudo pip install git-url-parse
```

## Run RAS Experiment

Run the full RAS experiment with robot and tablet.
```
roslaunch scheduler ras_experiment.launch
```

Run RAS experiment partially. You append arguments at the end of the roslaunch command to set options. To run without robot, append `use_robot:=false`. To run without tablet, append `use_tablet:=false`. You can also use both arguments. By default, they are both set to `true`.
```
roslaunch scheduler ras_experiment.launch use_robot:=false use_tablet:=true
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

## Autostart on boot
To enable on boot (check the service file to make sure the path is right):

    sudo cp src/scheduler/ras_experiment.service /etc/systemd/system/
    sudo systemctl daemon-reload
    sudo systemctl enable ras_experiment.service

To start or stop, use `systemctl`, e.g.:

    sudo systemctl start ras_experiment.service

## Allow powering off via the experimenter interface
By default users don't have the privilege to power off without the admin
password or using sudo. If we want the ROS service to power off via the
experimenter interface, you can do this on both the Joule and Jetson and then
start the `poweroff.py` node:

    sudo chmod a+s /sbin/poweroff

## Git Subtrees
Cloning repo and its subtrees (ordinary clone):

    git clone https://github.com/WSU-RAS/ras.git ras

git subtree commands:

    git subtree add   -P <prefix> <commit>
    git subtree add   -P <prefix> <repository> <ref>
    git subtree pull  -P <prefix> <repository> <ref>
    git subtree push  -P <prefix> <repository> <ref>
    git subtree merge -P <prefix> <commit>
    git subtree split -P <prefix> [OPTIONS] [<commit>]
    
git push (note: might need to pull before pushing):

    git subtree push --prefix=src/adl_error_detection https://github.com/WSU-RAS/adl_error_detection.git master

More information about [git subtrees](https://github.com/git/git/blob/master/contrib/subtree/git-subtree.txt).
