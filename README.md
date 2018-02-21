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
rostopic pub /doerror_server/goal scheduler/DoErrorActionGoal '{header: auto, goal:{error: 12}}'
```

Next steps:
* Identify what should we used as goals? Example:
    * `uint32` task_number
    * `uint32` error_step
* Manager should have access to the database of object location and manager sends the `(x, y)` object location to  `goto` server node
* Create a code that would identify if goto node is performing:
    * Go to home
    * Go find person
    * Go to object
* We can either create one goto node to perform all goto functions or we can create a goto node for each different action.
* Connect `goto` with Chris' turtlebot navigation through actionlib. Chris should write an actionlib server to receive goals.
* Chris part will have to understand what action it needs to perform.
* Tablet pending...

## Git Submodules
Clone getting submodules:

    git clone --recursive https://github.com/WSU-RAS/ras.git ras

Adding a new submodule:

    cd ~/ras/src/
    git submodule add https://github.com/WSU-RAS/object_detection_msgs.git object_detection_msgs
    git submodule init

Updating all submodules to latest commit on origin:

    git submodule foreach git pull

Then to commit and push your changes:

    git commit -a
    git push
