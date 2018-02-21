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
