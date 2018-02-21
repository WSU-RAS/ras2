# RAS

## Scheduler

Run the goto action server.
```
rosrun scheduler goto.py
```

Run the manager server.
```
rosrun scheduler manager.py
```

Publish a goal to the `DoError` actionlib server `/doerror_server/goal` to test the manager server and goto action server.
```
rostopic pub /doerror_server/goal scheduler/DoErrorActionGoal '{header: auto, goal:{error: 12}}'
```
