#!/bin/bash

# Kill on exit, interrupt, or terminate, otherwise  it'll keep running.
trap 'ssh jetson "killall roslaunch"' 0 2 15

# Run on the NVidia Jetson
ssh jetson 'roslaunch object_detection everything.launch'
