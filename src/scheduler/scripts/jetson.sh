#!/bin/bash

# Wait till we can connect to the Jetson, particularly useful if running on
# boot since the Jetson takes a minute or so to come up.
echo "Waiting for Jetson to come up..."
while ! ping -c 1 10.42.0.217 &>/dev/null; do
    sleep 1
done
echo "Jetson is online!"

# Kill on exit, interrupt, or terminate, otherwise  it'll keep running.
trap 'ssh jetson "killall roslaunch"' 0 2 15

# Run on the NVidia Jetson
lab="$1"

if [[ $lab == true  || $lab == True ]]; then
    ssh jetson 'roslaunch object_detection everything.launch lab:=true'
else
    ssh jetson 'roslaunch object_detection everything.launch lab:=false'
fi
