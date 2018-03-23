#!/bin/bash
# Need to set up all the ROS environment
source /opt/ros/kinetic/setup.bash
source ~/cartographer_ws/install_isolated/setup.bash
source ~/ras/devel/setup.bash
export ROS_IP=10.42.0.1
export ROS_MASTER_URI=http://$ROS_IP:11311
export TURTLEBOT_3D_SENSOR=astra
export TURTLEBOT3_MODEL=waffle

# Launch
roslaunch scheduler ras_experiment.launch

# Use 'rviz' alias to access the rviz GUI
