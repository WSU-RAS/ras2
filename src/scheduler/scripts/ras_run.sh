source /opt/ros/kinetic/setup.bash
source ~/cartographer_ws/install_isolated/setup.bash
source ~/ras/devel/setup.bash


#export ROS_MASTER_URI=http://wsu-ras-joule:11311
export ROS_HOSTNAME=wsu-ras-joule

# Jetson
export ROS_IP=10.42.0.1
export ROS_MASTER_URI=http://${ROS_IP}:11311
# No Jetson
#export ROS_MASTER_URI=http://localhost:11311

export TURTLEBOT_3D_SENSOR=astra
export TURTLEBOT3_MODEL=waffle
