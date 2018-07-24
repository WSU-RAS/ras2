==========
Parameters
==========

====
RViZ
====

The RViZ parameters are located in the rviz folder. There should not be any navigational paramters in this folder as RViZ is only for visualizing the robot. The robot should run exactly the same with or without RViZ running.

============
Cartographer
============

Our only modified file is RAS_cartographer_params. The rest of the files are taken as-is from the cartographer system.

RAS_cartographer_params
=======================

This file holds the RAS system specific parameters. The full documentation can be found on cartographer's 'readthedocs'_.

.. _readthedocs: http://google-cartographer.readthedocs.io/en/latest/configuration.html

map_frame
---------

Our global fixed reference frame (default = 'map').

tracking_frame
--------------

The frame cartographer will use to preform SLAM on (default = 'base_link').

published_footprint
-------------------

The frame cartographer will publish the SLAMMED system on (default = 'base_footprint').

odom_frame
----------

The frame cartographer publishes odom data to (defalut = 'odom'). This is dependet on provide_odom_frame.

provide_odom_frame
------------------

If enabled, cartographer will published odom data to the odom_frame previously defined (default = 'true').

use_odometry
------------

This breaks the system, do not enable!! (default = 'false').


TRAJECTORY_BUILDER.pure_localization
------------------------------------

This is used to allow the system to find itself in the map. Enabling this will not stop new map construction (default = 'true').


other_params
------------

The other params are less significant and are used to fine-tune the system. For more information about these params see 'readthedocs'_.

==============
costmap_params
==============

This folder holds the costmap paramters. The costmap system determines what objects are in the way and how big the robot is and how much space to leave between objects for navigation. Full documentation can be found on the 'ros_nav_wiki_'. The costmaps are divided into two parts, the local and global maps. The local map is a smaller map that is tracked locally by the robot and is updated very quickly. The global map is fixed with respect to the global reference map used for all navigation. It is slow to update but contains the entire costmap for the robot.

.. _ros_nav_wiki: http://wiki.ros.org/navigation/Tutorials/RobotSetup

costmap_common_params_waffle
============================

This file sets the common costmap paramters to be used both locally and globally.


footprint
---------

This parameter sets the bounding box for the robot. It is currently set to take into consideration the back wheel and to add one inch extra on all sides.

robot_radius
------------

robot_radius can be used to set a circle bounding around the center of the robot. The center of the robot is the approximately where the LiDAR is.

inflation_radius 
----------------

How much should each detected objects size be icnreased by (effectively reduces the distance between the object and the robot).

scan
----

It will get its data from the LaserScan topic. It is possible to use other sensors here such as ultrasonic or depthcamera.

local_costmap
=============

global_frame
------------

The frame the local costmap should use for reference. (default = 'odom')

local_frame
-----------

The frame the local costmap should track for navigation. (default = 'base_link')

rolling_window
--------------

Allows the local costmap to be tracked with regards to our global frame. (default = true)


global_costmap
=============

global_frame
------------

The frame the global costmap should use for reference. (default = 'map')

local_frame
-----------

The frame the global costmap should track for navigation. (default = 'base_link')

width, height
-------------

These balues are used to set the size of the global costmap. The size should be set big enough for the robot to navigate the entire desired area.


================
move_base_params
================


dwa_local_planner_params
========================

This file sets the movement speed paramters and the goal tolerance parameters. Full documentation can be found at 'dwa_local_planner_'

.. _dwa_local_planner: http://wiki.ros.org/dwa_local_planner


move_base_params
================

Default values





