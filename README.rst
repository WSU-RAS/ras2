============================
RAS_Navigation
============================

Purpose
=======

RAS_Navigation is a set of systems that allows a turtlebot3 to navigation around the world. This is done by utilizing `Cartographer`_ for real-time simultaneous localization and mapping (`SLAM`_). The navigational system is setup to go to a certain point on the map by either listeing to the setGoal topic or through the use of the action library. 


.. _Cartographer: https://github.com/googlecartographer/cartographer
.. _SLAM: https://en.wikipedia.org/wiki/Simultaneous_localization_and_mapping


Getting started
===============

To get started, simply download the RAS github package and follow the main package's installation instructions!

To launch the navigational system enter:

roslaunch ras_navigation RAS_Navigation

To create a new map with the navigational system:

roslaunch ras_navigation RAS_Navigation new_map:=true



Configuration
=============

To configure the navigational settings, edit the files located in params. More specification on parameters can be found there.


tf_tree
=======

For the time being, RAS_Navigation handles setting up the tf tree. The tf tree when only the navigational system is running can be seen below.

.. image:: tf_tree.jpg
   :width: 300pt

When the object detection system is runnig, the camera tree will be linked to the main tree under base_link.
