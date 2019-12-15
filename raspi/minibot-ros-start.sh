#!/bin/bash

# needed for ROS
source /opt/ros/kinetic/setup.bash

# need to find the ROS workspace
source /home/pi/catkin_ws/devel/setup.bash

# change to ROS workspace (not really necessary)
cd /home/pi/catkin_ws

# start ROS
roslaunch minibot joystick_in_robot.launch
