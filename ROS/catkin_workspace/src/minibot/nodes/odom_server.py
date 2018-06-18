#!/usr/bin/env python
# coding=utf-8

"""
This node tries to generate odometrie by converting wheel encoder tickts from my
robot 'minibot'.
It also publishes odometrie messages into ROS for use in the navigation stack.

Author:  Markus Knapp, 2018
Website: https://direcs.de
"""

import logging
import sys
import time

import rospy
import math

from std_msgs.msg import Header, Float32
from sensor_msgs.msg import Imu

# see also http://wiki.ros.org/navigation/Tutorials/RobotSetup/Odom:
# "The nav_msgs/Odometry message stores an estimate of the position
#  and velocity of a robot in free space"

# see also ROS Odometry Python example
# https://gist.github.com/atotto/f2754f75bedb6ea56e3e0264ec405dcf
import nav_msgs.msg
from nav_msgs.msg import Odometry
# for the tf broadcaster
import tf
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3

# sleep time for this node in seconds
sleepTime = 0.25


# initialise the ROS node
rospy.init_node('odom_node', anonymous=False)


# initial position
x = 0.0;
y = 0.0;
th = 0;

# velocity
vx = 0.4;
vy = 0.0;
vth = 0.4;
current_time = rospy.Time.now()
last_time = rospy.Time.now()
broadcaster = tf.TransformBroadcaster()
degree = M_PI/180;

# message declarations
# the tf broadcaster
# "any odometry source must publish information about the coordinate frame that it manages"
# org: geometry_msgs::TransformStamped odom_trans;
odom_trans = tf.TransformBroadcaster()
odom_trans.header.frame_id = "odom";
odom_trans.child_frame_id = "base_footprint";


# for getting the hostname of the underlying system
import socket
# showing hostname
hostname = socket.gethostname()
rospy.loginfo("Running on host %s.", hostname)
if (hostname != 'minibot') and (hostname != 'minibottest'):
    rospy.logwarn("Test mode only due to other host. Skipping all hardware staff!")


# define a clean node exit
def my_exit():
  rospy.loginfo("Shutting down odometry service...")
  # run some parts only on the real robot
  if hostname == 'minibot':
      # @# TODO: GPIO stuff here. TO BE DEFINED
  rospy.loginfo("...shutting down odometrie service complete.")

# call this method on node exit
rospy.on_shutdown(my_exit)




while not rospy.is_shutdown():
    # for header time stamps
    current_time = rospy.Time.now()



    # Sleep for a second until the next reading.
    rospy.sleep(sleepTime)
