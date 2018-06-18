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
from sensor_msgs.msg import Imu, Temperature

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


# for getting the hostname of the underlying system
import socket
# showing hostname
hostname = socket.gethostname()
rospy.loginfo("Running on host %s.", hostname)
if (hostname != 'minibot') and (hostname != 'minibottest'):
    rospy.logwarn("Test mode only due to other host. Skipping all hardware staff!")




while not rospy.is_shutdown():
    # for header time stamps
    current_time = rospy.Time.now()



    # Sleep for a second until the next reading.
    rospy.sleep(sleepTime)
