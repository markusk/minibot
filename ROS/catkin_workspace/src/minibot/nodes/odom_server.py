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


# --------------------
# wheel encoder stuff
# --------------------
import atexit

# for signal handling
import signal
import sys

# count the ticks from the wheel encoders "forever"
odomCountForeverFrontLeft  = 0
odomCountForeverRearLeft   = 0
odomCountForeverFrontRight = 0
odomCountForeverRearRight  = 0

# for getting the hostname of the underlying system
import socket
# showing hostname
hostname = socket.gethostname()
rospy.loginfo("Running on host %s.", hostname)
if (hostname != 'minibot') and (hostname != 'minibottest'):
    rospy.logwarn("Test mode only due to other host. Skipping all hardware staff!")
else:
    # we are "live" on robot hardware
    import RPi.GPIO as GPIO

    # GPIO init
    GPIO.setmode(GPIO.BCM) # use the BCM-GPIO names, _not_ the pin numbers on the board

    # Raspberry Pi GPIO configuration:
    frontLeftEncoderGPIO  = 27 # forward
    rearLeftEncoderGPIO   = 22 # forward
    frontRightEncoderGPIO = 23 # forward
    rearRightEncoderGPIO  = 24 # forward

    # setup
    rospy.loginfo("Wheel encoder GPIO setup...")
    GPIO.setup(frontLeftEncoderGPIO,  GPIO.IN)
    GPIO.setup(rearLeftEncoderGPIO,   GPIO.IN)
    GPIO.setup(frontRightEncoderGPIO, GPIO.IN)
    GPIO.setup(rearRightEncoderGPIO,  GPIO.IN)

    # encoder pulse detection by interrupt
    def fronLeftEncoderCallback(answer):
        global odomCountForeverFrontLeft
        odomCountForeverFrontLeft = odomCountForeverFrontLeft +1
        rospy.loginfo("Front left encoder.")

    def rearLeftEncoderCallback(answer):
        global odomCountForeverRearLeft
        odomCountForeverRearLeft = odomCountForeverRearLeft +1
        rospy.loginfo("Rear left encoder.")

    def frontRightEncoderCallback(answer):
        global odomCountForeverFrontRight
        odomCountForeverFrontRight = odomCountForeverFrontRight +1
        rospy.loginfo("Front right encoder.")

    def rearRightEncoderCallback(answer):
        global odomCountForeverRearRight
        odomCountForeverRearRight = odomCountForeverRearRight +1
        rospy.loginfo("Rear right encoder.")

    # add GPIO event detectors (interrupt service routines)
    print("registering event handlers...")

    # enabling event handlers (if needed only)
    def enableEncoderTracking():
        GPIO.add_event_detect(frontLeftEncoderGPIO,  GPIO.FALLING, callback=fronLeftEncoderCallback)
        GPIO.add_event_detect(rearLeftEncoderGPIO, GPIO.FALLING, callback=rearLeftEncoderCallback)
        GPIO.add_event_detect(frontRightEncoderGPIO, GPIO.FALLING, callback=frontRightEncoderCallback)
        GPIO.add_event_detect(rearRightEncoderGPIO, GPIO.FALLING, callback=rearRightEncoderCallback)

    # disabling event handlers
    def disableEncoderTracking():
        GPIO.remove_event_detect(frontLeftEncoderGPIO)
        GPIO.remove_event_detect(rearLeftEncoderGPIO)
        GPIO.remove_event_detect(frontRightEncoderGPIO)
        GPIO.remove_event_detect(rearRightEncoderGPIO)


# define a clean node exit
def my_exit():
  rospy.loginfo("Shutting down odometry service...")
  # run some parts only on the real robot
  if hostname == 'minibot' or hostname == 'minibottest':
        """ STOP wheel encoder tracking """
        disableEncoderTracking()
        GPIO.cleanup()
  rospy.loginfo("...shutting down odometrie service complete.")

# call this method on node exit
rospy.on_shutdown(my_exit)




while not rospy.is_shutdown():
    # for header time stamps
    current_time = rospy.Time.now()



    # Sleep for a second until the next reading.
    rospy.sleep(sleepTime)
