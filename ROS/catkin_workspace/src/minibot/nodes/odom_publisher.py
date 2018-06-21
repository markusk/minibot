#!/usr/bin/env python
# coding=utf-8

"""
This node tries to generate odometrie by converting wheel encoder ticks from my
robot 'minibot'.
It then publishes odometrie messages into ROS for use in the navigation stack.

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
x  = 0.0
y  = 0.0
th = 0.0

# velocity
vx  = 0.4
vy  = 0.0
vth = 0.4
current_time = rospy.Time.now()
last_time = rospy.Time.now()
tfBroadcaster = tf.TransformBroadcaster()
degree = M_PI/180


""" robot parameters

:@# TODO: move these values to ROS parameter server
i.e.: node.getParam("odom/distancepercount", param_distancepercount)

16 Flankenwechsel pro Umdrehung,
durch Getriebe:
544 Flankenwechsel pro Umdrehung für ein 34:1 Getriebe

double DistancePerCount = 50.0/12.0 * 0.6/17.0 """
distancePerCount       = 0.0  # param_distancepercount   in meters (m)!!
lengthBetweenTwoWheels = 0.0  # param_widthbetweenwheels


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

odomCountFrontLeft  = 0
odomCountRearLeft   = 0
odomCountFrontRight = 0
odomCountRearRight  = 0

# this is one tick from a wheel encoder
odomStep = 1


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
        global odomCountFrontLeft
        global odomCountForeverFrontLeft
        # velocity is always positive, so we always increment
        odomCountFrontLeft        = odomCountFrontLeft        + odomStep
        odomCountForeverFrontLeft = odomCountForeverFrontLeft + odomStep
        rospy.loginfo("Front left encoder.")

    def rearLeftEncoderCallback(answer):
        global odomCountRearLeft
        global odomCountForeverRearLeft
        odomCountRearLeft        = odomCountRearLeft        + odomStep
        odomCountForeverRearLeft = odomCountForeverRearLeft + odomStep
        rospy.loginfo("Rear left encoder.")

    def frontRightEncoderCallback(answer):
        global odomCountFrontRight
        global odomCountForeverFrontRight
        odomCountFrontRight        = odomCountFrontRight        + odomStep
        odomCountForeverFrontRight = odomCountForeverFrontRight + odomStep
        rospy.loginfo("Front right encoder.")

    def rearRightEncoderCallback(answer):
        global odomCountRearRight
        global odomCountForeverRearRight
        odomCountRearRight        = odomCountRearRight        + odomStep
        odomCountForeverRearRight = odomCountForeverRearRight + odomStep
        rospy.loginfo("Rear right encoder.")

    # add GPIO event detectors (interrupt service routines)
    rospy.loginfo("registering event handlers...")

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

    """ START wheel encoder tracking """
    enableEncoderTracking()



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



""" this is where the odometry magic happens """
def calculateOdometry():
    # get current time
    current_time = rospy.Time.now()

	XXX = (0.1 / 50.0)

    # calculate passed time
    dt = (current_time - odo_last_time).toSec()

#	if (dt>=ODOM_PERIOD)
#	{
	# https://answers.ros.org/question/207392/generating-odom-message-from-encoder-ticks-for-robot_pose_ekf/
	# http://www.seattlerobotics.org/encoder/200610/Article3/IMU%20Odometry,%20by%20David%20Anderson.htm

    # extract the wheel velocities from the tick signals count
    deltaFrontLeft  = odomCountFrontLeft
    deltaRearLeft   = odomCountRearLeft
    deltaFrontRight = odomCountFrontRight
    deltaRearRight  = odomCountRearRight

    v_FrontLeft  = (deltaFrontLeft  * distancePerCount) / dt
    v_RearLeft   = (deltaRearLeft   * distancePerCount) / dt
    v_FrontRight = (deltaFrontRight * distancePerCount) / dt
    v_RearRight  = (deltaRearRight  * distancePerCount) / dt

    #motor_md[MOTOR_L].odom_rate = v_left;
    #motor_md[MOTOR_R].odom_rate = v_right;

    # :@TODO what about the rear values?!?
    motor_md[MOTOR_L].odom_rate = (4.0 * motor_md[MOTOR_L].odom_rate + v_FrontLeft) / 5.0
    motor_md[MOTOR_R].odom_rate = (4.0 * motor_md[MOTOR_R].odom_rate + v_FrontRight) / 5.0

    #motor_md[MOTOR_L].odom_rate = (19.0 * motor_md[MOTOR_L].odom_rate + v_left) / 20.0;
    #motor_md[MOTOR_R].odom_rate = (19.0 * motor_md[MOTOR_R].odom_rate + v_right) / 20.0;

    # :@TODO what about the rear values?!?
    vx = ((v_FrontRight + v_FrontLeft) / 2)
    vy = 0
    # :@TODO declare and define lengthBetweenTwoWheels
    # :@TODO what about the rear values?!?
    vth = ((v_FrontRight - v_FrontLeft) / lengthBetweenTwoWheels)

    #double dt = (current_time - last_time).toSec();
    delta_x = (vx * cos(th)) * dt
    delta_y = (vx * sin(th)) * dt
    delta_th = vth * dt

    x += delta_x
    y += delta_y
    th += delta_th

    # org: geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);
    # since all odometry is 6DOF we'll need a quaternion created from yaw
    odom_quat = tf::createQuaternionMsgFromYaw(th)
    # oder von imu_bno055:     odom_quat = tf.transformations.quaternion_from_euler(x, y, z) #   z vs. th ?!??


    """ generate tf broacast message and publish """
    # message declarations
    # the tf broadcaster
    # "any odometry source must publish information about the coordinate frame that it manages"
    # org: geometry_msgs::TransformStamped odom_trans;
    odom_trans = tf.TransformBroadcaster()
	odom_trans.header.stamp = current_time
	odom_trans.header.frame_id = "odom"
	odom_trans.child_frame_id = "base_footprint"

	odom_trans.transform.translation.x = x
	odom_trans.transform.translation.y = y
	odom_trans.transform.translation.z = 0.0
	odom_trans.transform.rotation = odom_quat

	# send the transform
	tfBroadcaster.sendTransform(odom_trans)


	""" Odometry message """
	odom = Odometry()
	odom.header.stamp = current_time;
	#odom.header.frame_id = "odom";

	# set the position
	odom.pose.pose.position.x = x;
	odom.pose.pose.position.y = y;
	odom.pose.pose.position.z = 0.0;
	odom.pose.pose.orientation = odom_quat;
	max = 1000000000000.0
	min = 0.001
	odom.pose.covariance = {
			min,    0.0, 0.0,  0.0,  0.0,  0.0,
			0.0,    min, 0.0,  0.0,  0.0,  0.0,
			0.0,    0.0, max,  0.0,  0.0,  0.0,
			0.0,    0.0, 0.0,  max,  0.0,  0.0,
			0.0,    0.0, 0.0,  0.0,  max,  0.0,
			0.0,    0.0, 0.0,  0.0,  0.0,  max };

	# set the velocity
	#odom.child_frame_id = "base_footprint";
	odom.twist.twist.linear.x = vx;
	odom.twist.twist.linear.y = vy;
	odom.twist.twist.angular.z = vth;
	odom.twist.covariance = {
			min,     0.0,  0.0,  0.0,  0.0,  0.0,
			0.0,     min,  0.0,  0.0,  0.0,  0.0,
			0.0,     0.0,  max,  0.0,  0.0,  0.0,
			0.0,     0.0,  0.0,  max,  0.0,  0.0,
			0.0,     0.0,  0.0,  0.0,  max,  0.0,
			0.0,     0.0,  0.0,  0.0,  0.0,  max };

	ROS_DEBUG("handleODO() dt=%f encoder=%d,%d,%d,%d position=%f,%f twist=%f,%f,%f ",
			dt,
			motor_md[MOTOR_R].odom_cnt,motor_md[MOTOR_R].odom_step,
			motor_md[MOTOR_L].odom_cnt,motor_md[MOTOR_L].odom_step,
			x,
			y,
			vx,
			vy,
			vth);

	# publish the message
	odom_pub.publish(odom)

	odo_last_time = current_time


	#
	# insert PID mode used/not used stuff here
	#


	#
	# publish diagnostic data
	#
	Float32 msg

	msg.data = motor_md[MOTOR_L].setpoint;
	motor_pub_setpoint[MOTOR_L].publish(msg);
	msg.data = motor_md[MOTOR_R].setpoint;
	motor_pub_setpoint[MOTOR_R].publish(msg);

	msg.data = motor_md[MOTOR_L].value;
	motor_pub_value[MOTOR_L].publish(msg);
	msg.data = motor_md[MOTOR_R].value;
	motor_pub_value[MOTOR_R].publish(msg);

	msg.data = motor_md[MOTOR_L].odom_rate/XXX;
	motor_pub_rate[MOTOR_L].publish(msg);
	msg.data = motor_md[MOTOR_R].odom_rate/XXX;
	motor_pub_rate[MOTOR_R].publish(msg);

	msg.data = motor_md[MOTOR_L].odom_cnt_forever;
	motor_pub_odomcnt[MOTOR_L].publish(msg);
	msg.data = motor_md[MOTOR_R].odom_cnt_forever;
	motor_pub_odomcnt[MOTOR_R].publish(msg);


    # reset odom tick counters
	motor_md[MOTOR_L].odom_cnt = 0;
	motor_md[MOTOR_R].odom_cnt = 0;


""" 'main' """
while not rospy.is_shutdown():
    # measure staff and publish odom
    calculateOdometry()


    # Sleep for a second until the next reading.
    rospy.sleep(sleepTime)
