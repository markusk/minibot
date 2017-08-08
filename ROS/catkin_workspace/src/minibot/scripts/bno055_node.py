#!/usr/bin/env python
# coding=utf-8

# ROS node for publishing a /imu/data topic using the Adafruit BNO055 9DOF board.
#
# Based on Adafruits simpletest.py example.
# Author: Markus Knapp, 2017. https://direcs.de
#
#
# Copyright (c) 2015 Adafruit Industries
# Original-Author: Tony DiCola
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.
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
from nav_msgs.msg import Odometry
# for the tf broadcaster
import tf

from Adafruit_BNO055 import BNO055


# sleep time for this node in seconds
sleepTime = 0.25


# initialise the node
rospy.init_node('bno055_node')

# the odometry topic
pubOdom = rospy.Publisher('odom', Odometry, queue_size=50)
# the tf broadcaster
# "any odometry source must publish information about the coordinate frame that it manages"
odomBroadcaster = tf.TransformBroadcaster() # "br" in tf_broadcaster.py

# Euler topics
pubH = rospy.Publisher('heading', Float32, queue_size=1)
pubR = rospy.Publisher('roll',    Float32, queue_size=1)
pubP = rospy.Publisher('pitch',   Float32, queue_size=1)
# Temperature
pubTemp = rospy.Publisher('temperature',   Temperature, queue_size=1)
# ROS IMU format messages
pubImu  = rospy.Publisher('imu/data', Imu, queue_size=1)

# header frame for odometry message
frame_id = 'odom' #
child_id = 'base_link' # normally the coordinate frame of the mobile base, so base_link.
seq = 0


# define temperature message
temp_msg = Temperature()
# ignore the covariance data, it is optional
temp_msg.variance = 0

# define IMU message
imu_msg = Imu()
# ignore the covariance data, it is optional
imu_msg.orientation_covariance[0]         = -1
imu_msg.angular_velocity_covariance[0]    = -1
imu_msg.linear_acceleration_covariance[0] = -1


# Create and configure the BNO sensor connection.
# Using I2C without a RST pin
bno = BNO055.BNO055()

# Initialize the BNO055 and stop if something went wrong.
if not bno.begin():
    rospy.logerr("Failed to initialize BNO055! Is the sensor connected?")

# Print system status
status, self_test, error = bno.get_system_status()
rospy.loginfo('System status:      0x{0:02X} '.format(status))

# Print self test
rospy.loginfo('Self test result:   0x{0:02X}'.format(self_test))
if self_test != 0x0F:
    rospy.logwarn('WARNING: Self test result is 0x{0:02X} instead if 0x0F!'.format(self_test))

# Print out an error if system status is in error mode.
if status == 0x01:
    rospy.logerr('System error:    {0}'.format(error))
    rospy.loginfo('See datasheet section 4.3.59 for the meaning.')

# Print BNO055 software revision and other diagnostic data.
sw, bl, accel, mag, gyro = bno.get_revision()
rospy.loginfo('Software version:   {0}'.format(sw))
rospy.loginfo('Bootloader version: {0}'.format(bl))
rospy.loginfo('Accelerometer ID:   0x{0:02X}'.format(accel))
rospy.loginfo('Magnetometer ID:    0x{0:02X}'.format(mag))
rospy.loginfo('Gyroscope ID:       0x{0:02X}\n'.format(gyro))


rospy.loginfo('Reading BNO055 data, press Ctrl-C to quit...')



while not rospy.is_shutdown():
    # check for incoming messages
    rospy.spinOnce()

    # for header time stamps
    current_time = rospy.Time.now()

    # since all odometry is 6DOF we'll need a quaternion created from yaw
    # org: geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(z)
    odom_quat = tf.createQuaternionMsgFromYaw(z)

    # first, we'll publish the transform over tf
    geometry_msgs::TransformStamped odom_trans

    odom_trans.header.stamp = current_time
    odom_trans.header.frame_id = "odom"
    odom_trans.child_frame_id = "base_link"

    odom_trans.transform.translation.x = x
    odom_trans.transform.translation.y = y
    odom_trans.transform.translation.z = 0.0
    odom_trans.transform.rotation = odom_quat

    # send the transform
    odom_broadcaster.sendTransform(odom_trans)

    # define message header for IMU and temperature
    h = rospy.Header()
    h.stamp = current_time
    h.frame_id = frame_id  # "odom"
    h.seq = seq
    # increase sequence
    seq = seq + 1
    # add header to IMU message
    imu_msg.header = h

    # Read the Euler angles for heading, roll, pitch (all in degrees).
    heading, roll, pitch = bno.read_euler()

    # Read the calibration status, 0=uncalibrated and 3=fully calibrated.
    sys, gyro, accel, mag = bno.get_calibration_status()

    # Print everything out.
    # rospy.loginfo('Heading={0:0.2F} Roll={1:0.2F} Pitch={2:0.2F}\tSys_cal={3} Gyro_cal={4} Accel_cal={5} Mag_cal={6}'.format(heading, roll, pitch, sys, gyro, accel, mag))

    # publish Euler values
    pubH.publish(heading)
    pubR.publish(roll)
    pubP.publish(pitch)


    # Read orientation as a quaternion:
    x,y,z,w = bno.read_quaternion()
    imu_msg.orientation.x = x # is this the pose then?
    imu_msg.orientation.y = y
    imu_msg.orientation.z = z
    imu_msg.orientation.w = w
    # Print
    # rospy.loginfo('Quaternion: x={} y={} z={} w={}'.format(imu_msg.orientation.x, imu_msg.orientation.y, imu_msg.orientation.z, imu_msg.orientation.w))

    # Gyroscope data (in degrees per second):
    xg,yg,zg = bno.read_gyroscope()
    imu_msg.angular_velocity.x = xg;
    imu_msg.angular_velocity.y = yg;
    imu_msg.angular_velocity.z = zg;
    # Print
    rospy.loginfo('Gyroscope: x={} y={} z={}'.format(imu_msg.angular_velocity.x, imu_msg.angular_velocity.y, imu_msg.angular_velocity.z))

    # Accelerometer data (in meters per second squared):
    xa,ya,za = bno.read_accelerometer()
    imu_msg.linear_acceleration.x = xa;
    imu_msg.linear_acceleration.y = ya;
    imu_msg.linear_acceleration.z = za;
    # Print
    rospy.loginfo('Accelerometer: x={} y={} z={}'.format(imu_msg.linear_acceleration.x, imu_msg.linear_acceleration.y, imu_msg.linear_acceleration.z))

    # publish message
    pubImu.publish(imu_msg)


    # Read sensor temperature in degrees Celsius:
    temp = bno.read_temp()

    # Print
    # rospy.loginfo('Temperature: {}°C'.format(temp))

    # add header to temperature message
    temp_msg.header = h
    # publish message
    temp_msg.temperature = temp
    pubTemp.publish(temp_msg)

    # Other values you can optionally read:
    # Magnetometer data (in micro-Teslas):
    #x,y,z = bno.read_magnetometer()
    # Linear acceleration data (i.e. acceleration from movement, not gravity--
    # returned in meters per second squared):
    #x,y,z = bno.read_linear_acceleration()
    # Gravity acceleration data (i.e. acceleration just from gravity--returned
    # in meters per second squared):
    #x,y,z = bno.read_gravity()

    # Sleep for a second until the next reading.
    rospy.sleep(sleepTime)
