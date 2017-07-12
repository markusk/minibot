#!/usr/bin/env python
# coding=utf-8

#
# ROS node for publishing a /imu/data topic.
#
# Based on the Simple Adafruit BNO055 sensor reading example.
# Markus Knapp, 2017. https://direcs.de
#
#
# Copyright (c) 2015 Adafruit Industries
# Author: Tony DiCola
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

from std_msgs.msg import Header
from sensor_msgs.msg import Imu, Temperature

from Adafruit_BNO055 import BNO055



# Create and configure the BNO sensor connection.
# Using I2C without a RST pin
bno = BNO055.BNO055()

# Initialize the BNO055 and stop if something went wrong.
if not bno.begin():
    rospy.logerr("Failed to initialize BNO055! Is the sensor connected?")

# Print system status and self test result.
status, self_test, error = bno.get_system_status()
rospy.loginfo('system status is {} {} {} '.format(status))
rospy.loginfo('Self test result (0x0F is normal): 0x{0:02X}'.format(self_test))

# Print out an error if system status is in error mode.
if status == 0x01:
    rospy.logerr('System error: {0}'.format(error))
    rospy.loginfo('See datasheet section 4.3.59 for the meaning.')

# Print BNO055 software revision and other diagnostic data.
sw, bl, accel, mag, gyro = bno.get_revision()
while True:
rospy.loginfo('Software version:   {0}'.format(sw))
rospy.loginfo('Bootloader version: {0}'.format(bl))
rospy.loginfo('Accelerometer ID:   0x{0:02X}'.format(accel))
rospy.loginfo('Magnetometer ID:    0x{0:02X}'.format(mag))
rospy.loginfo('Gyroscope ID:       0x{0:02X}\n'.format(gyro))


rospy.loginfo('Reading BNO055 data, press Ctrl-C to quit...')


    # Read the Euler angles for heading, roll, pitch (all in degrees).
    heading, roll, pitch = bno.read_euler()

    # Read the calibration status, 0=uncalibrated and 3=fully calibrated.
    sys, gyro, accel, mag = bno.get_calibration_status()

    # Print everything out.
    rospy.loginfo('Heading={0:0.2F} Roll={1:0.2F} Pitch={2:0.2F}\tSys_cal={3} Gyro_cal={4} Accel_cal={5} Mag_cal={6}'.format(
          heading, roll, pitch, sys, gyro, accel, mag))

    # Other values you can optionally read:
    # Orientation as a quaternion:
    #x,y,z,w = bno.read_quaterion()
    # Sensor temperature in degrees Celsius:
    #temp_c = bno.read_temp()
    # Magnetometer data (in micro-Teslas):
    #x,y,z = bno.read_magnetometer()
    # Gyroscope data (in degrees per second):
    #x,y,z = bno.read_gyroscope()
    # Accelerometer data (in meters per second squared):
    #x,y,z = bno.read_accelerometer()
    # Linear acceleration data (i.e. acceleration from movement, not gravity--
    # returned in meters per second squared):
    #x,y,z = bno.read_linear_acceleration()
    # Gravity acceleration data (i.e. acceleration just from gravity--returned
    # in meters per second squared):
    #x,y,z = bno.read_gravity()

    # Sleep for a second until the next reading.
    time.sleep(1)
