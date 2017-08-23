#!/usr/bin/env python
# coding=utf-8

"""
This publisher is for observing the robots battery
and publishing the voltage as message
We use a publisher, since it is will be published non-stop.
And is not critical if we might loose a message.

@todo: change format to sensor_msgs/BatteryState

Author:  Markus Knapp, 2017
Website: https://direcs.de
"""

import rospy
import time # for ROS message header timestamp
import math # for NaN values (Not a Number)

# we need a data type to publish the voltage
from std_msgs.msg import Header, Float32
# ros default battery messages
from sensor_msgs.msg import BatteryState

# initialise the node
rospy.init_node('battery_publisher')

# name of topic is 'voltage'
pubBattery = rospy.Publisher('voltage', BatteryState, queue_size=1)
# pubImu  = rospy.Publisher('imu/data', Imu, queue_size=1)

# Begin loop @ one Hz
rate = rospy.Rate(1)


#
# for getting the hostname of the underlying system
import socket
# showing hostname
hostname = socket.gethostname()
rospy.loginfo("Running on host %s.", hostname)
if hostname != 'minibot':
    rospy.logwarn("Test mode only due to other host. Skipping all SPI staff!")
    demoVoltage = 11.11
    rospy.logwarn("Using demo voltage of %.1f Volt" % demoVoltage)


""" ROS message header """
# define IMU message
battery_msg = BatteryState()
seq = 0


""" for the AD converter (connected to Raspberry Pi via SPI) """
# run some parts only on the real robot
if hostname == 'minibot':
    from MCP3008 import MCP3008
    adc = MCP3008()

voltage = 0.0
value = 0


while not rospy.is_shutdown():
    """ message header """
    # for header time stamps
    current_time = rospy.Time.now()

    h = rospy.Header()
    h.stamp = current_time
    h.frame_id = "battery"
    h.seq = seq
    # increase sequence
    seq = seq + 1
    # add header to IMU message
    battery_msg.header = h


    # run some parts only on the real robot
    if hostname == 'minibot':
        # read AD converter (battery voltage)
        # use channel 0 on IC
        value = adc.read(channel = 0)
        # 2.73 V = 12.32 V (measured) > 1023 / 3.3 * 2.73 / 12.32 = 68.693182
        voltage = (value / 68.693182)
    else:
        # simulated value!
        voltage = demoVoltage

    # print out pure ADC voltage
    # rospy.loginfo("Battery: %.1f Volt" % voltage)

    # completing the ROS message
    battery_msg.voltage = voltage

    """
    # Other battery state are not provided:
    battery_msg.current         = Float32('nan')
    battery_msg.current         = Float32('nan')
    battery_msg.charge          = Float32('nan')
    battery_msg.capacity        = Float32('nan')
    battery_msg.design_capacity = Float32('nan')
    battery_msg.percentage      = Float32('nan')
    """

    # print out voltage message
    rospy.loginfo("Battery: %.1f Volt" % battery_msg.voltage)

    # publish voltage
    pubBattery.publish(battery_msg)


    rate.sleep()
