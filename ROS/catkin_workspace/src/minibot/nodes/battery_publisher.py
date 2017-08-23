#!/usr/bin/env python
# coding=utf-8

"""
This publisher is for observing the robots battery
and publishing the voltage as message
We use a publisher, since it is will be published non-stop.
And is not critical if we might loose a message.

Author:  Markus Knapp, 2017
Website: https://direcs.de
"""

import rospy

# we need a data type to publish the voltage
from std_msgs.msg import Float32

# initialise the node
rospy.init_node('battery_publisher')

# name of topic is 'voltage'
pub = rospy.Publisher('voltage', Float32, queue_size=10)

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


#
# for the AD converter (connected to Raspberry Pi via SPI)
#
# run some parts only on the real robot
if hostname == 'minibot':
    from MCP3008 import MCP3008
    adc = MCP3008()

voltage = 0.0
value = 0


while not rospy.is_shutdown():
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

    # print out voltage
    # rospy.loginfo("Battery: %.1f Volt" % voltage)

    # publish voltage
    pub.publish(voltage)


    rate.sleep()
