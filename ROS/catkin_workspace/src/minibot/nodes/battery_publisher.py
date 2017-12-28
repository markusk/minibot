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
# import power_supply.h

# we need a data type to publish the voltage
from std_msgs.msg import Header, Float32
# ros default battery messages
from sensor_msgs.msg import BatteryState

# initialise the node
rospy.init_node('battery_publisher')

# name of topic is 'voltage'
pubBattery = rospy.Publisher('voltage', BatteryState, queue_size=1)

# sleep time for this node in seconds
sleepTime = 1


# for reading the voltages
voltage = 0.0
value = 0
demoVoltage = 11.11


#
# for getting the hostname of the underlying system
import socket
# showing hostname
hostname = socket.gethostname()
rospy.loginfo("Running on host %s.", hostname)
if hostname != 'minibot':
    rospy.logwarn("Test mode only due to other hostname. Skipping all SPI staff!")
    rospy.logwarn("Using demo voltage of %.1f Volt" % demoVoltage)


""" ROS message header """
# define battery message
battery_msg = BatteryState()
frame_id = 'battery'
seq = 0


""" for the AD converter (connected to Raspberry Pi via SPI) """
# run some parts only on the real robot
if hostname == 'minibot':
    import Adafruit_ADS1x15

    # the Analog Digital converter object
    adc = Adafruit_ADS1x15.ADS1015()

    """ Gain 1 means, max a value of +4.096 Volt (+4,096 Volt in Europe) on the ADC channel, resulting in a 'value' of +2047. """
    GAIN = 1
    voltage = 0


while not rospy.is_shutdown():
    """ message header """
    # for header time stamps
    current_time = rospy.Time.now()

    h = rospy.Header()
    h.stamp = current_time
    h.frame_id = frame_id  # "battery"
    h.seq = seq
    # increase sequence
    seq = seq + 1
    # add header to IMU message
    battery_msg.header = h


    # run some parts only on the real robot
    if hostname == 'minibot':
        # read AD converter (battery voltage)
        # use channel 0 on IC
        value = adc.read_adc(0, gain=GAIN)
        # 13.435 Volt battery voltage resulted in 3.845(! wtf?) Volt on the ADC channel with my circuit (voltage divider).
        # This resulted in a ADC 'value' of 1896.
        # The conversion factor for the battery voltage is then: 1896 / 13.435 = 141.123930033494604
        #
        voltage = (value / 141.123930033494604)
    else:
        # simulated value!
        voltage = demoVoltage

    # print out pure ADC voltage
    # rospy.loginfo("Battery: %.1f Volt" % voltage)

    # completing the ROS message
    """
    # Other battery state are not provided:
    battery_msg.current         = Float32('nan')
    battery_msg.current         = Float32('nan')
    battery_msg.charge          = Float32('nan')
    battery_msg.capacity        = Float32('nan')
    battery_msg.design_capacity = Float32('nan')
    battery_msg.percentage      = Float32('nan')
    battery_msg.current         = Float32('nan')
    """
    battery_msg.current         = 0
    battery_msg.charge          = 0
    battery_msg.capacity        = 0
    battery_msg.design_capacity = 0
    battery_msg.percentage      = 0

    """
    battery_msg.power_supply_status = POWER_SUPPLY_STATUS_DISCHARGING
    battery_msg.power_supply_health = POWER_SUPPLY_HEALTH_GOOD
    battery_msg.power_supply_technology = POWER_SUPPLY_TECHNOLOGY_LIPO
    """
    battery_msg.present = True

    # this is the battery voltage
    # @TODO strange, this assignment as to be exactly here...
    battery_msg.voltage = voltage

    # print out voltage message
    # rospy.loginfo("Battery: %.1f Volt" % battery_msg.voltage)

    # publish voltage
    pubBattery.publish(battery_msg)

    # Sleep for a second until the next reading.
    rospy.sleep(sleepTime)
