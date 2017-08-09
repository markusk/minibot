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

# we need a data type to publish
from std_msgs.msg import Int32

# initialise the node
rospy.init_node('battery_publisher')

# name of topic is 'voltage'
pub = rospy.Publisher('voltage', Int32, queue_size=10)


# BEGIN LOOP
rate = rospy.Rate(2)

voltage = 0
while not rospy.is_shutdown():
    pub.publish(voltage)

    # @todo: read real battery voltage here
    voltage += 1

    rate.sleep()
# END LOOP
