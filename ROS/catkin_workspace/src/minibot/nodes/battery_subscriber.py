#!/usr/bin/env python
# coding=utf-8

"""
Author:  Markus Knapp, 2018
Website: https://direcs.de
"""

import rospy
from std_msgs.msg import Float32

def callback(message):
    # log and print received message
    rospy.loginfo("Battery voltage: %s V.", message.data)

def listener():
    # init a subscriber node with any name
    rospy.init_node('voltage_subscriber')
    # first argument is the name of the topic we want to listen to
    rospy.Subscriber("voltage", Float32, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    listener()
