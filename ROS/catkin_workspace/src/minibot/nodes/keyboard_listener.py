#!/usr/bin/env python
# coding=utf-8

"""
Simple keyboard_listener demo
that listens to geometry_msgs/Twist published by the 'cmd_vel' topic
of the teleop_twist_keyboard node

Usage:
roslaunch minibot keyboard_control.launch

Author:  Markus Knapp, 2017
Website: https://direcs.de
"""


import rospy
from geometry_msgs.msg import Twist


def callback(data):
    # print out received message from the teleop_twist_keyboard
    rospy.loginfo(rospy.get_caller_id() + ' x=%s', data.linear.x)
    rospy.loginfo(rospy.get_caller_id() + ' y=%s', data.linear.y)
    rospy.loginfo(rospy.get_caller_id() + ' z=%s', data.angular.z)


def listener():
    # In ROS, nodes are uniquely named. The anonymous=True flag
    # means that rospy will choose a unique name for this listener node
    rospy.init_node('keyboard_listener', anonymous=True)

    # subscribe the message cmd_vel
    rospy.Subscriber('cmd_vel', Twist, callback)

    # Ready
    rospy.loginfo("Ready. Control me via keyboard now.")

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    listener()
