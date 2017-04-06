#!/usr/bin/env python
# coding=utf-8


""" This is a simple listener for the joy_node. It listens on the topic 'joy' and prints out some information.

Usage:
1. Start roscore
2. Set joystick device if different to js0: rosparam set joy_node/dev "/dev/input/js2"
3. Run the joystick node: rosrun joy joy_node
4. Run this listener: rosrun minibot joy_listener
5. Press button 1 on th joystick and see the output. En-Joy! ;-)

"""

import rospy
from sensor_msgs.msg import Joy

def callback(joy):
    # simple:
    if (joy.buttons[0] == 1):
      rospy.loginfo('Button 1 pressed!')


def listener():
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'joy_listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('joy_listener', anonymous=True)

    # subscribe the joy(stick) topic
    rospy.Subscriber('joy', Joy, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    listener()
