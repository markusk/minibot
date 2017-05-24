#!/usr/bin/env python

## Simple keyboard_listener demo
## that listens to geometry_msgs/Twist published by the 'cmd_vel' topic
## of the teleop_twist_keyboard node

import rospy
from geometry_msgs.msg import Twist

def callback(data):
    # print out received message from the teleop_twist_keyboard
    rospy.loginfo(rospy.get_caller_id() + 'I received %s', data.linear.x)

def listener():

    # In ROS, nodes are uniquely named. The anonymous=True flag
    # means that rospy will choose a unique name for this listener node
    rospy.init_node('keyboard_listener', anonymous=True)

    # subscribe the message cmd_vel
    rospy.Subscriber('cmd_vel', Twist, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
