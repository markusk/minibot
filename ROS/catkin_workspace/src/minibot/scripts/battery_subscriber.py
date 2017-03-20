#!/usr/bin/env python

import rospy
from std_msgs.msg import Int32

def callback(message):
    # log and print received message
    rospy.loginfo("Battery voltage %s", message.data)

def listener():
    # init a subscriber node with any name
    rospy.init_node('voltage_subscriber')
    # first argument is the name of the topic we want to listen to
    rospy.Subscriber("voltage", Int32, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    listener()
