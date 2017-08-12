#!/usr/bin/env python
# coding=utf-8

"""
This node does the transform tree setup for my robot 'minibot'.
It also publishes these transforms into ROS.

Author:  Markus Knapp, 2017
Website: https://direcs.de
"""

import logging
import rospy
import tf

if __name__ == '__main__':
    # init node
    rospy.init_node('robot_tf_broadcaster', anonymous=False)
    rospy.loginfo("robot_tf_broadcaster started.")

    # the broacaster (publisher)
    br = tf.TransformBroadcaster()
    rate = rospy.Rate(10.0)

    while not rospy.is_shutdown():
        """ laser_link transform """
        # Format: sendTransform(translation, rotation, time, child, parent)
        br.sendTransform((0.05, 0.0, 0.03), # @todo: check real laser x,y,z positions!
                         (0.0, 0.0, 0.0, 1.0),
                         rospy.Time.now(),
                         "laser_link", # child node
                         "base_link")   # parent node

        """ odom_link transform """
        br.sendTransform((0.0, 0.0, 0.03), # @todo: check real IMU x,y,z positions!
                         (0.0, 0.0, 0.0, 1.0),
                         rospy.Time.now(),
                         "odom_link", # child node
                         "base_link")   # parent node
        rate.sleep()
