#!/usr/bin/env python
# coding=utf-8

"""
This node does the transform tree setup for my robot 'minibot'.
It also publishes these transforms into ROS.
"""

import logging
import rospy
import tf

if __name__ == '__main__':
    # init node
    rospy.init_node('robot_tf_broadcaster')
    rospy.loginfo("robot_tf_broadcaster started.")

    # the broacaster (publisher)
    br = tf.TransformBroadcaster()
    rate = rospy.Rate(10.0)

    while not rospy.is_shutdown():
        # send laser_link transform
        br.sendTransform((0.0, 0.0, 0.0, 1.0),
                         (0.5, 0.1, 0.0), # @todo: check real measurements!
                         rospy.Time.now(),
                         "base_link",   # parent node
                         "laser_link") # child node
                         # @todo: what about IMU here?!?
        rate.sleep()
