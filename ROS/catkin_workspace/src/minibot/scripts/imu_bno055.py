#!/usr/bin/env python
# coding=utf-8

#include<ros/ros.h>
import rospy
#include<geometry_msgs/Twist.h>
from geometry_msgs.msg import Twist

#include<sensor_msgs/Imu.h>
from sensor_msgs.msg import Imu

#include<iostream>
#include<tf/LinearMath/Matrix3x3.h>
#include<tf/LinearMath/Quaternion.h>

#using namespace std;


def callback(imu):
	# tf::Quaternion bq(imu->orientation.x, imu->orientation.y, imu->orientation.z, imu->orientation.w);
	# double roll,pitch,yaw;
	# tf::Matrix3x3(bq).getRPY(roll,pitch,yaw);

	# convert to twist message for turtle
	vel = Twist()
	vel.linear.x  = imu.linear.x # pitch
	vel.angular.z = imu.angular.z # roll

	# debug messages
	rospy.loginfo(rospy.get_caller_id() + ' Sending x=%s to turtle', vel.linear.x)
	rospy.loginfo(rospy.get_caller_id() + ' Sending z=%s to turtle', vel.angular.z)

	# don't be too fast
        rate.sleep()

	# publish
	pub.publish(vel)


def listener():
	# In ROS, nodes are uniquely named. The anonymous=True flag
	# means that rospy will choose a unique name for this listener node
	rospy.init_node('teleopImu')

	# sleep time for publish/refresh
	rate = rospy.Rate(1) # 1hz

	# publish topic is cmd_vel
#	pub = rospy.Publisher("/turtle1/cmd_vel", Twist, queue_size=10)
	rospy.loginfo("Publishing 'turtle1/cmd_vel'")
	pub = rospy.Publisher('turtle1/cmd_vel', Twist, queue_size=10)

	# Service 'bno055_driver' from bno055_driver.py ready?
#	rospy.loginfo("Waiting for service 'bno055_driver'")
#	rospy.wait_for_service('bno055_driver')

	# subscribe (listen) to IMU data
	rospy.Subscriber('imu/data', Imu, callback)

	# Ready
	rospy.loginfo("Ready. Start Turtlesim and move sensor around to move the turtle.")

	# spin() simply keeps python from exiting until this node is stopped
	rospy.spin()


if __name__ == '__main__':
	listener()
