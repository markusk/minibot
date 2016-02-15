#!/usr/bin/env python
# coding=utf-8

"""
This node is for observing (subscribing) the robots IMU (Bosch BNO055) data.
It than publishes a /turtle1/cmd_vel message to control the famous turtle sim.
We use a publisher, since it is will be published non-stop. And is not critical
if we might loose a message.
"""

#include<ros/ros.h>
import rospy
#include<geometry_msgs/Twist.h>
from geometry_msgs.msg import Twist

#include<sensor_msgs/Imu.h>
from sensor_msgs.msg import Imu

#include<iostream>
#include<tf/LinearMath/Matrix3x3.h>
#include<tf/LinearMath/Quaternion.h>
import math
import tf

# Euler stuff
roll = 0
pitch = 0
yaw = 0

#using namespace std;


def callback(imu):
	# publish topic is cmd_vel
	#	pub = rospy.Publisher("/turtle1/cmd_vel", Twist, queue_size=10)
	pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)

	# Ready
	rospy.loginfo("Publishing '/turtle1/cmd_vel'")

	# tf::Quaternion bq(imu->orientation.x, imu->orientation.y, imu->orientation.z, imu->orientation.w);
	# double roll,pitch,yaw;
	# tf::Matrix3x3(bq).getRPY(roll,pitch,yaw);

    # Convert quaternions to Euler angles. See: http://answers.ros.org/question/11545/plotprint-rpy-from-quaternion/
	(roll, pitch, yaw) = tf.transformations.euler_from_quaternion([imu.orientation.x, imu.orientation.y, imu.orientation.z, imu.orientation.w])

	# convert to twist message for turtle
	vel = Twist()
	vel.linear.x  = pitch
	vel.angular.z = roll

	# debug messages
"""	rospy.loginfo(rospy.get_caller_id() + ' IMU x=%s', imu.orientation.x)
	rospy.loginfo(rospy.get_caller_id() + ' IMU y=%s', imu.orientation.y)
	rospy.loginfo(rospy.get_caller_id() + ' IMU z=%s', imu.orientation.z)
	rospy.loginfo(rospy.get_caller_id() + ' IMU w=%s', imu.orientation.w)
"""
	#rospy.loginfo(rospy.get_caller_id() + ' r=%s', roll)
	#rospy.loginfo(rospy.get_caller_id() + ' p=%s', pitch)
	#rospy.loginfo(rospy.get_caller_id() + ' y=%s', yaw)

	# publish
#	rospy.loginfo(rospy.get_caller_id() + ' Sending x=%s to turtle', vel.linear.x)
#	rospy.loginfo(rospy.get_caller_id() + ' Sending z=%s to turtle', vel.angular.z)
	pub.publish(vel)

	# sleep 0.5 seconds
	rospy.sleep(0.5)


def listener():
	# node init
	rospy.init_node('imo_to_turtle_publisher')

	# Service 'bno055_driver' from bno055_driver.py ready?
	#	rospy.loginfo("Waiting for service 'bno055_driver'")
	#	rospy.wait_for_service('bno055_driver')

	# subscribe (listen) to IMU data
	rospy.Subscriber('/imu/data', Imu, callback)

	# Ready
	rospy.loginfo("Ready. Start Turtlesim and move sensor around to move the turtle.")

	# spin() simply keeps python from exiting until this node is stopped
	rospy.spin()


if __name__ == '__main__':
	listener()
