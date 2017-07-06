#!/usr/bin/env python
# coding=utf-8

#include<ros/ros.h>
import rospy
#include<geometry_msgs/Twist.h>
from geometry_msgs.msg import Twist

#include<sensor_msgs/Imu.h>
from sensr_msgs import Imu

#include<iostream>
#include<tf/LinearMath/Matrix3x3.h>
#include<tf/LinearMath/Quaternion.h>

#using namespace std;

# Service 'bno055_driver' from bno055_driver.py ready?
rospy.loginfo("Waiting for service 'bno055_driver'")
rospy.wait_for_service('bno055_driver')


def work():
	# publish topic is cmd_vel
#	pub = rospy.Publisher("/turtle1/cmd_vel", Twist, queue_size=10)
	pub = rospy.Publisher('turtle1/cmd_vel', Twist, queue_size=10)

	# listen to IMU data
	rospy.Subscriber('imu/data', Imu, callback)


def callBack(imu):
	Twist vel
	# tf::Quaternion bq(imu->orientation.x, imu->orientation.y, imu->orientation.z, imu->orientation.w);
	# double roll,pitch,yaw;
	# tf::Matrix3x3(bq).getRPY(roll,pitch,yaw);

	# convert to twist message for turtle
	vel.angular.z = imu.angular.z # roll
	vel.linear.x  = imu.linear.x # pitch

	# publish
	pub.publish(vel)


def listener():
    # In ROS, nodes are uniquely named. The anonymous=True flag
    # means that rospy will choose a unique name for this listener node
    rospy.init_node('teleopImu', anonymous=True)

    # Ready
    rospy.loginfo("Ready. Start Turtlesim and move sensor around to move the turtle.")

    # subscribe and publish
    work()

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    listener()
