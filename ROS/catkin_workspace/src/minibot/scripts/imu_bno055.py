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

# c variables
ros::NodeHandle n;
ros::Publisher pub;
ros::Subscriber sub;


def TeleopImu():
	pub = n.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel",1);
	sub = n.subscribe<sensor_msgs::Imu>("imu/data", 10, &TeleopImu::callBack, this);


def callBack(const sensor_msgs::Imu::ConstPtr& imu):
	geometry_msgs::Twist vel;
	tf::Quaternion bq(imu->orientation.x,imu->orientation.y,imu->orientation.z,imu->orientation.w);
	double roll,pitch,yaw;
	tf::Matrix3x3(bq).getRPY(roll,pitch,yaw);
	vel.angular.z = roll;
	vel.linear.x = pitch;
	pub.publish(vel);


def listener():

    # In ROS, nodes are uniquely named. The anonymous=True flag
    # means that rospy will choose a unique name for this listener node
    rospy.init_node('teleopImu', anonymous=True)

    # Ready
    rospy.loginfo("Ready. Start Turtlesim and move sensor around to move the turtle.")

	TeleopImu teleop_turtle;

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    listener()