#!/usr/bin/env python
# coding=utf-8

"""
This is the ROS node for the minibot (https://minibot.me).

I expects "cmd_vel" geometry_msgs/Twist messages to control the robots motors.
It will then publish messages like "FORWARD, BACKWARD, LEFT, RIGHT, STOP",
which will be received by a "motor_server" node. The latter is responsible for
controlling the motors with lowlevel I2C commands on a Raspberry Pi.

This node can (also) be controlled via keyboard with the
teleop_twist_keyboard node.

Usage:
roslaunch minibot keyboard_motor_control.launch

Author:  Markus Knapp, 2017
Website: https://direcs.de
"""


import rospy
from geometry_msgs.msg import Twist

# The minibot package has the motor server, which listens to messages from here
from minibot.srv import *

# set the motor speed (from 0 (off) to 255 (max speed))
startSpeed = 100


# node init
rospy.init_node('keyboard_listener', anonymous=False)


# Service 'motor' from motor_server.py ready?
rospy.loginfo("Waiting for service 'motor'")
rospy.wait_for_service('motor')

#  this will execute the "drive" command
def drive(direction):
    # Send driving direction to motor
    try:
        # Create the handle 'motor_switcher' with the service type 'Motor'.
        # The latter automatically generates the MotorRequest and MotorResponse objects.
        motor_switcher = rospy.ServiceProxy('motor', Motor)

         # the handle can be called like a normal function
        rospy.loginfo("Switching motors to %s @ speed %s.", direction, startSpeed)
        response = motor_switcher(direction, startSpeed)

        # show result
        rospy.loginfo(rospy.get_caller_id() + ' says result is %s.', response.result)

    except rospy.ServiceException, e:
        rospy.logerr("Service call for 'motor' failed: %s", e)


def callback(data):
    # print out received message from the teleop_twist_keyboard
    # rospy.loginfo(rospy.get_caller_id() + ' received x=%s', data.linear.x)
    # rospy.loginfo(rospy.get_caller_id() + ' received z=%s', data.angular.z)

    # which command was received/key was pressed?
    if  (data.linear.x > 0.0) and (data.angular.z == 0.0):
      rospy.loginfo("FORWARD command.")
      drive("FORWARD")
    # k key
    elif  (data.linear.x == 0.0) and (data.angular.z == 0.0):
      rospy.loginfo("STOP command.")
      drive("STOP")
    # , key
    elif  (data.linear.x < 0.0) and (data.angular.z == 0.0):
      rospy.loginfo("BACKWARD command.")
      drive("BACKWARD")
    # j key
    elif  (data.linear.x == 0.0) and (data.angular.z > 0.0):
      rospy.loginfo("LEFT command.")
      drive("LEFT")
    # l key
    elif  (data.linear.x == 0.0) and (data.angular.z < 0.0):
      rospy.loginfo("BACKWARD command.")
      drive("RIGHT")


def listener():
    # subscribe the message cmd_vel
    rospy.Subscriber('cmd_vel', Twist, callback)

    # Ready
    rospy.loginfo("Ready. Control me via navigation stack/keyboard now.")

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
