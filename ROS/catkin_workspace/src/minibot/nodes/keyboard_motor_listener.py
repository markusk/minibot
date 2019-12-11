#!/usr/bin/env python
# coding=utf-8

"""
Simple keyboard_listener demo
that listens to geometry_msgs/Twist published by the 'cmd_vel' topic
of the teleop_twist_keyboard node

Usage:
roslaunch minibot keyboard_motor_control.launch

Author:  Markus Knapp, 2019
Website: https://direcs.de
"""


import rospy
from geometry_msgs.msg import Twist

# The minibot package has the motor server, which listens to messages from here
from minibot.srv import *

# Getting robot parameters
rospy.loginfo('Getting parameters for robot.')
# speed of the motors (0-255).
drivingSpeed = rospy.get_param('/minibot/drivingSpeed')
rospy.loginfo('Using drivingSpeed %s.', drivingSpeed)
# the speed when turning the bot can be higher if needed (higher friction)
turnSpeed = rospy.get_param('/minibot/turnSpeed')
rospy.loginfo('Using turnSpeed %s.', turnSpeed)

# Service 'motor' from motor_server.py ready?
rospy.loginfo("Waiting for service 'motor'")
rospy.wait_for_service('motor')


#  this will execute the "drive" command
def drive(direction, speed):
    # Send driving direction to motor
    try:
        # Create the handle 'motor_switcher' with the service type 'Motor'.
        # The latter automatically generates the MotorRequest and MotorResponse objects.
        motor_switcher = rospy.ServiceProxy('motor', Motor)

         # the handle can be called like a normal function
        rospy.loginfo("Switching motors to %s @ speed %s.\n", direction, speed)
        response = motor_switcher(direction, speed)

        # show result
        rospy.loginfo(rospy.get_caller_id() + ' says result is %s.\n', response.result)

    except rospy.ServiceException, e:
        rospy.logerr("Service call for 'motor' failed: %s", e)


def callback(data):
    # print out received message from the teleop_twist_keyboard
    # rospy.loginfo(rospy.get_caller_id() + ' received x=%s', data.linear.x)
    # rospy.loginfo(rospy.get_caller_id() + ' received z=%s', data.angular.z)

    # which key / Speed was pressed?
    # i key
    if  (data.linear.x > 0.0) and (data.angular.z == 0.0):
      rospy.loginfo("FORWARD key pressed.")
      drive("FORWARD", drivingSpeed)
    # , key
    elif  (data.linear.x < 0.0) and (data.angular.z == 0.0):
      rospy.loginfo("BACKWARD key pressed.")
      drive("BACKWARD", drivingSpeed)
    # j key
    elif  (data.linear.x == 0.0) and (data.angular.z > 0.0):
      rospy.loginfo("LEFT key pressed.")
      drive("LEFT", turnSpeed)
    # l key
    elif  (data.linear.x == 0.0) and (data.angular.z < 0.0):
      rospy.loginfo("BACKWARD key pressed.")
      drive("RIGHT", turnSpeed)
    # k key
    elif  (data.linear.x == 0.0) and (data.angular.z == 0.0):
      rospy.loginfo("STOP key pressed.")
      drive("STOP", 0)


def listener():

    # In ROS, nodes are uniquely named. The anonymous=True flag
    # means that rospy will choose a unique name for this listener node
    rospy.init_node('keyboard_listener', anonymous=True)

    # subscribe the message cmd_vel
    rospy.Subscriber('cmd_vel', Twist, callback)

    # Ready
    rospy.loginfo("Ready. Control me via keyboard now.")

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
