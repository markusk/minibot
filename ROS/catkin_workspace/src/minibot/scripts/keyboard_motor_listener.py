#!/usr/bin/env python

## Simple keyboard_listener demo
## that listens to geometry_msgs/Twist published by the 'cmd_vel' topic
## of the teleop_twist_keyboard node
#
# Usage:
# roslaunch minibot keyboard_motor_control.launch

import rospy
from geometry_msgs.msg import Twist

# The minibot package has the motor server, which listens to messages from here
from minibot.srv import *

# set the motor speed (from 0 (off) to 255 (max speed))
startSpeed = 100


#  this will execute the "drive" command
def drive(direction):
    # Service 'motor' from motor_server.py ready?
    rospy.wait_for_service('motor')

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
    rospy.loginfo(rospy.get_caller_id() + ' received x=%s', data.linear.x)
    rospy.loginfo(rospy.get_caller_id() + ' received z=%s', data.angular.z)

    # which key / Speed was pressed?
    #
    if  (data.linear.x == 1.0):
      rospy.loginfo("FORWARD key pressed.")
      drive("FORWARD")
    #
    elif (data.linear.x == 2.0):
      rospy.loginfo("BACKWARD key pressed.")
      drive("BACKWARD")
    #
    elif (data.linear.x == 3.0):
      rospy.loginfo("LEFT key pressed.")
      drive("LEFT")
    #
    elif (data.linear.x == 4.0):
      rospy.loginfo("RIGHT key pressed.")
      drive("RIGHT")
    #
    elif (data.linear.x == 5.0):
      rospy.loginfo("STOP key pressed.")
      drive("STOP")


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
