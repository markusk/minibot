#!/usr/bin/env python
# coding=utf-8


"""
This is my listener for the joy_node. It listens on the topic 'joy' and prints out some information.
I then switches on motors on my Raspberry when the D-Pad is used on the joystick/gamepad.
This needs the motor_server to be run on the Raspberry Pi. Like this:

Usage
-----
Raspberry Pi:
1. roslaunch minibot motor_server. (This starts also the roscore on this computer automatically).

Another Ubuntu machine:
1. export ROS_MASTER_URI=http://hostname-of-your-pi:11311/ from the robot. I.E.:
   export ROS_MASTER_URI=http://minibot:11311/
2. Set joystick device (if different) to js0. I.E.:
   rosparam set joy_node/dev "/dev/input/js2"
3. roslaunch minibot joystick_control


Author:  Markus Knapp, 2017
Website: https://direcs.de
"""

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Joy
# name of the package(!).srv
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


def callback(joy):
    """
    # debug messages
    rospy.loginfo("axis 0: %s", joy.axes[0])
    rospy.loginfo("axis 1: %s", joy.axes[1])
    rospy.loginfo("axis 2: %s", joy.axes[2])
    rospy.loginfo("axis 3: %s", joy.axes[3])
    rospy.loginfo("axis 4: %s", joy.axes[4])
    rospy.loginfo("axis 5: %s", joy.axes[5])
    rospy.loginfo("-------------------")
    """

    # which button was pressed?
    # D-Pad, vertikal up
    if   (joy.axes[5] == 1.0):
      rospy.loginfo("FORWARD button pressed.")
      drive("FORWARD")
    # D-Pad, vertikal down
    elif (joy.axes[5] == -1.0):
      rospy.loginfo("BACKWARD button pressed.")
      drive("BACKWARD")
    # D-Pad, horizontal left
    elif (joy.axes[4] ==  1.0):
      rospy.loginfo("LEFT button pressed.")
      drive("LEFT")
    # D-Pad, horizontal right
    elif (joy.axes[4] == -1.0):
      rospy.loginfo("RIGHT button pressed.")
      drive("RIGHT")
    # red button on my gamepad
    elif (joy.buttons[10] == 1.0):
      rospy.loginfo("RED button pressed.")
      drive("STOP")


def listener():
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'joy_listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('joy_listener', anonymous=True)

    # subscribe the joy(stick) topic
    rospy.Subscriber('joy', Joy, callback)

    # Ready
    rospy.loginfo("Ready. Press a button on your joystick D-Pad now.")

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    listener()
