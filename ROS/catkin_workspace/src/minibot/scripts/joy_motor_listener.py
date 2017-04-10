#!/usr/bin/env python
# coding=utf-8


""" This is my listener for the joy_node. It listens on the topic 'joy' and prints out some information.
    I then switches on motors on my Raspberry when the D-Pad is used on the joystick/gamepad.
    This needs the motor_server to be run on the Raspberry Pi. Like this:

Usage
-----

Raspberry Pi:
1. Run: roscore
2. Run the motor server on the Pi: rosrun minibot motor_server.py

Another Ubuntu machine:
1. export ROS_MASTER_URI=http://hostname-of-your-pi:11311/
   i.E. export ROS_MASTER_URI=http://pi-desktop:11311/
2. Set joystick device if different to js0: rosparam set joy_node/dev "/dev/input/js2"
3. In the next terminal window repeat step 1. (export...)
4. Run the joystick node: rosrun joy joy_node
5. Run this listener: rosrun minibot joy_motor_listener
6. Use the D-Pad buttons on the joystick/gamepad to control the motors.

"""

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Joy
# name of the package(!).srv
from minibot.srv import *


# set the motor speed (from 0 (off) to 255 (max speed))
startSpeed = 100


def callback(joy):
    # D-Pad, vertikal up
    if (joy.axes[5] == -32767):
      rospy.loginfo("Forward button pressed.")

      # Service 'motor' from motor_server.py ready?
      rospy.wait_for_service('motor')

      # Send driving direction to motor
      try:
          # Create the handle 'motor_switcher' with the service type 'Motor'.
          # The latter automatically generates the MotorRequest and MotorResponse objects.
          motor_switcher = rospy.ServiceProxy('motor', Motor)
          # the handle can be called like a normal function
          rospy.loginfo("Switching motors to forward @ speed %s.", startSpeed)
          response = motor_switcher("FORWARD", startSpeed)

          # show result
          rospy.loginfo(rospy.get_caller_id() + ' says result is %s.', response.result)

      except rospy.ServiceException, e:
          rospy.logerr("Service call for 'motor' failed: %s", e)

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
