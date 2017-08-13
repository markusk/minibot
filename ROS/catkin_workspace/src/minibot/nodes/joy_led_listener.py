#!/usr/bin/env python
# coding=utf-8


"""
This is my listener for the joy_node. It listens on the topic 'joy' and prints out some information.
I then switches a LED on my Raspberry when button 1 is used on the joystick/gamepad.
This needs the led_server to be run on the Raspberry Pi. Like this:

Usage
-----
Raspberry Pi:
1. roslaunch minibot led_server
   (This starts also the roscore on this computer automatically).

Another Ubuntu machine:
1. export ROS_MASTER_URI=http://hostname-of-your-pi:11311/ from the robot. I.E.:
   export ROS_MASTER_URI=http://minibot:11311/
2. Set joystick device (if different) to js0. I.E.:
   rosparam set joy_node/dev "/dev/input/js2"
3. roslaunch minibot joystick_led
4. Press button 1 on the joystick/gamepad and see the output (the LED turing ON or OFF).


Author:  Markus Knapp, 2017
Website: https://direcs.de
"""

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Joy
# name of the package(!).srv
from minibot.srv import *


# stores the state of the LED pin/GPIO
ledPin = False


def callback(joy):
    # need to modify the global variable
    global ledPin
    # button 1 (index 0) pressed (1)?
    if (joy.buttons[0] == 1):
      rospy.loginfo('Button 1 pressed!')
      #
      # turn a led ON
      #
      # Service 'led' from led_server.py ready?
      rospy.wait_for_service('led')

      try:
          # Create the handle 'led_switcher' with the service type 'Led'.
          # The latter automatically generates the LedRequest and LedResponse objects.
          led_switcher = rospy.ServiceProxy('led', Led)

          # if LED is stored as OFF, turn it ON
          if (ledPin == False):
              # Turn LED ON
              # store new LED state
              ledPin = True
              # the handle can be called like a normal function
              rospy.loginfo('Turning LED on.')
              response = led_switcher(18, 0)
          else:
              # Turn LED OFF
              # store new LED state
              ledPin = False
              # the handle can be called like a normal function
              rospy.loginfo('Turning LED off.')
              response = led_switcher(18, 1)

          # show result
          rospy.loginfo(rospy.get_caller_id() + ' says result is %s.', response.result)

      except rospy.ServiceException, e:
          rospy.logerr("Service call for 'led' failed: %s", e)

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
    rospy.loginfo('Ready. Press button 1 on your joystick now.')

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    listener()
