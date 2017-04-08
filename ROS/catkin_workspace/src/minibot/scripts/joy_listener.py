#!/usr/bin/env python
# coding=utf-8


""" This is my listener for the joy_node. It listens on the topic 'joy' and prints out some information.
    I also switches a LED on on my Raspberry (GPIO 18) when button 1 is pressed.
    This needs the led_listener to be run on the Raspberry Pi. Like this:

Usage
-----

Raspberry Pi:
1. Run: roscore
2. Run the led server on the Pi: rosrun minibot led_server.py

Another Ubuntu machine:
1. export ROS_MASTER_URI=http://hostname-of-your-pi:11311/
   i.E. export ROS_MASTER_URI=http://pi-desktop:11311/
2. Set joystick device if different to js0: rosparam set joy_node/dev "/dev/input/js2"
3. Run the joystick node: rosrun joy joy_node
4. Run this listener: rosrun minibot joy_listener
5. Press button 1 on th joystick and see the output. En-Joy! ;-)

"""

import rospy
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
          print "Service call for 'led' failed: %s"%e

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
