#!/usr/bin/env python
# coding=utf-8

"""
This is a service node (server). It receives intructions to switch a LED.

Author:  Markus Knapp, 2017
Website: https://direcs.de
"""

# name of the package(!).srv
from minibot.srv import *
import rospy


# Service nodes have to be initialised
rospy.init_node('led_server')


# for getting the hostname of the underlying system
import socket
# showing hostname
hostname = socket.gethostname()
rospy.loginfo("Running on host %s.", hostname)


rospy.loginfo("Setting up RPi.GPIO...")
# run some parts only on the real robot
if hostname == 'minibot':
    # for GPIO pin usage on the Raspberry Pi
    try:
        import RPi.GPIO as GPIO
    except RuntimeError:
        rospy.logerr("Error importing RPi.GPIO!")
else:
    rospy.loginfo("NOT setup. Simulating due to other host!")


## GPIO stuff
# We use the GPIO names, _not_ the pin numbers on the board
if hostname == 'minibot':
    GPIO.setmode(GPIO.BCM)
# Raspberry Pi pin configuration:
# pins	    BCM   BOARD
ledPin    = 18 # pin 12

# setting these LOW at startup
# pinListLow = (ledPin)

# setting these HIGH at startup
pinListHigh = (ledPin)

# GPIO setup
rospy.loginfo("GPIO setup...")
if hostname == 'minibot':
    GPIO.setup(ledPin, GPIO.OUT)

# all pins which should be LOW at ini
# GPIO.output(pinListLow, GPIO.LOW)

# all pins which should be HIGH at ini
if hostname == 'minibot':
    GPIO.output(pinListHigh, GPIO.HIGH)


# define a clean node exit
def my_exit():
  rospy.loginfo("Shutting LED server down...")
  if hostname == 'minibot':
      # GPIO cleanup
      GPIO.cleanup()
  rospy.loginfo("Done.")

# call this method on node exit
rospy.on_shutdown(my_exit)


# handle_led is called with instances of LedRequest and returns instances of LedResponse
# The request name comes directly from the .srv filename
def handle_led(req):
    """ In this function all the work is done :) """

    # switch GPIO to HIGH, if '1' was sent
    if (req.state == 1):
        if hostname == 'minibot':
            GPIO.output(req.pin, GPIO.HIGH)
    else:
      # for all other values we set it to LOW
      # (LEDs are low active!)
      if hostname == 'minibot':
          GPIO.output(req.pin, GPIO.LOW)

    # debug
    rospy.loginfo("GPIO %s switched to %s. Result: %s", req.pin, req.state, req.pin)

    # The name of the 'xyzResponse' comes directly from the Xyz.srv filename!
    return LedResponse(req.pin)


def led_server():
    # This declares a new service named 'led'' with the 'Led' service type.
    # All requests are passed to the 'handle_led' function.
    # 'handle_led' is called with instances of LedRequest and returns instances of LedResponse
    s = rospy.Service('led', Led, handle_led)
    if hostname != 'minibot':
        rospy.logwarn("SIMULATING due the fact that the hostname is not 'minibot'.")
    rospy.loginfo("Ready to switch LEDs.")

    # Keep our code from exiting until this service node is shutdown
    rospy.spin()


if __name__ == "__main__":
    led_server()
