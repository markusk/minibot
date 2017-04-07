#!/usr/bin/env python
# coding=utf-8

# This is a service node (server)

# name of the package(!).srv
from minibot.srv import *
import rospy

# for GPIO pin usage on the Raspberry Pi
try:
    import RPi.GPIO as GPIO
except RuntimeError:
    print("Error importing RPi.GPIO!")


## GPIO stuff
# We use the GPIO names, _not_ the pin numbers on the board
GPIO.setmode(GPIO.BCM)
# Raspberry Pi pin configuration:
# pins	    BCM   BOARD
ledPin    = 18 # pin 12

# setting these LOW at startup
# pinListLow = (ledPin)

# setting these HIGH at startup
pinListHigh = (ledPin)

# GPIO setup
print "GPIO setup..."
GPIO.setup(ledPin, GPIO.OUT)

# all pins which should be LOW at ini
# GPIO.output(pinListLow, GPIO.LOW)

# all pins which should be HIGH at ini
GPIO.output(pinListHigh, GPIO.HIGH)


# define a clean node exit
def my_exit():
  print "Shutting LED server down..."
  # GPIO cleanup
  GPIO.cleanup()
  print "Done."

# call this method on node exit
rospy.on_shutdown(my_exit)


# handle_led is called with instances of LedRequest and returns instances of LedResponse
# The request name comes directly from the .srv filename
def handle_led(req):
    """ In this function all the work is done :) """

    # switch GPIO to HIGH, if '1' was sent
    if (req.state == 1):
      GPIO.output(req.pin, GPIO.HIGH)
    else:
      # for all other values we set it to LOW
      # (LEDs are low active!)
      GPIO.output(req.pin, GPIO.LOW)

    # debug
    print "GPIO %s switched to %s. Result: %s"%(req.pin, req.state, req.pin)

    # The name of the 'xyzResponse' comes directly from the Xyz.srv filename!
    return LedResponse(req.pin)


def led_server():
    # Service nodes have to be initialised
    rospy.init_node('led_server')

    # This declares a new service named 'led'' with the 'Led' service type.
    # All requests are passed to the 'handle_led' function.
    # 'handle_led' is called with instances of LedRequest and returns instances of LedResponse
    s = rospy.Service('led', Led, handle_led)
    print "Ready to switch LEDs."

    # Keep our code from exiting until this service node is shutdown
    rospy.spin()


if __name__ == "__main__":
    led_server()
