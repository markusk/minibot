#!/usr/bin/env python
# coding=utf-8

# name of the package(!).srv
from minibot.srv import *
import rospy

# for GPIO pin usage
try:
    import RPi.GPIO as GPIO
except RuntimeError:
    print("Error importing RPi.GPIO!")

##
## GPIO stuff
##
GPIO.setmode(GPIO.BCM) # use the GPIO names, _not_ the pin numbers on the board
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

# all LOW
# GPIO.output(pinListLow, GPIO.LOW)

# all HIGH
GPIO.output(pinListHigh, GPIO.HIGH)

# a list for saving the state of each of the 40 GPIO when changed via this node
# False means LOW
gpioStatus = [False for i in range(40)]


# define a clean node exit
def my_exit():
  print "Shutting LED server down..."
  # GPIO cleanup
  GPIO.cleanup()
  print "Done."

# call this method on mode exit
rospy.on_shutdown(my_exit)


# handle_add_two_ints is called with instances of BatteryRequest and returns instances of BatteryResponse
# The request name comes directly from the .srv filename
def handle_led(req):
    # here is all the work done :)

    # switch GPIO
    if (req.state == 1):
      GPIO.output(req.pin, GPIO.HIGH)
    else:
      # (LEDs are low active!)
      GPIO.output(req.pin, GPIO.LOW)

    # store state in list
    gpioStatus[req.pin-1] = False

    # debug
    print "GPIO %s switched to %s. Result: %s"%(req.pin, req.state, req.pin)

    # The name of the 'xyzResponse' comes directly from the Xyz.srv filename!
    return LedResponse(req.pin)


def led_server():
    rospy.init_node('led_server')

    # This declares a new service named battery with the Battery service type.
    s = rospy.Service('led', Led, handle_led)

    print "Ready to switch LEDs."
    rospy.spin()


if __name__ == "__main__":
    led_server()
