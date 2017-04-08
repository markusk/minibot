#!/usr/bin/env python
# coding=utf-8

# This is a service node (server)

# name of the package(!).srv
from minibot.srv import *
import rospy

# for getting the hostname of the underlying system
#import socket
# showing hostname
#hostname = socket.gethostname()
#print("Running on host " + hostname + ".")


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


###### motor stuff
from Adafruit_MotorHAT import Adafruit_MotorHAT, Adafruit_DCMotor

# run some parts only on the real robot
# if hostname == 'minibot':
# create a default motor object, no changes to I2C address or frequency
mh = Adafruit_MotorHAT(addr=0x60)

# using motor 1 and 2 on RasPi hat
myMotor1 = mh.getMotor(1)
myMotor2 = mh.getMotor(2)

# recommended for auto-disabling motors on shutdown!
def turnOffMotors():
    myMotor1.run(Adafruit_MotorHAT.RELEASE);
    myMotor2.run(Adafruit_MotorHAT.RELEASE);

# turning off motors NOW
turnOffMotors();

# set the speed (from 0 (off) to 255 (max speed))
startSpeed = 100
myMotor1.setSpeed(startSpeed)
myMotor2.setSpeed(startSpeed)


# define a clean node exit
def my_exit():
  rospy.loginfo('Shutting down motor service...')
  # run some parts only on the real robot
  # if hostname == 'minibot':
  turnOffMotors();
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
    rospy.loginfo('Ready to switch motors.')

    # Keep our code from exiting until this service node is shutdown
    rospy.spin()


if __name__ == "__main__":
    led_server()



""" motor stuff to be implemented here:
    ### drive
    # run some parts only on the real robot
    if hostname == 'minibot':
        # drive
        print("Forward! ")
        myMotor1.run(Adafruit_MotorHAT.FORWARD)
        myMotor2.run(Adafruit_MotorHAT.FORWARD)

        print("\tSpeed up...")
        for i in range(startSpeed, 255):
            myMotor1.setSpeed(i)
            myMotor2.setSpeed(i)
            time.sleep(0.01)

        print("\tSlow down...")
        for i in range(255, startSpeed, -1):
            myMotor1.setSpeed(i)
            myMotor2.setSpeed(i)
            time.sleep(0.01)

        print("Backward! ")
        myMotor1.run(Adafruit_MotorHAT.BACKWARD)
        myMotor2.run(Adafruit_MotorHAT.BACKWARD)

        print("\tSpeed up...")
        for i in range(startSpeed, 255):
            myMotor1.setSpeed(i)
            myMotor2.setSpeed(i)
            time.sleep(0.01)

        print("\tSlow down...")
        for i in range(255, startSpeed, -1):
            myMotor1.setSpeed(i)
            myMotor2.setSpeed(i)
            time.sleep(0.01)

        print("Release")
        myMotor1.run(Adafruit_MotorHAT.RELEASE)
        myMotor2.run(Adafruit_MotorHAT.RELEASE)
"""
