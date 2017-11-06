#!/usr/bin/python
# coding=utf-8

import time
import atexit


# ----------------------
# Encoder stuff
# ----------------------
import RPi.GPIO as GPIO

# init
GPIO.setmode(GPIO.BCM) # use the GPIO names, _not_ the pin numbers on the board

# Raspberry Pi pin configuration:
# pins	    BCM   BOARD
leftEncoderGPIO  = 27 # pin
rightEncoderGPIO = 22 # pin

# setup
print('setup...')
GPIO.setup(leftEncoderGPIO,  GPIO.IN)
GPIO.setup(rightEncoderGPIO, GPIO.IN)


# encoder pulse detection by interrupt
def leftEncoderCallback(answer):
    print 'Left Encoder.'

def rightEncoderCallback(answer):
    print 'Right Encoder.'


# add GPIO event detectors
print('registering event handlers...')
GPIO.add_event_detect(leftEncoderGPIO,  GPIO.FALLING, callback=leftEncoderCallback)
GPIO.add_event_detect(rightEncoderGPIO, GPIO.FALLING, callback=rightEncoderCallback)


# ----------------------
# Motor stuff
# ----------------------
from Adafruit_MotorHAT import Adafruit_MotorHAT, Adafruit_DCMotor

# create a default motor object, no changes to I2C address or frequency
mh = Adafruit_MotorHAT(addr=0x60)

# recommended for auto-disabling motors on shutdown!
def turnOffMotors():
    mh.getMotor(1).run(Adafruit_MotorHAT.RELEASE)
    mh.getMotor(2).run(Adafruit_MotorHAT.RELEASE)

# user motor 1 and 2 on RasPi hat
myMotor1 = mh.getMotor(1)
myMotor2 = mh.getMotor(2)

# turn off motors
myMotor1.run(Adafruit_MotorHAT.RELEASE);
myMotor2.run(Adafruit_MotorHAT.RELEASE);

# set the speed (from 0 (off) to 255 (max speed))
startSpeed = 100
maxSpeed   = 255 # max is 255!

# test switch
fullSpeedDuration = 3 # default 0

myMotor1.setSpeed(startSpeed)
myMotor2.setSpeed(startSpeed)


## -----------------------
## what to do on exit pgm
## -----------------------
def exitMinibot():
    turnOffMotors();
    ## GPIO cleanup
    GPIO.remove_event_detect(leftEncoderGPIO)
    GPIO.remove_event_detect(rightEncoderGPIO)
    GPIO.cleanup()

atexit.register(exitMinibot)


#
# Cowntdown
#
print("Starting in 3...")
time.sleep(1)
print("Starting in 2...")
time.sleep(1)
print("Starting in 1...")
time.sleep(1)
print("GO!\n")


######
###### forever - or until ctrl+c  :)
######
while (True):
    ### drive
    # drive
    """ print("Forward! ")

    myMotor1.run(Adafruit_MotorHAT.FORWARD)
    myMotor2.run(Adafruit_MotorHAT.FORWARD)

    print("\tSpeed up...")
    for i in range(startSpeed, maxSpeed):
        myMotor1.setSpeed(i)
        myMotor2.setSpeed(i)
        time.sleep(0.01)

    # full speed for n seconds
    print("+++ full speed for " + str(fullSpeedDuration) + " seconds +++")
    time.sleep(fullSpeedDuration)

    print("\tSlow down...")
    for i in range(maxSpeed, startSpeed, -1):
        myMotor1.setSpeed(i)
        myMotor2.setSpeed(i)
        time.sleep(0.01)

    # wait one second
    time.sleep(1)

    print("Backward! ")
    myMotor1.run(Adafruit_MotorHAT.BACKWARD)
    myMotor2.run(Adafruit_MotorHAT.BACKWARD)

    print("\tSpeed up...")
    for i in range(startSpeed, maxSpeed):
        myMotor1.setSpeed(i)
        myMotor2.setSpeed(i)
        time.sleep(0.01)

    print("\tSlow down...")
    for i in range(maxSpeed, startSpeed, -1):
        myMotor1.setSpeed(i)
        myMotor2.setSpeed(i)
        time.sleep(0.01)

    print("Release")
    myMotor1.run(Adafruit_MotorHAT.RELEASE)
    myMotor2.run(Adafruit_MotorHAT.RELEASE)
    """

    # wait some time
    time.sleep(0.25)
