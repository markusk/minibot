#!/usr/bin/python
# coding=utf-8

import time
import atexit


###### motor stuff
from Adafruit_MotorHAT import Adafruit_MotorHAT, Adafruit_DCMotor


###### LCD stuff
import Adafruit_GPIO.SPI as SPI
import Adafruit_SSD1306

from PIL import Image
from PIL import ImageDraw
from PIL import ImageFont


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
myMotor1.setSpeed(startSpeed)
myMotor2.setSpeed(startSpeed)


##
## what to do on exit pgm
##
def exitMinibot():
    turnOffMotors();


##
## what to do at program exit
##
atexit.register(exitMinibot)


######
###### forever - or until ctrl+c  :)
######
while (True):
    ### drive
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
