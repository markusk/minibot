#!/usr/bin/env python
# coding=utf-8

""" This is a service node (server) to control motors an the minibot
(https://minibot.me). It controls the motors via I2C on a Raspberry Pi.
I expects the messsages "FORWARD, BACKWARD, LEFT, RIGHT, STOP".

Author:  Markus Knapp, 2018
Website: https://direcs.de
"""


# speed of the motors (0-255).
drivingSpeed = 0
# the speed when turning the bot can be higher if needed (higher friction)
turnSpeed = 0

# name of the package(!).srv
from minibot.srv import *
import rospy

# Service nodes have to be initialised
rospy.init_node('motor_server', anonymous=False)


# for getting the hostname of the underlying system
import socket
# showing hostname
hostname = socket.gethostname()
if hostname == 'minibot':
    rospy.loginfo("Running on host %s.", hostname)
else:
    rospy.logwarn("Running on host %s!", hostname)


###############################
###### motor stuff
###############################
# run some parts only on the real robot
if hostname == 'minibot':
    rospy.loginfo("Setting up I2C...")

    from Adafruit_MotorHAT import Adafruit_MotorHAT, Adafruit_DCMotor

    # create a default motor object, no changes to I2C address or frequency
    mh = Adafruit_MotorHAT(addr=0x60)

    # using motor 1 and 2 on RasPi hat
    myMotor1 = mh.getMotor(1)
    myMotor2 = mh.getMotor(2)
    myMotor3 = mh.getMotor(3)
    myMotor4 = mh.getMotor(4)

    # recommended for auto-disabling motors on shutdown!
    def turnOffMotors():
        myMotor1.run(Adafruit_MotorHAT.RELEASE);
        myMotor2.run(Adafruit_MotorHAT.RELEASE);
        myMotor3.run(Adafruit_MotorHAT.RELEASE);
        myMotor4.run(Adafruit_MotorHAT.RELEASE);

    # turning off motors NOW - you never know...
    turnOffMotors();

else:
    rospy.logwarn("Skipping I2C setup. This is not the robot.")


# define a clean node exit
def my_exit():
  rospy.loginfo("Shutting down motor service...")
  # run some parts only on the real robot
  if hostname == 'minibot':
      turnOffMotors();
  rospy.loginfo("...shutting down motor service complete.")

# call this method on node exit
rospy.on_shutdown(my_exit)


# handle_motor is called with instances of MotorRequest and returns instances of MotorResponse
# The request name comes directly from the .srv filename
def handle_motor(req):
    """ In this function all the work is done :) """

    # set speed for motors from arguments
    drivingSpeed = req.speed
    turnSpeed = req.speed

    # switch xxx to HIGH, if '1' was sent
    if (req.direction == "FORWARD"): # and speed. returns result.
        # drive
        rospy.loginfo("Driving forward.")
        if hostname == 'minibot':
            # setting speed
            myMotor1.setSpeed(drivingSpeed)
            myMotor2.setSpeed(drivingSpeed)
            myMotor3.setSpeed(drivingSpeed)
            myMotor4.setSpeed(drivingSpeed)
            # @todo: increase speed?
            myMotor1.run(Adafruit_MotorHAT.FORWARD)
            myMotor2.run(Adafruit_MotorHAT.BACKWARD)
            myMotor3.run(Adafruit_MotorHAT.BACKWARD)
            myMotor4.run(Adafruit_MotorHAT.FORWARD)
    elif (req.direction == "BACKWARD"):
        rospy.loginfo("Driving backwards.")
        if hostname == 'minibot':
            # setting speed
            myMotor1.setSpeed(drivingSpeed)
            myMotor2.setSpeed(drivingSpeed)
            myMotor3.setSpeed(drivingSpeed)
            myMotor4.setSpeed(drivingSpeed)
            # @todo: increase speed?
            myMotor1.run(Adafruit_MotorHAT.BACKWARD)
            myMotor2.run(Adafruit_MotorHAT.FORWARD)
            myMotor3.run(Adafruit_MotorHAT.FORWARD)
            myMotor4.run(Adafruit_MotorHAT.BACKWARD)
    elif (req.direction == "LEFT"):
        rospy.loginfo("Driving left.")
        if hostname == 'minibot':
            # seeting speed
            # adding more power when turning
            myMotor1.setSpeed(turnSpeed)
            myMotor2.setSpeed(turnSpeed)
            myMotor3.setSpeed(turnSpeed)
            myMotor4.setSpeed(turnSpeed)
            # @todo: increase speed?
            myMotor1.run(Adafruit_MotorHAT.BACKWARD)
            myMotor2.run(Adafruit_MotorHAT.FORWARD)
            myMotor3.run(Adafruit_MotorHAT.BACKWARD)
            myMotor4.run(Adafruit_MotorHAT.FORWARD)
    elif (req.direction == "RIGHT"):
        rospy.loginfo("Driving right.")
        if hostname == 'minibot':
            # seeting speed
            # adding more power when turning
            myMotor1.setSpeed(turnSpeed)
            myMotor2.setSpeed(turnSpeed)
            myMotor3.setSpeed(turnSpeed)
            myMotor4.setSpeed(turnSpeed)
            # @todo: increase speed?
            myMotor1.run(Adafruit_MotorHAT.FORWARD)
            myMotor2.run(Adafruit_MotorHAT.BACKWARD)
            myMotor3.run(Adafruit_MotorHAT.FORWARD)
            myMotor4.run(Adafruit_MotorHAT.BACKWARD)
    elif (req.direction == "STOP"):
        rospy.loginfo("Stopping.")
        if hostname == 'minibot':
            myMotor1.run(Adafruit_MotorHAT.RELEASE)
            myMotor2.run(Adafruit_MotorHAT.RELEASE)
            myMotor3.run(Adafruit_MotorHAT.RELEASE)
            myMotor4.run(Adafruit_MotorHAT.RELEASE)
        # @todo: also slow down?
    else:
      rospy.logerr("Direction '%s' not implemented.", req.direction)

    # The name of the 'xyzResponse' comes directly from the Xyz.srv filename!
    # We return the speed as "okay"
    return MotorResponse(req.speed)


def motor_server():
    # This declares a new service named 'motor with the Motor service type.
    # All requests are passed to the 'handle_motor' function.
    # 'handle_motor' is called with instances of MotorRequest and returns instances of MotorResponse
    s = rospy.Service('motor', Motor, handle_motor)
    rospy.loginfo("Ready to switch motors.")

    # Keep our code from exiting until this service node is shutdown
    rospy.spin()


if __name__ == "__main__":
    motor_server()



""" motor spped increase to be implemented...

        rospy.loginfo("Speed up...")
        for i in range(drivingSpeed, 255):
            myMotor1.setSpeed(i)
            myMotor2.setSpeed(i)
            myMotor3.setSpeed(i)
            myMotor4.setSpeed(i)
            time.sleep(0.01)

        rospy.loginfo("Slow down...")
        for i in range(255, drivingSpeed, -1):
            myMotor1.setSpeed(i)
            myMotor2.setSpeed(i)
            myMotor3.setSpeed(i)
            myMotor4.setSpeed(i)
            time.sleep(0.01)
"""
