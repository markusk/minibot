#!/usr/bin/env python
# coding=utf-8

# This is a service node (server)

# name of the package(!).srv
from minibot.srv import *
import rospy

# for getting the hostname of the underlying system
import socket
# showing hostname
hostname = socket.gethostname()
rospy.loginfo("Running on host %s.", hostname)


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
else:
    rospy.loginfo("Skipping I2C setup. This is not the robot.")


# define a clean node exit
def my_exit():
  rospy.loginfo("Shutting down motor service...")
  # run some parts only on the real robot
  if hostname == 'minibot':
      turnOffMotors();
  rospy.loginfo("Done.")

# call this method on node exit
rospy.on_shutdown(my_exit)


# handle_motor is called with instances of MotorRequest and returns instances of MotorResponse
# The request name comes directly from the .srv filename
def handle_motor(req):
    """ In this function all the work is done :) """

    # switch xxx to HIGH, if '1' was sent
    if (req.state == 1):
      # do something
    else:
      # for all other values we set it to LOW
      # do something

    # debug
    rospy.loginfo("xxx %s switched to %s. Result: %s", req.pin, req.state, req.pin)

    # The name of the 'xyzResponse' comes directly from the Xyz.srv filename!
    # we return the speed as "okay"
    return MotorResponse(req.speed)


def motor_server():
    # Service nodes have to be initialised
    rospy.init_node('motor_server')

    # This declares a new service named 'motor with the Motor service type.
    # All requests are passed to the 'handle_motor' function.
    # 'handle_motor' is called with instances of MotorRequest and returns instances of MotorResponse
    s = rospy.Service('motor', Motor, handle_motor)
    rospy.loginfo("Ready to switch motors.")

    # Keep our code from exiting until this service node is shutdown
    rospy.spin()


if __name__ == "__main__":
    motor_server()



""" motor stuff to be implemented here:
    ### drive
    # run some parts only on the real robot
    if hostname == 'minibot':
        # drive
        rospy.loginfo("Forward! ")
        myMotor1.run(Adafruit_MotorHAT.FORWARD)
        myMotor2.run(Adafruit_MotorHAT.FORWARD)

        rospy.loginfo("Speed up...")
        for i in range(startSpeed, 255):
            myMotor1.setSpeed(i)
            myMotor2.setSpeed(i)
            time.sleep(0.01)

        rospy.loginfo("Slow down...")
        for i in range(255, startSpeed, -1):
            myMotor1.setSpeed(i)
            myMotor2.setSpeed(i)
            time.sleep(0.01)

        rospy.loginfo("Backward! ")
        myMotor1.run(Adafruit_MotorHAT.BACKWARD)
        myMotor2.run(Adafruit_MotorHAT.BACKWARD)

        rospy.loginfo("Speed up...")
        for i in range(startSpeed, 255):
            myMotor1.setSpeed(i)
            myMotor2.setSpeed(i)
            time.sleep(0.01)

        rospy.loginfo("Slow down...")
        for i in range(255, startSpeed, -1):
            myMotor1.setSpeed(i)
            myMotor2.setSpeed(i)
            time.sleep(0.01)

        rospy.loginfo("Release")
        myMotor1.run(Adafruit_MotorHAT.RELEASE)
        myMotor2.run(Adafruit_MotorHAT.RELEASE)
"""
