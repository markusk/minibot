#!/usr/bin/python
# coding=utf-8

"""
Sends n beeps to a piezo buzzer.
Usage for 5 beeps: beep.py 5
"""

# wait time in seconds
waitTime = 0.3

# for getting arguments
import sys

# check arguments
if len(sys.argv) != 2:
#    print "1. argument: " + sys.argv[1]
#    print "2. argument: " + sys.argv[2]
    print "ERROR. Correct usage for three beeps is: " + sys.argv[0] + " 3"
    sys.exit(-1)


# for GPIO pin usage
try:
    import RPi.GPIO as GPIO
except RuntimeError:
    print("Error importing RPi.GPIO!")

# for sleep
import time

##
## GPIO stuff
##
GPIO.setmode(GPIO.BCM) # use the GPIO names, _not_ the pin numbers on the board
# Raspberry Pi pin configuration:
# pins	    BCM   BOARD
piezoPin  = 25 # pin

# GPIO setup
GPIO.setup(piezoPin, GPIO.OUT)


# for signal handling
import signal
import sys

# my signal handler
def sig_handler(_signo, _stack_frame):
    # GPIO cleanup
    GPIO.cleanup()
    print "piezo terminated clean."
    sys.exit(0)

# signals to be handled
signal.signal(signal.SIGINT,  sig_handler)
signal.signal(signal.SIGHUP,  sig_handler)
signal.signal(signal.SIGTERM, sig_handler)


######
###### Beeping
######
for x in range(0, int(sys.argv[1])):
    # Piezo OFF
    GPIO.output(piezoPin, GPIO.HIGH)
    # wait
    time.sleep(waitTime)

    # Piezo ON (low active!)
    GPIO.output(piezoPin, GPIO.LOW)
    # "wait" (generate a square wave for the piezo)
    time.sleep(waitTime)
