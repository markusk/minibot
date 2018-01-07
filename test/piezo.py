#!/usr/bin/python
# coding=utf-8

"""
Testing a piezo buzzer.
"""


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
ledPin    = 18 # pin 12

# GPIO setup
print('GPIO setup...')
GPIO.setup(ledPin, GPIO.OUT)


# for signal handling
import signal
import sys

# my signal handler
def sig_handler(_signo, _stack_frame):
    # clear display
    # GPIO cleanup
    GPIO.cleanup()
    print "led_blinky terminated clean."
    sys.exit(0)

# signals to be handled
signal.signal(signal.SIGINT,  sig_handler)
signal.signal(signal.SIGHUP,  sig_handler)
signal.signal(signal.SIGTERM, sig_handler)


######
###### forever - or until ctrl+c  :)
######
print('Blinking...')
while (True):
    # LED ON (low active!)
    GPIO.output(ledPin, GPIO.LOW)

    # wait 1 second
    time.sleep(1)

    # LED OFF
    GPIO.output(ledPin, GPIO.HIGH)

    # wait 1 second
    time.sleep(1)
