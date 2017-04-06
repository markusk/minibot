#!/usr/bin/python
# coding=utf-8


# for GPIO pin usage
try:
    import RPi.GPIO as GPIO
except RuntimeError:
    print("Error importing RPi.GPIO!")

# what to do when exiting this pgm
import atexit
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


##
## what to do on exit pgm
##
def exitMinibot():
  # GPIO cleanup
  GPIO.cleanup()


##
## what to do at program exit
##
atexit.register(exitMinibot)


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
