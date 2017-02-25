#!/usr/bin/python

try:
    import RPi.GPIO as GPIO
except RuntimeError:
    print("Error importing RPi.GPIO!  This is probably because you need superuser privileges.  You can achieve this by using 'sudo' to run your script")

# import time # for sleep
import time # for sleep

# timings
secs = 2

# init
GPIO.setmode(GPIO.BCM) # use the GPIO names, _not_ the pin numbers on the board

# pins	    BCM   BOARD
ledPin     = 18 # pin 12
switchPin  = 23 # pin 16
batteryPin = 24 # pin 18

# setup GPIO outputs
print('setup...')
GPIO.setup(ledPin,   GPIO.OUT)
GPIO.setup(switchPin, GPIO.IN)
GPIO.setup(batteryPin, GPIO.IN)

#
# loop
#

# turn LED on
GPIO.output(ledPin, GPIO.LOW)

# delay x seconds
time.sleep(secs)

# turn LED off
GPIO.output(ledPin, GPIO.HIGH)

# cleanup
GPIO.cleanup()
