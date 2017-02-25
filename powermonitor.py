#!/usr/bin/python

try:
    import RPi.GPIO as GPIO
except RuntimeError:
    print("Error importing RPi.GPIO!  This is probably because you need superuser privileges.  You can achieve this by using 'sudo' to run your script")

# import time # for sleep
import time # for sleep

#------
# init
#------
GPIO.setmode(GPIO.BCM) # use the GPIO names, _not_ the pin numbers on the board

# pins	    BCM   BOARD
ledPin     = 18 # pin 12
switchPin  = 23 # pin 16
batteryPin = 24 # pin 18


# setup
print('setup...')
GPIO.setup(ledPin,   GPIO.OUT)
GPIO.setup(switchPin, GPIO.IN, pull_up_down=GPIO.PUD_UP) # waits for LOW
GPIO.setup(batteryPin, GPIO.IN)


# switch detection by interrupt, falling edge, with debouncing
def my_callback(answer):
    print 'Button on GPIO ' + str(answer) + ' pushed!'

GPIO.add_event_detect(switchPin, GPIO.FALLING, callback=my_callback, bouncetime=200)


# timing
secs = 4


#------
# loop
#------
print('loop...')

# turn LED on
GPIO.output(ledPin, GPIO.LOW)

# delay
time.sleep(secs)

# turn LED off
GPIO.output(ledPin, GPIO.HIGH)


# cleanup
GPIO.cleanup()
