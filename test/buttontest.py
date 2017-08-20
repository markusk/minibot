#!/usr/bin/python

try:
    import RPi.GPIO as GPIO
except RuntimeError:
    print("Error importing RPi.GPIO!  This is probably because you need superuser privileges.  You can achieve this by using 'sudo' to run your script")

# import time # for sleep
import time # for sleep

# for poweroff
from subprocess import call


#------
# init
#------
GPIO.setmode(GPIO.BCM) # use the GPIO names, _not_ the pin numbers on the board

# pins	    BCM   BOARD
ledPin     = 18 # pin 12
switchPin  = 23 # pin 16

# buttonCounter
buttonPressed = 0


# setup
print('setup...')
GPIO.setup(ledPin,   GPIO.OUT)
GPIO.setup(switchPin, GPIO.IN, pull_up_down=GPIO.PUD_UP) # waits for LOW

# test
GPIO.setup(4, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(17, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(27, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(22, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(5, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(6, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(13, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(26, GPIO.IN, pull_up_down=GPIO.PUD_UP)
#23 oben
GPIO.setup(24, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(25, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(8, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(7, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(12, GPIO.IN, pull_up_down=GPIO.PUD_UP)


# switch detection by interrupt, falling edge, with debouncing
def my_callback(answer):
    global buttonPressed
    buttonPressed += 1
    print 'Button on GPIO ' + str(answer) + ' pushed the ' + str(buttonPressed) + ' time.'

"""    # shutdown computer!!
    if buttonPressed == 5:
	print '++++++++++++++++++++++++++++++++++'
	print '+++ Shutting down in 5 seconds +++'
	print '++++++++++++++++++++++++++++++++++'
	# delay
	time.sleep(5)

	# power off
	call('sudo shutdown --poweroff "now"', shell=True)
"""

# add button pressed event detector
GPIO.add_event_detect(switchPin, GPIO.FALLING, callback=my_callback, bouncetime=200)
#test
GPIO.add_event_detect(4, GPIO.FALLING, callback=my_callback, bouncetime=200)
GPIO.add_event_detect(17, GPIO.FALLING, callback=my_callback, bouncetime=200)
GPIO.add_event_detect(27, GPIO.FALLING, callback=my_callback, bouncetime=200)
GPIO.add_event_detect(22, GPIO.FALLING, callback=my_callback, bouncetime=200)
GPIO.add_event_detect(5, GPIO.FALLING, callback=my_callback, bouncetime=200)
GPIO.add_event_detect(6, GPIO.FALLING, callback=my_callback, bouncetime=200)
GPIO.add_event_detect(13, GPIO.FALLING, callback=my_callback, bouncetime=200)
GPIO.add_event_detect(26, GPIO.FALLING, callback=my_callback, bouncetime=200)
GPIO.add_event_detect(24, GPIO.FALLING, callback=my_callback, bouncetime=200)
GPIO.add_event_detect(25, GPIO.FALLING, callback=my_callback, bouncetime=200)
GPIO.add_event_detect(8, GPIO.FALLING, callback=my_callback, bouncetime=200)
GPIO.add_event_detect(7, GPIO.FALLING, callback=my_callback, bouncetime=200)
GPIO.add_event_detect(12, GPIO.FALLING, callback=my_callback, bouncetime=200)


# timing
secs = 5


#------
# loop
#------
print('Press button now (5 secs)...!')

# turn LED on
GPIO.output(ledPin, GPIO.LOW)

# delay
time.sleep(secs)

# turn LED off
GPIO.output(ledPin, GPIO.HIGH)


# cleanup
GPIO.remove_event_detect(switchPin)
GPIO.cleanup()
