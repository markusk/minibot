#!/usr/bin/python

try:
    import RPi.GPIO as GPIO
except RuntimeError:
    print("Error importing RPi.GPIO!  This is probably because you need superuser privileges.  You can achieve this by using 'sudo' to run your script")

# import time # for sleep
import time # for sleep

# for poweroff
# see: https://stackoverflow.com/questions/23013274/shutting-down-computer-linux-using-python
import dbus


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


# switch detection by interrupt, falling edge, with debouncing
def my_callback(answer):
    global buttonPressed
    buttonPressed += 1
    print 'Button on GPIO ' + str(answer) + ' pushed the ' + str(buttonPressed) + ' time.'

    # shutdown computer!!
    if buttonPressed == 3:
	print '++++++++++++++++++++++++++++++++++'
	print '+++ Shutting down in 3 seconds +++'
	print '++++++++++++++++++++++++++++++++++'
	# delay
	time.sleep(3)

	sys_bus = dbus.SystemBus()
	ck_srv = sys_bus.get_object('org.freedesktop.ConsoleKit', '/org/freedesktop/ConsoleKit/Manager')
	ck_iface = dbus.Interface(ck_srv, 'org.freedesktop.ConsoleKit.Manager')
	stop_method = ck_iface.get_dbus_method("Stop")
	stop_method()


# add button pressed event detector
GPIO.add_event_detect(switchPin, GPIO.FALLING, callback=my_callback, bouncetime=200)


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
