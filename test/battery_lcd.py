#!/usr/bin/python
# coding=utf-8

"""
This code reads the battery voltage via Adafruits ADS1015 AD converter and shows
the values on an OLED via SSD1306.

It also checks a pushbotton state, connected to #23 (pin 16 on Raspberry Pi 3)
via 10k pull-down resistor. If pushed, it calls the "shutdown now" command.
"""

# for time and sleep
import time

# AD converter stuff
import Adafruit_ADS1x15

from PIL import Image
from PIL import ImageDraw
from PIL import ImageFont

# Raspberry Pi pin configuration:
RST = 24

# 128x32 display with hardware I2C:
disp = Adafruit_SSD1306.SSD1306_128_32(rst=RST)

# Initialize library.
disp.begin()


# --------------------
# for signal handling
# --------------------
import signal
import sys

# my signal handler
def sig_handler(_signo, _stack_frame):
    # LED OFF (low active!)
    GPIO.output(ledPin, GPIO.HIGH)
    ## GPIO cleanup
    GPIO.remove_event_detect(switchPin)
    GPIO.cleanup()
    # clear display
    disp.clear()
    disp.display()
    print "battery_lcd terminated clean."
    sys.exit(0)

# signals to be handled
signal.signal(signal.SIGINT,  sig_handler)
signal.signal(signal.SIGHUP,  sig_handler)
signal.signal(signal.SIGTERM, sig_handler)


# ----------------------
# GPIO/pushbotton stuff
# ----------------------
import RPi.GPIO as GPIO
# for poweroff
from subprocess import call

# init
GPIO.setmode(GPIO.BCM) # use the GPIO names, _not_ the pin numbers on the board

# Raspberry Pi pin configuration:
# pins	    BCM   BOARD
switchPin  = 17 # pin 11
ledPin     = 18 # pin 12

# setup
print('setup...')
GPIO.setup(switchPin, GPIO.IN, pull_up_down=GPIO.PUD_UP) # waits for LOW
GPIO.setup(ledPin, GPIO.OUT)

# LED OFF (low active!)
GPIO.output(ledPin, GPIO.HIGH)

# checker
buttonPressed = False

# switch detection by interrupt, falling edge, with debouncing
def my_callback(answer):
    # LED ON (low active!)
    GPIO.output(ledPin, GPIO.LOW)

    print 'Shutdown button on GPIO ' + str(answer) + ' pushed.'

    # clear display
    disp.clear()
    disp.display()
    # show some shutdown text on OLED

    # shutdown computer!!
    print '++++++++++++++++++++++++++++++++++'
    print '+++ Shutting down in 5 seconds +++'
    print '++++++++++++++++++++++++++++++++++'
    # delay
    time.sleep(5)

    # power off
    call('sudo shutdown --poweroff "now"', shell=True)


# add button pressed event detector
print('registering event handler...')
GPIO.add_event_detect(switchPin, GPIO.FALLING, callback=my_callback, bouncetime=200)



# Clear display.
disp.clear()
disp.display()

# Create blank image for drawing.
# Make sure to create image with mode '1' for 1-bit color.
width = disp.width
height = disp.height
image = Image.new('1', (width, height))

# Get drawing object to draw on image.
draw = ImageDraw.Draw(image)

# Load default font.
size = 15
font = ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf', size)

# read voltage
from MCP3008 import MCP3008 # for battery reading

# the AD converter object
adc = Adafruit_ADS1x15.ADS1015()
GAIN = 1

while (buttonPressed == False):
    # Draw a black filled box to clear the image.
    draw.rectangle((0,0,width,height), outline=0, fill=0)

    voltage = 0

    # read AD converter (battery voltage)
    # use channel 0 on IC
    value = adc.read_adc(0, gain=GAIN)
    # GAIN = 1  =  2047 = 4.096 Volt  -> 1849 = 3,7 Volt?
    # 2.73 V = 12.32 V (measured) -> 1849 / 3.3 * 2.73 / 12.32 = 124,158057851239669
    voltage = (value / 124.158057851239669)
    # print("Value: %d" % value)
    # print("Battery: %.1f Volt" % voltage)

    # get time
    timeString = time.strftime("%H:%M:%S", time.localtime(time.time()) )

    # Write Text and Voltage
    draw.text((0, 0),    ("Time: %s" % timeString),  font=font, fill=255)

    # show measured voltage or -- when no battery is connected
    # (it won't be at 0 Volt hopefully)
    if (voltage > 0):
        draw.text((0, size), ("Battery: %.2fV" % voltage), font=font, fill=255)
    else:
        draw.text((0, size), ("Battery: --"), font=font, fill=255)

    # Display image.
    disp.image(image)
    disp.display()

    # check button here as well
    if GPIO.input(switchPin) == GPIO.LOW:
    	# LED ON (low active!)
    	GPIO.output(ledPin, GPIO.LOW)
    	print('Button pressed (in while loop)!')
        buttonPressed = True
	# call the piushbutton event handler
        my_callback()

    # wait 1 second
    time.sleep(1)


# wtf?
print('This line should never be reached')
