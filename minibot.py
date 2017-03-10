#!/usr/bin/python
# coding=utf-8

# for getting the hostname of the underlying system
import socket

# for GPIO pin usage
try:
    import RPi.GPIO as GPIO
except RuntimeError:
    print("Error importing RPi.GPIO!")

##
## GPIO stuff
##
GPIO.setmode(GPIO.BCM) # use the GPIO names, _not_ the pin numbers on the board
# Raspberry Pi pin configuration:
# pins	    BCM   BOARD
RST       = 24
ledPin    = 18 # pin 12
switchPin = 23 # pin 16

# GPIO setup
print('GPIO setup...')
GPIO.setup(ledPin,    GPIO.OUT)
GPIO.setup(switchPin, GPIO.IN, pull_up_down=GPIO.PUD_UP) # waits for LOW

# switch detection by interrupt, falling edge, with debouncing
def my_callback(answer):
    print 'Button on GPIO ' + str(answer) + ' pushed!'
    # LED ON (test)
    GPIO.output(ledPin, GPIO.LOW)


# add event for pressed button  detection
GPIO.add_event_detect(switchPin, GPIO.FALLING, callback=my_callback, bouncetime=200)


###### AD converter stuff
from MCP3008 import MCP3008
adc = MCP3008()
voltage = 0
value = 0


###### motor stuff
from Adafruit_MotorHAT import Adafruit_MotorHAT, Adafruit_DCMotor
import time
import atexit


###### LCD stuff
import Adafruit_GPIO.SPI as SPI
import Adafruit_SSD1306

from PIL import Image
from PIL import ImageDraw
from PIL import ImageFont


# create a 128x32 display with hardware I2C
disp = Adafruit_SSD1306.SSD1306_128_32(rst=RST)

# showing hostname
hostname = socket.gethostname()
print("Running on host " + hostname + ".")


# run some parts only on the real robot
if hostname == 'minibot':
    # create a default motor object, no changes to I2C address or frequency
    mh = Adafruit_MotorHAT(addr=0x60)

    # recommended for auto-disabling motors on shutdown!
    def turnOffMotors():
        mh.getMotor(1).run(Adafruit_MotorHAT.RELEASE)
        mh.getMotor(2).run(Adafruit_MotorHAT.RELEASE)

    # user motor 1 and 2 on RasPi hat
    myMotor1 = mh.getMotor(1)
    myMotor2 = mh.getMotor(2)

    # turn off motors
    myMotor1.run(Adafruit_MotorHAT.RELEASE);
    myMotor2.run(Adafruit_MotorHAT.RELEASE);

    # set the speed (from 0 (off) to 255 (max speed))
    startSpeed = 100
    myMotor1.setSpeed(startSpeed)
    myMotor2.setSpeed(startSpeed)


##
## what to do on exit pgm
##
def exitMinibot():
  # run some parts only on the real robot
  if hostname == 'minibot':
    turnOffMotors();
  # Clear display.
  disp.clear()
  disp.display()
  # GPIO cleanup
  GPIO.remove_event_detect(switchPin)
  GPIO.cleanup()


##
## what to do at program exit
##
atexit.register(exitMinibot)


################## LCD
# Initialize library.
disp.begin()

# Clear display.
disp.clear()
disp.display()

# Create blank image for drawing.
# Make sure to create image with mode '1' for 1-bit color.
width = disp.width    # 128
height = disp.height  #  64
image = Image.new('1', (width, height))

# Get drawing object to draw on image.
draw = ImageDraw.Draw(image)

# Draw a black filled box to clear the image.
draw.rectangle((0,0,width,height), outline=0, fill=0)

# load my favorite TTF font
fontText   = ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf', 20)
fontSymbol = ImageFont.truetype('Digital Camera Symbols.ttf', 36)

# Display image.
disp.image(image)
disp.display()


######
###### Threads
######
def readVoltage():
  # read AD converter (battery voltage)
  # use channel 0 on IC
  value = adc.read(channel = 0)
  print("Value: %d" % value)
  # 3.19 V = 12.35 V (measured)
  print("Voltage: %.2f" % (value * 12.35 / 1024))
  # displaySensorwertAusgabe()


######
###### forever - or until ctrl+c  :)
######
while (True):
    # read voltage from AD converter
    readVoltage()

    # clear LCD
    draw.rectangle((0,0, width, height), outline=0, fill=0)
    # LCD battery symbol
    draw.text(( 0, 0), "d", font=fontSymbol, fill=255)
    # draw.text((55, 4), "11.0 V", font=fontText, fill=255)
    # LCD voltage
    draw.text((55, 4), str("%.2f V" % (voltage / 1023.0 * 3.3)), font=fontText, fill=255)
    # go
    disp.image(image)
    disp.display()

    """ test test test "empty battery" (clear part of battery symbol)
    time.sleep(1)
    rectw = 18
    recth = 22

    draw.rectangle((8, 10, rectw, recth), outline=0, fill=0)
    time.sleep(1)
    disp.image(image)
    disp.display()

    draw.rectangle((8+1*rectw, 10, rectw, recth), outline=0, fill=0)
    time.sleep(1)
    disp.image(image)
    disp.display()

    draw.rectangle((8+2*rectw, 10, rectw, recth), outline=0, fill=0)
    time.sleep(1)
    disp.image(image)
    disp.display()"""


    ### drive
    # run some parts only on the real robot
    if hostname == 'minibot':
        # drive
        print("Forward! ")
        myMotor1.run(Adafruit_MotorHAT.FORWARD)
        myMotor2.run(Adafruit_MotorHAT.FORWARD)

        print("\tSpeed up...")
        for i in range(startSpeed, 255):
            myMotor1.setSpeed(i)
            myMotor2.setSpeed(i)
            time.sleep(0.01)

        print("\tSlow down...")
        for i in range(255, startSpeed, -1):
            myMotor1.setSpeed(i)
            myMotor2.setSpeed(i)
            time.sleep(0.01)

        print("Backward! ")
        myMotor1.run(Adafruit_MotorHAT.BACKWARD)
        myMotor2.run(Adafruit_MotorHAT.BACKWARD)

        print("\tSpeed up...")
        for i in range(startSpeed, 255):
            myMotor1.setSpeed(i)
            myMotor2.setSpeed(i)
            time.sleep(0.01)

        print("\tSlow down...")
        for i in range(255, startSpeed, -1):
            myMotor1.setSpeed(i)
            myMotor2.setSpeed(i)
            time.sleep(0.01)

        print("Release")
        myMotor1.run(Adafruit_MotorHAT.RELEASE)
        myMotor2.run(Adafruit_MotorHAT.RELEASE)

    # sleep
    #time.sleep(1.0)
