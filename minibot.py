#!/usr/bin/python

###### AD converter stuff
from MCP3008 import MCP3008
adc = MCP3008()
voltage = 0

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
# Raspberry Pi pin configuration:
RST = 24

# create a 128x32 display with hardware I2C
disp = Adafruit_SSD1306.SSD1306_128_32(rst=RST)


# create a default motor object, no changes to I2C address or frequency
mh = Adafruit_MotorHAT(addr=0x60)

# recommended for auto-disabling motors on shutdown!
def turnOffMotors():
    mh.getMotor(1).run(Adafruit_MotorHAT.RELEASE)
    mh.getMotor(2).run(Adafruit_MotorHAT.RELEASE)

# what to do at program exit
atexit.register(turnOffMotors)

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


################## loop

# LCD
# Initialize library.
disp.begin()

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

# Draw a black filled box to clear the image.
draw.rectangle((0,0,width,height), outline=0, fill=0)

# Load default font.
# font = ImageFont.load_default()

# load my favorite TTF font
font = ImageFont.truetype('Roboto-Regular.ttf', 12)

# Write two lines of text.
draw.text((0, 0),  'Domo Arigato Roboto!',  font=font, fill=255)
draw.text((0, 20), 'Battery: 11.1V', font=font, fill=255)

# Display image.
disp.image(image)
disp.display()


###### forever - or until ctrl+c  :)
while (True):
    # read AD converter (battery voltage)
    # use channel 0 on IC
    voltage = adc.read(channel = 0)
    print("Voltage: %.2f" % (value / 1023.0 * 3.3))

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
    time.sleep(1.0)
