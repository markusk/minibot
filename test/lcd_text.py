#!/usr/bin/python
# coding=utf-8

# import time

# for getting arguments
import sys

# check arguments
if len(sys.argv) != 3:
#    print "1. argument: " + sys.argv[1]
#    print "2. argument: " + sys.argv[2]
    print "ERROR. Correct Usage is: " + sys.argv[0] + " text1 text2"
    sys.exit(-1)

# check arg length
length = 15
if len(sys.argv[1]) > length:
    print "ERROR: Text 1 exceeds length of %d." % length
    if len(sys.argv[2]) > length:
        print "ERROR: Text 2 exceeds length of %d." % length
    sys.exit(-1)

if len(sys.argv[2]) > length:
    if len(sys.argv[1]) > length:
        print "ERROR: Text 1 exceeds length of %d." % length
    print "ERROR: Text 2 exceeds length of %d." % length
    sys.exit(-1)


# LCD stuff
import Adafruit_GPIO.SPI as SPI
import Adafruit_SSD1306

from PIL import Image
from PIL import ImageDraw
from PIL import ImageFont


# Raspberry Pi pin configuration:
RST = 24

# 128x32 display with hardware I2C:
disp = Adafruit_SSD1306.SSD1306_128_32(rst=RST)

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

# Load TTF font.
size = 15
font = ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf', size)

# Send text to LCD (arguments from command line)
draw.text((0, 0),    sys.argv[1],  font=font, fill=255)
draw.text((0, size), sys.argv[2], font=font, fill=255)

# Display image.
disp.image(image)
disp.display()
