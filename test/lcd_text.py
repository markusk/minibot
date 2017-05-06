#!/usr/bin/python
# coding=utf-8

import time

# for getting arguments
import sys

# print arguments
i = len(sys.argv)

if i == 2:
    print "%d. argument: %s" % (i,sys.argv[1])
elif i == 3:
    print "%d. argument: %s" % (i,sys.argv[2])
else:
    print("Usage: " + sys.argv[0] + " text1 text2")
    sys.exit(-1)

sys.exit(0)


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

# Load default font.
# font = ImageFont.load_default()
# Alternatively load a TTF font.  Make sure the .ttf font file is in the same directory as the python script!
# Some other nice fonts to try: http://www.dafont.com/bitmap.php
# font = ImageFont.truetype('Roboto-Regular.ttf', 12)
size = 15
font = ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf', size)

# Write two lines of text.
draw.text((0, 0),    'Domo Arigato :)',  font=font, fill=255)
draw.text((0, size), 'Battery: 99.9 V', font=font, fill=255)

# Display image.
disp.image(image)
disp.display()
