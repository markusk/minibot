#!/usr/bin/python
# coding=utf-8

import time

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

#
# read voltage
#
from MCP3008 import MCP3008 # for battery reading

adc = MCP3008()
voltage = 0

# read AD converter (battery voltage)
# use channel 0 on IC
value = adc.read(channel = 0)
# 2.73 V = 12.32 V (measured) > 1023 / 3.3 * 2.73 / 12.32 = 68.693182
voltage = (value / 68.693182)
# print("Value: %d" % value)
# print("Battery: %.1f Volt" % voltage)


#
# Write Text and Voltage
#
draw.text((0, 0),     'Domo Arigato...',  font=font, fill=255)
draw.text((0, size), ("Battery: %.2fV" % voltage), font=font, fill=255)

# Display image.
disp.image(image)
disp.display()