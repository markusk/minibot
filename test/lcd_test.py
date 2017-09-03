#!/usr/bin/python
# coding=utf-8

import time
import Adafruit_GPIO.SPI as SPI
import Adafruit_SSD1306
from PIL import Image, ImageDraw, ImageFont

# Raspberry Pi pin configuration:
RST = 24

# 128x32 display with hardware I2C:
disp = Adafruit_SSD1306.SSD1306_128_32(rst=RST)

# Initialize library.
disp.begin()

# Clear display.
disp.clear()
disp.display()

# Create blank image for drawing. Make sure to create image with mode '1' for 1-bit color.
width = disp.width
height = disp.height
image = Image.new('1', (width, height))

# The fonts and sizes
size = 15
symbolWidth = 28
# text
fontText = ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf', size)
# https://fontawesome.io
# install via sudo 'sudo apt install fonts-font-awesome'
fontSymbol = ImageFont.truetype('/usr/share/fonts/truetype/font-awesome/fontawesome-webfont.ttf', size)

# see http://fontawesome.io/cheatsheet/
# battery-full = '' + unichr(0xf240)
# battery-full = unicode(u'0xf240')

# Get drawing object to draw on image.
draw = ImageDraw.Draw(image)

# Draw a black filled box to clear the image.
draw.rectangle((0,0,width,height), outline=0, fill=0)

# voltage in percent
percent = 100

# the battery symbols
batteryFull          = unichr(0xf240)
batteryThreeQuarters = unichr(0xf241)
batteryHalf          = unichr(0xf242)
batteryQuarter       = unichr(0xf243)
batteryEmpty         = unichr(0xf244)

# Write lines of text.
#
# line 1, battery symbol
draw.text((0, 0), batteryFull, font=fontSymbol, fill=255)
# line 1, text after symbol
draw.text((symbolWidth, 0), str(percent) + ' %', font=fontText, fill=255)
# line 2
draw.text((0, size), '12.05 Volt', font=fontText, fill=255)

# Display image.
disp.image(image)
disp.display()
