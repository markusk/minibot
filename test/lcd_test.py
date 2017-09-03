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
symbolWidth = 25
# text
font1 = ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf', size)
# https://fontawesome.io
# install via sudo 'sudo apt install fonts-font-awesome'
font2 = ImageFont.truetype('/usr/share/fonts/truetype/font-awesome/fontawesome-webfont.ttf', size)

# see http://fontawesome.io/cheatsheet/
# battery-full = '' + unichr(0xf240)
# battery-full = unicode(u'0xf240')

# Get drawing object to draw on image.
draw = ImageDraw.Draw(image)

# Draw a black filled box to clear the image.
draw.rectangle((0,0,width,height), outline=0, fill=0)

# Write lines of text.
# line 1
draw.text((0, 0),              'Battery-Status:', font=font1, fill=255)
# line 2, first symbol
draw.text((0, size),           unichr(0xf240),    font=font2, fill=255)
# line 2, text after symbol
draw.text((symbolWidth, size), '12.0 Volt',       font=font1, fill=255)

# Display image.
disp.image(image)
disp.display()
