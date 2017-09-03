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

# Get drawing object to draw on image.
draw = ImageDraw.Draw(image)

# Draw a black filled box to clear the image.
draw.rectangle((0,0,width,height), outline=0, fill=0)

# voltage in percent
percent = 100
# min and max voltages
minVoltage = 3*3.3 # 3S LiPo-Battery with 3 x 3.3Volt =  9.9 Volt (empty battery)
maxVoltage = 3*4.2 # 3S LiPo-Battery with 3 x 4.2Volt = 12.6 Volt (full  battery)

#
# test voltages
#
currentVoltage = 12.6

# the battery symbols
batteryFull          = unichr(0xf240) # >75% = maxVoltage
batteryThreeQuarters = unichr(0xf241) #  75%
batteryHalf          = unichr(0xf242) #  50%
batteryQuarter       = unichr(0xf243) #  25%
batteryEmpty         = unichr(0xf244) # <25% = minVoltage

# Write lines of text to display
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
