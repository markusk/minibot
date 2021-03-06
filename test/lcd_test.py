#!/usr/bin/python
# coding=utf-8

"""
Playing around with the OLED.

Currently implemented:
- a graphical battery symbol with voltage and percent
"""

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

# min and max voltages
minVoltage = 3*3.3 # 3S LiPo-Battery with 3 x 3.3Volt =  9.9 Volt (empty battery)
maxVoltage = 3*4.2 # 3S LiPo-Battery with 3 x 4.2Volt = 12.6 Volt (full  battery)


# ------------
# test voltage
# ------------
measuredVoltage = minVoltage


# the battery symbols
batteryEmpty = unichr(0xf244) # <25% = minVoltage


# battery level (white rectangle in empty battery symbol
maxRectLength = 16


# endless loop
while (1):
	# --------------
	# TEST TEST TEST
	#
	measuredVoltage = measuredVoltage + 0.1
	if measuredVoltage > maxVoltage:
		measuredVoltage = minVoltage
	#
	# --------------


	# percent calculation
	convertedVoltage = measuredVoltage - minVoltage
	percent = convertedVoltage / (maxVoltage-minVoltage) * 100
	if percent < 0:
		percent = 0
	# rectangle in battery symbol
	rectLength = round(percent * maxRectLength / 100, 0)

	# Draw a black filled box to clear the image.
	draw.rectangle((0,0,width,height), outline=0, fill=0)

	# Write lines of text to display
	#
	# line 1, battery symbol

	# draw empty battery symbol
	draw.text((0, 0), batteryEmpty, font=fontSymbol, fill=255)
	# add filling level as filled rectangle

	# empty: draw.rectangle((1, 3,  1, 11), outline=255, fill=255)
	# full:  draw.rectangle((1, 3, 16, 11), outline=255, fill=255)
	draw.rectangle((1, 3, rectLength, 11), outline=255, fill=255)

	# line 1, text after symbol
	string = ("%.0f %%" % round(percent, 2))
	draw.text((symbolWidth, 0), string, font=fontText, fill=255)
	# line 2
	draw.text((0, size), str("%.2f Volt" % measuredVoltage), font=fontText, fill=255)

	# Display image.
	disp.image(image)
	disp.display()

	# wait a bit
	time.sleep(0.5)
