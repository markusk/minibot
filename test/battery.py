#!/usr/bin/python
# coding=utf-8

import Adafruit_ADS1x15

# the Analog Digital converter object
adc = Adafruit_ADS1x15.ADS1015()

""" Gain 1 means, max a value of +4.096 Volt (+4,096 Volt in Europe) on the ADC channel, resulting in a 'value' of +2047. """
GAIN = 1
voltage = 0

# read AD converter (battery voltage)
# use channel 0 on IC
value = adc.read_adc(0, gain=GAIN)

# 12.32 Volt battery voltage resulted in 2.66 Volt on the ADC channel with my circuit (Z-Diode and Resistor).
# This resulted in a ADV 'value' of 1323.
# The conversion factor for the battery voltage is then: 1323 / 12.32 = 107.3863636
#
voltage = (value / 107.3863636)
print("Value: %d" % value)
print("Battery: %.1f Volt" % voltage)
