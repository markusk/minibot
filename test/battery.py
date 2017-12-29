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

# 13.44 Volt battery voltage resulted in 2,94 Volt on the ADC channel with my circuit (voltage divider w/ two resistors (39k + 11k)).
# This resulted in a ADC 'value' of 1465.
# The conversion factor for the battery voltage is then: 1465 / 13.44 = 109.00297619047619
#
voltage = (value / 109.00297619047619)
print("Value: %d" % value)
print("Battery: %.1f Volt" % voltage)
