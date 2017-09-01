#!/usr/bin/python
# coding=utf-8
import Adafruit_ADS1x15

adc = Adafruit_ADS1x15.ADS1015()
GAIN = 1
voltage = 0

# read AD converter (battery voltage)
# use channel 0 on IC
value = adc.read_adc(0, gain=GAIN)

# old: 2.73 V = 12.32 V (measured) > 1023 / 3.3 * 2.73 / 12.32 = 68.693182
#
# GAIN = 1  =  2047 = 4.096 Volt  -> 1849 = 3,7 Volt?
# 2.73 V = 12.32 V (measured) -> 1849 / 3.3 * 2.73 / 12.32 = 124,158057851239669
voltage = (value / 124.158057851239669)
#print("Value: %d" % value)
print("Battery: %.1f Volt" % voltage)
