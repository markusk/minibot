#!/usr/bin/python
# coding=utf-8

from MCP3008 import MCP3008

adc = MCP3008()
voltage = 0

# read AD converter (battery voltage)
# use channel 0 on IC
value = adc.read(channel = 0)
# 2.73 V = 12.32 V (measured) > 1023 / 3.3 * 2.73 / 12.32 = 68.693182
voltage = (value / 68.693182)
#print("Value: %d" % value)
print("Battery: %.1f Volt" % voltage)
