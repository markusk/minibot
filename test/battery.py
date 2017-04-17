#!/usr/bin/python
# coding=utf-8

from MCP3008 import MCP3008

adc = MCP3008()
value = adc.read( channel = 0 ) # use channel 0 for battery voltage
print("Voltage: %.2f" % (value / 1023.0 * 3.3) )
