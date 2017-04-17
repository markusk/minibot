#!/usr/bin/python
# coding=utf-8

import os

def getCpuTemperature():
	tempFile = open( "/sys/class/thermal/thermal_zone0/temp" )
	cpu_temp = tempFile.read()
	tempFile.close()
	return float(cpu_temp)/1000

print("CPU Temperature: " + str(getCpuTemperature()) + "°C." )
