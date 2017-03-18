EESchema Schematic File Version 2
LIBS:power
LIBS:device
LIBS:transistors
LIBS:conn
LIBS:linear
LIBS:regul
LIBS:74xx
LIBS:cmos4000
LIBS:adc-dac
LIBS:memory
LIBS:xilinx
LIBS:microcontrollers
LIBS:dsp
LIBS:microchip
LIBS:analog_switches
LIBS:motorola
LIBS:texas
LIBS:intel
LIBS:audio
LIBS:interface
LIBS:digital-audio
LIBS:philips
LIBS:display
LIBS:cypress
LIBS:siliconi
LIBS:opto
LIBS:atmel
LIBS:contrib
LIBS:valves
LIBS:minibot-cache
EELAYER 25 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
Title ""
Date ""
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L R R1
U 1 1 58C474F4
P 2300 2950
F 0 "R1" V 2380 2950 50  0000 C CNN
F 1 "620" V 2300 2950 50  0000 C CNN
F 2 "" V 2230 2950 50  0000 C CNN
F 3 "" H 2300 2950 50  0000 C CNN
	1    2300 2950
	0    1    1    0   
$EndComp
$Comp
L ZENER D1
U 1 1 58C475AD
P 2700 3150
F 0 "D1" H 2700 3250 50  0000 C CNN
F 1 "3,3V" H 2700 3050 50  0000 C CNN
F 2 "" H 2700 3150 50  0000 C CNN
F 3 "" H 2700 3150 50  0000 C CNN
	1    2700 3150
	0    1    1    0   
$EndComp
Wire Wire Line
	2450 2950 2950 2950
$Comp
L +BATT #PWR1
U 1 1 58C47783
P 1950 2950
F 0 "#PWR1" H 1950 2800 50  0001 C CNN
F 1 "+BATT" H 1950 3090 50  0000 C CNN
F 2 "" H 1950 2950 50  0000 C CNN
F 3 "" H 1950 2950 50  0000 C CNN
	1    1950 2950
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR2
U 1 1 58C4779D
P 1950 3350
F 0 "#PWR2" H 1950 3100 50  0001 C CNN
F 1 "GND" H 1950 3200 50  0000 C CNN
F 2 "" H 1950 3350 50  0000 C CNN
F 3 "" H 1950 3350 50  0000 C CNN
	1    1950 3350
	1    0    0    -1  
$EndComp
Wire Wire Line
	1950 2950 2150 2950
Wire Wire Line
	1950 3350 2700 3350
$Comp
L MCP3008 U1
U 1 1 58C47870
P 3650 3250
F 0 "U1" H 3200 3700 50  0000 C CNN
F 1 "MCP3008" H 4000 3700 50  0000 C CNN
F 2 "" H 3550 3150 50  0000 C CNN
F 3 "" H 3650 3250 50  0000 C CNN
	1    3650 3250
	1    0    0    -1  
$EndComp
Connection ~ 2700 2950
Wire Wire Line
	2700 3950 3750 3950
Wire Wire Line
	2700 3350 2700 3950
Connection ~ 3550 3950
$Comp
L +3.3V #PWR3
U 1 1 58C4793B
P 3550 2300
F 0 "#PWR3" H 3550 2150 50  0001 C CNN
F 1 "+3.3V" H 3550 2440 50  0000 C CNN
F 2 "" H 3550 2300 50  0000 C CNN
F 3 "" H 3550 2300 50  0000 C CNN
	1    3550 2300
	1    0    0    -1  
$EndComp
Wire Wire Line
	3550 2650 3550 2300
Wire Wire Line
	3750 2650 3550 2650
Connection ~ 3550 2650
Connection ~ 2700 3350
$EndSCHEMATC
