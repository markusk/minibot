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
P 4800 3400
F 0 "R1" V 4800 3400 50  0000 C CNN
F 1 "620" V 4800 3400 50  0000 C CNN
F 2 "" V 4730 3400 50  0000 C CNN
F 3 "" H 4800 3400 50  0000 C CNN
	1    4800 3400
	-1   0    0    1   
$EndComp
$Comp
L ZENER D1
U 1 1 58C475AD
P 4800 3900
F 0 "D1" H 4800 4000 50  0000 C CNN
F 1 "3,3V" H 4800 3800 50  0000 C CNN
F 2 "" H 4800 3900 50  0000 C CNN
F 3 "" H 4800 3900 50  0000 C CNN
	1    4800 3900
	0    1    1    0   
$EndComp
$Comp
L +BATT #PWR2
U 1 1 58C47783
P 4800 1900
F 0 "#PWR2" H 4800 1750 50  0001 C CNN
F 1 "+BATT" H 4800 2040 50  0000 C CNN
F 2 "" H 4800 1900 50  0000 C CNN
F 3 "" H 4800 1900 50  0000 C CNN
	1    4800 1900
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR5
U 1 1 58C4779D
P 4800 4850
F 0 "#PWR5" H 4800 4600 50  0001 C CNN
F 1 "GND" H 4800 4700 50  0000 C CNN
F 2 "" H 4800 4850 50  0000 C CNN
F 3 "" H 4800 4850 50  0000 C CNN
	1    4800 4850
	1    0    0    -1  
$EndComp
$Comp
L MCP3008 U1
U 1 1 58C47870
P 5750 4000
F 0 "U1" H 5300 4450 50  0000 C CNN
F 1 "MCP3008" H 6100 4450 50  0000 C CNN
F 2 "" H 5650 3900 50  0000 C CNN
F 3 "" H 5750 4000 50  0000 C CNN
	1    5750 4000
	1    0    0    -1  
$EndComp
$Comp
L +3.3V #PWR4
U 1 1 58C4793B
P 5650 1900
F 0 "#PWR4" H 5650 1750 50  0001 C CNN
F 1 "+3.3V" H 5650 2040 50  0000 C CNN
F 2 "" H 5650 1900 50  0000 C CNN
F 3 "" H 5650 1900 50  0000 C CNN
	1    5650 1900
	1    0    0    -1  
$EndComp
$Comp
L CONN_02X20_RaspberryPi_3 P1
U 1 1 58E25E94
P 1350 4150
F 0 "P1" H 1350 5200 50  0000 C CNN
F 1 "Raspberry_Pi" V 1350 4150 50  0000 C CNN
F 2 "" H 1350 3200 50  0000 C CNN
F 3 "" H 1350 3200 50  0000 C CNN
	1    1350 4150
	1    0    0    -1  
$EndComp
$Comp
L OLED_128x64_0.96inch P2
U 1 1 58E26202
P 1350 5950
F 0 "P2" H 1350 6200 50  0000 C CNN
F 1 "OLED" V 1450 5950 50  0000 C CNN
F 2 "" H 1350 5950 50  0000 C CNN
F 3 "" H 1350 5950 50  0000 C CNN
	1    1350 5950
	-1   0    0    -1  
$EndComp
$Comp
L +5V #PWR3
U 1 1 58E26794
P 2300 1900
F 0 "#PWR3" H 2300 1750 50  0001 C CNN
F 1 "+5V" H 2300 2040 50  0000 C CNN
F 2 "" H 2300 1900 50  0000 C CNN
F 3 "" H 2300 1900 50  0000 C CNN
	1    2300 1900
	1    0    0    -1  
$EndComp
$Comp
L LED D2
U 1 1 58E271A7
P 2850 3700
F 0 "D2" H 2850 3800 50  0000 C CNN
F 1 "LED" H 2850 3600 50  0000 C CNN
F 2 "" H 2850 3700 50  0000 C CNN
F 3 "" H 2850 3700 50  0000 C CNN
	1    2850 3700
	1    0    0    -1  
$EndComp
$Comp
L R R2
U 1 1 58E2740A
P 3350 3700
F 0 "R2" V 3430 3700 50  0000 C CNN
F 1 "370" V 3350 3700 50  0000 C CNN
F 2 "" V 3280 3700 50  0000 C CNN
F 3 "" H 3350 3700 50  0000 C CNN
	1    3350 3700
	0    1    1    0   
$EndComp
Wire Wire Line
	4800 3550 4800 3700
Wire Wire Line
	4800 3700 5050 3700
Connection ~ 4800 3700
Wire Wire Line
	4800 4750 5650 4750
Wire Wire Line
	5650 4750 5850 4750
Wire Wire Line
	4800 4100 4800 4750
Wire Wire Line
	4800 4750 4800 4850
Connection ~ 5650 3400
Connection ~ 4800 4100
Wire Wire Line
	2300 4800 2300 5450
Wire Wire Line
	2300 5450 2300 5800
Wire Wire Line
	2300 5800 2300 6200
Wire Wire Line
	1600 4800 2300 4800
Wire Wire Line
	1600 3700 2650 3700
Wire Wire Line
	3050 3700 3200 3700
Wire Wire Line
	1100 3200 1100 3100
Wire Wire Line
	1100 3100 1100 1900
Wire Wire Line
	1600 3200 1700 3200
Wire Wire Line
	1700 3200 2300 3200
Wire Wire Line
	2300 3200 2300 1900
$Comp
L SW_PUSH SW1
U 1 1 58E2783A
P 2750 3000
F 0 "SW1" H 2900 3110 50  0000 C CNN
F 1 "Shutdown" H 2750 2920 50  0000 C CNN
F 2 "" H 2750 3000 50  0000 C CNN
F 3 "" H 2750 3000 50  0000 C CNN
	1    2750 3000
	1    0    0    -1  
$EndComp
$Comp
L R R3
U 1 1 58E2790A
P 3350 3000
F 0 "R3" V 3430 3000 50  0000 C CNN
F 1 "10k" V 3350 3000 50  0000 C CNN
F 2 "" V 3280 3000 50  0000 C CNN
F 3 "" H 3350 3000 50  0000 C CNN
	1    3350 3000
	0    1    1    0   
$EndComp
Wire Wire Line
	3050 3000 3200 3000
Wire Wire Line
	2300 5800 1550 5800
Wire Wire Line
	1800 5900 1550 5900
Wire Wire Line
	1100 3400 600  3400
Wire Wire Line
	600  3400 600  5400
Wire Wire Line
	600  5400 1900 5400
Wire Wire Line
	1900 5400 1900 6000
Wire Wire Line
	1900 6000 1550 6000
Wire Wire Line
	1100 3300 700  3300
Wire Wire Line
	700  3300 700  5300
Wire Wire Line
	700  5300 2000 5300
Wire Wire Line
	2000 5300 2000 6100
Wire Wire Line
	2000 6100 1550 6100
Wire Wire Line
	1100 4100 950  4100
Wire Wire Line
	950  4100 950  5200
Wire Wire Line
	950  5200 6700 5200
Wire Wire Line
	6700 5200 6700 4100
Wire Wire Line
	6700 4100 6450 4100
Wire Wire Line
	1100 4200 1000 4200
Wire Wire Line
	1000 4200 1000 5650
Wire Wire Line
	1000 5650 6900 5650
Wire Wire Line
	6900 5650 6900 4000
Wire Wire Line
	6900 4000 6450 4000
Wire Wire Line
	1100 4300 1050 4300
Wire Wire Line
	1050 4300 1050 5550
Wire Wire Line
	1050 5550 6800 5550
Wire Wire Line
	6800 5550 6800 3900
Wire Wire Line
	6800 3900 6450 3900
Wire Wire Line
	1600 4300 1900 4300
Wire Wire Line
	1900 4300 1900 5100
Wire Wire Line
	1900 5100 6600 5100
Wire Wire Line
	6600 5100 6600 4200
Wire Wire Line
	6600 4200 6450 4200
$Comp
L SFH9202 P3
U 1 1 59EF9434
P 8400 1600
F 0 "P3" H 8400 1850 50  0000 C CNN
F 1 "SFH9202" V 8400 1600 50  0000 C CNN
F 2 "" H 8300 800 50  0000 C CNN
F 3 "" H 8300 800 50  0000 C CNN
	1    8400 1600
	1    0    0    -1  
$EndComp
$Comp
L R R4
U 1 1 59EF9864
P 9250 1250
F 0 "R4" V 9330 1250 50  0000 C CNN
F 1 "680" V 9250 1250 50  0000 C CNN
F 2 "" V 9180 1250 50  0000 C CNN
F 3 "" H 9250 1250 50  0000 C CNN
	1    9250 1250
	1    0    0    -1  
$EndComp
$Comp
L R R5
U 1 1 59EF98E3
P 9250 1650
F 0 "R5" V 9330 1650 50  0000 C CNN
F 1 "10k" V 9250 1650 50  0000 C CNN
F 2 "" V 9180 1650 50  0000 C CNN
F 3 "" H 9250 1650 50  0000 C CNN
	1    9250 1650
	1    0    0    -1  
$EndComp
$Comp
L R R6
U 1 1 59EF994E
P 9600 1250
F 0 "R6" V 9680 1250 50  0000 C CNN
F 1 "1M" V 9600 1250 50  0000 C CNN
F 2 "" V 9530 1250 50  0000 C CNN
F 3 "" H 9600 1250 50  0000 C CNN
	1    9600 1250
	1    0    0    -1  
$EndComp
$Comp
L C C1
U 1 1 59EF9A11
P 9250 2050
F 0 "C1" H 9275 2150 50  0000 L CNN
F 1 "100n" H 9275 1950 50  0000 L CNN
F 2 "" H 9288 1900 50  0000 C CNN
F 3 "" H 9250 2050 50  0000 C CNN
	1    9250 2050
	1    0    0    -1  
$EndComp
$Comp
L C C2
U 1 1 59EF9AFE
P 10400 2050
F 0 "C2" H 10425 2150 50  0000 L CNN
F 1 "100n" H 10425 1950 50  0000 L CNN
F 2 "" H 10438 1900 50  0000 C CNN
F 3 "" H 10400 2050 50  0000 C CNN
	1    10400 2050
	1    0    0    -1  
$EndComp
$Comp
L POT 200k
U 1 1 59EF9BBB
P 8900 1100
F 0 "200k" H 8900 1020 50  0000 C CNN
F 1 "POT" H 8900 1100 50  0000 C CNN
F 2 "" H 8900 1100 50  0000 C CNN
F 3 "" H 8900 1100 50  0000 C CNN
	1    8900 1100
	1    0    0    -1  
$EndComp
$Comp
L +3.3V #PWR6
U 1 1 59EF9CF1
P 8000 900
F 0 "#PWR6" H 8000 750 50  0001 C CNN
F 1 "+3.3V" H 8000 1040 50  0000 C CNN
F 2 "" H 8000 900 50  0000 C CNN
F 3 "" H 8000 900 50  0000 C CNN
	1    8000 900 
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR7
U 1 1 59EF9DE5
P 8650 2400
F 0 "#PWR7" H 8650 2150 50  0001 C CNN
F 1 "GND" H 8650 2250 50  0000 C CNN
F 2 "" H 8650 2400 50  0000 C CNN
F 3 "" H 8650 2400 50  0000 C CNN
	1    8650 2400
	1    0    0    -1  
$EndComp
$Comp
L MCP6001 U2
U 1 1 59EF9ED9
P 9950 1550
F 0 "U2" H 10000 1750 50  0000 C CNN
F 1 "MCP6001" H 10150 1350 50  0000 C CNN
F 2 "" H 9900 1650 50  0000 C CNN
F 3 "" H 10000 1750 50  0000 C CNN
	1    9950 1550
	1    0    0    -1  
$EndComp
$Comp
L R R7
U 1 1 59EFA98B
P 8000 1300
F 0 "R7" V 8080 1300 50  0000 C CNN
F 1 "680" V 8000 1300 50  0000 C CNN
F 2 "" V 7930 1300 50  0000 C CNN
F 3 "" H 8000 1300 50  0000 C CNN
	1    8000 1300
	1    0    0    -1  
$EndComp
Wire Wire Line
	8000 1450 8150 1450
Wire Wire Line
	8000 900  8000 950 
Wire Wire Line
	8000 950  8000 1150
Wire Wire Line
	8000 2250 8650 2250
Wire Wire Line
	8650 2250 9250 2250
Wire Wire Line
	9250 2250 9850 2250
Wire Wire Line
	9850 2250 10400 2250
Wire Wire Line
	9250 1350 9250 1450
Wire Wire Line
	9250 1450 9250 1500
Wire Wire Line
	8650 1450 9250 1450
Wire Wire Line
	9250 1450 9600 1450
Wire Wire Line
	9600 1450 9650 1450
Connection ~ 9250 1450
Wire Wire Line
	9250 1100 9050 1100
Wire Wire Line
	8000 950  8900 950 
Wire Wire Line
	8900 950  9850 950 
Wire Wire Line
	9850 950  10400 950 
Connection ~ 8000 950 
Wire Wire Line
	9250 1800 9250 1850
Wire Wire Line
	9250 1850 9250 1900
Wire Wire Line
	8150 1750 8000 1750
Wire Wire Line
	8000 1750 8000 2250
Wire Wire Line
	9250 2250 9250 2200
Wire Wire Line
	9600 1450 9600 1400
Wire Wire Line
	9600 1100 10250 1100
Wire Wire Line
	10250 1100 10250 1550
Connection ~ 9600 1450
Wire Wire Line
	9650 1650 9600 1650
Wire Wire Line
	9600 1650 9600 1850
Wire Wire Line
	9600 1850 9250 1850
Connection ~ 9250 1850
Wire Wire Line
	9850 2250 9850 1850
Connection ~ 9250 2250
Wire Wire Line
	8650 1750 8650 2250
Wire Wire Line
	8650 2250 8650 2400
Connection ~ 8650 2250
Wire Wire Line
	10400 2250 10400 2200
Connection ~ 9850 2250
Wire Wire Line
	9850 950  9850 1250
Connection ~ 8900 950 
Wire Wire Line
	10400 950  10400 1900
Connection ~ 9850 950 
Wire Wire Line
	10250 1550 10800 1550
$Comp
L GND #PWR?
U 1 1 59EFD850
P 2300 6200
F 0 "#PWR?" H 2300 5950 50  0001 C CNN
F 1 "GND" H 2300 6050 50  0000 C CNN
F 2 "" H 2300 6200 50  0000 C CNN
F 3 "" H 2300 6200 50  0000 C CNN
	1    2300 6200
	1    0    0    -1  
$EndComp
Connection ~ 2300 5800
$Comp
L +3.3V #PWR?
U 1 1 59EFDCBF
P 1800 1900
F 0 "#PWR?" H 1800 1750 50  0001 C CNN
F 1 "+3.3V" H 1800 2040 50  0000 C CNN
F 2 "" H 1800 1900 50  0000 C CNN
F 3 "" H 1800 1900 50  0000 C CNN
	1    1800 1900
	1    0    0    -1  
$EndComp
Wire Wire Line
	5650 1900 5650 3400
$Comp
L +3.3V #PWR?
U 1 1 59EFF75B
P 3900 1900
F 0 "#PWR?" H 3900 1750 50  0001 C CNN
F 1 "+3.3V" H 3900 2040 50  0000 C CNN
F 2 "" H 3900 1900 50  0000 C CNN
F 3 "" H 3900 1900 50  0000 C CNN
	1    3900 1900
	1    0    0    -1  
$EndComp
Wire Wire Line
	3900 1900 3900 3700
Wire Wire Line
	3900 3700 3500 3700
Wire Wire Line
	5650 3400 5850 3400
Wire Wire Line
	4800 1900 4800 3250
Connection ~ 4800 4750
Wire Wire Line
	5650 4750 5650 4700
Wire Wire Line
	5850 4750 5850 4700
Connection ~ 5650 4750
$Comp
L SFH9202 P?
U 1 1 59F014B7
P 8400 4050
F 0 "P?" H 8400 4300 50  0000 C CNN
F 1 "SFH9202" V 8400 4050 50  0000 C CNN
F 2 "" H 8300 3250 50  0000 C CNN
F 3 "" H 8300 3250 50  0000 C CNN
	1    8400 4050
	1    0    0    -1  
$EndComp
$Comp
L R R?
U 1 1 59F014BD
P 9250 3700
F 0 "R?" V 9330 3700 50  0000 C CNN
F 1 "680" V 9250 3700 50  0000 C CNN
F 2 "" V 9180 3700 50  0000 C CNN
F 3 "" H 9250 3700 50  0000 C CNN
	1    9250 3700
	1    0    0    -1  
$EndComp
$Comp
L R R?
U 1 1 59F014C3
P 9250 4100
F 0 "R?" V 9330 4100 50  0000 C CNN
F 1 "10k" V 9250 4100 50  0000 C CNN
F 2 "" V 9180 4100 50  0000 C CNN
F 3 "" H 9250 4100 50  0000 C CNN
	1    9250 4100
	1    0    0    -1  
$EndComp
$Comp
L R R?
U 1 1 59F014C9
P 9600 3700
F 0 "R?" V 9680 3700 50  0000 C CNN
F 1 "1M" V 9600 3700 50  0000 C CNN
F 2 "" V 9530 3700 50  0000 C CNN
F 3 "" H 9600 3700 50  0000 C CNN
	1    9600 3700
	1    0    0    -1  
$EndComp
$Comp
L C C?
U 1 1 59F014CF
P 9250 4500
F 0 "C?" H 9275 4600 50  0000 L CNN
F 1 "100n" H 9275 4400 50  0000 L CNN
F 2 "" H 9288 4350 50  0000 C CNN
F 3 "" H 9250 4500 50  0000 C CNN
	1    9250 4500
	1    0    0    -1  
$EndComp
$Comp
L C C?
U 1 1 59F014D5
P 10400 4500
F 0 "C?" H 10425 4600 50  0000 L CNN
F 1 "100n" H 10425 4400 50  0000 L CNN
F 2 "" H 10438 4350 50  0000 C CNN
F 3 "" H 10400 4500 50  0000 C CNN
	1    10400 4500
	1    0    0    -1  
$EndComp
$Comp
L POT 200k?
U 1 1 59F014DB
P 8900 3550
F 0 "200k?" H 8900 3470 50  0000 C CNN
F 1 "POT" H 8900 3550 50  0000 C CNN
F 2 "" H 8900 3550 50  0000 C CNN
F 3 "" H 8900 3550 50  0000 C CNN
	1    8900 3550
	1    0    0    -1  
$EndComp
$Comp
L +3.3V #PWR?
U 1 1 59F014E1
P 8000 3350
F 0 "#PWR?" H 8000 3200 50  0001 C CNN
F 1 "+3.3V" H 8000 3490 50  0000 C CNN
F 2 "" H 8000 3350 50  0000 C CNN
F 3 "" H 8000 3350 50  0000 C CNN
	1    8000 3350
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR?
U 1 1 59F014E7
P 8650 4850
F 0 "#PWR?" H 8650 4600 50  0001 C CNN
F 1 "GND" H 8650 4700 50  0000 C CNN
F 2 "" H 8650 4850 50  0000 C CNN
F 3 "" H 8650 4850 50  0000 C CNN
	1    8650 4850
	1    0    0    -1  
$EndComp
$Comp
L MCP6001 U?
U 1 1 59F014ED
P 9950 4000
F 0 "U?" H 10000 4200 50  0000 C CNN
F 1 "MCP6001" H 10150 3800 50  0000 C CNN
F 2 "" H 9900 4100 50  0000 C CNN
F 3 "" H 10000 4200 50  0000 C CNN
	1    9950 4000
	1    0    0    -1  
$EndComp
$Comp
L R R?
U 1 1 59F014F3
P 8000 3750
F 0 "R?" V 8080 3750 50  0000 C CNN
F 1 "680" V 8000 3750 50  0000 C CNN
F 2 "" V 7930 3750 50  0000 C CNN
F 3 "" H 8000 3750 50  0000 C CNN
	1    8000 3750
	1    0    0    -1  
$EndComp
Wire Wire Line
	8000 3900 8150 3900
Wire Wire Line
	8000 3350 8000 3400
Wire Wire Line
	8000 3400 8000 3600
Wire Wire Line
	8000 4700 8650 4700
Wire Wire Line
	8650 4700 9250 4700
Wire Wire Line
	9250 4700 9850 4700
Wire Wire Line
	9850 4700 10400 4700
Wire Wire Line
	9250 3800 9250 3900
Wire Wire Line
	9250 3900 9250 3950
Wire Wire Line
	8650 3900 9250 3900
Wire Wire Line
	9250 3900 9600 3900
Wire Wire Line
	9600 3900 9650 3900
Connection ~ 9250 3900
Wire Wire Line
	9250 3550 9050 3550
Wire Wire Line
	8000 3400 8900 3400
Wire Wire Line
	8900 3400 9850 3400
Wire Wire Line
	9850 3400 10400 3400
Connection ~ 8000 3400
Wire Wire Line
	9250 4250 9250 4300
Wire Wire Line
	9250 4300 9250 4350
Wire Wire Line
	8150 4200 8000 4200
Wire Wire Line
	8000 4200 8000 4700
Wire Wire Line
	9250 4700 9250 4650
Wire Wire Line
	9600 3900 9600 3850
Wire Wire Line
	9600 3550 10250 3550
Wire Wire Line
	10250 3550 10250 4000
Connection ~ 9600 3900
Wire Wire Line
	9650 4100 9600 4100
Wire Wire Line
	9600 4100 9600 4300
Wire Wire Line
	9600 4300 9250 4300
Connection ~ 9250 4300
Wire Wire Line
	9850 4700 9850 4300
Connection ~ 9250 4700
Wire Wire Line
	8650 4200 8650 4700
Wire Wire Line
	8650 4700 8650 4850
Connection ~ 8650 4700
Wire Wire Line
	10400 4700 10400 4650
Connection ~ 9850 4700
Wire Wire Line
	9850 3400 9850 3700
Connection ~ 8900 3400
Wire Wire Line
	10400 3400 10400 4350
Connection ~ 9850 3400
Wire Wire Line
	10250 4000 10800 4000
Text Notes 8650 850  0    120  ~ 0
Encoder links
Text Notes 8650 3350 0    120  ~ 0
Encoder rechts
$Comp
L GND #PWR?
U 1 1 59F02393
P 3600 3100
F 0 "#PWR?" H 3600 2850 50  0001 C CNN
F 1 "GND" H 3600 2950 50  0000 C CNN
F 2 "" H 3600 3100 50  0000 C CNN
F 3 "" H 3600 3100 50  0000 C CNN
	1    3600 3100
	1    0    0    -1  
$EndComp
Wire Wire Line
	3500 3000 3600 3000
Wire Wire Line
	3600 3000 3600 3100
Wire Wire Line
	1100 3700 750  3700
Wire Wire Line
	750  3700 750  3000
Wire Wire Line
	750  3000 2450 3000
Wire Wire Line
	10800 1550 10800 2700
Wire Wire Line
	10800 2700 1000 2700
Wire Wire Line
	1000 2700 1000 3800
Wire Wire Line
	1000 3800 1100 3800
Wire Wire Line
	1100 3900 950  3900
Wire Wire Line
	950  3900 950  2750
Wire Wire Line
	950  2750 10800 2750
Wire Wire Line
	10800 2750 10800 4000
$Comp
L +3.3V #PWR?
U 1 1 59F03941
P 1100 1900
F 0 "#PWR?" H 1100 1750 50  0001 C CNN
F 1 "+3.3V" H 1100 2040 50  0000 C CNN
F 2 "" H 1100 1900 50  0000 C CNN
F 3 "" H 1100 1900 50  0000 C CNN
	1    1100 1900
	1    0    0    -1  
$EndComp
Wire Wire Line
	1800 2900 1800 5900
Wire Wire Line
	1800 1900 1800 2900
$EndSCHEMATC
