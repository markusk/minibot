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
P 2300 6700
F 0 "R1" V 2400 6700 50  0000 C CNN
F 1 "39,2k" V 2300 6700 50  0000 C CNN
F 2 "" V 2230 6700 50  0000 C CNN
F 3 "" H 2300 6700 50  0000 C CNN
	1    2300 6700
	-1   0    0    1   
$EndComp
$Comp
L +BATT #PWR8
U 1 1 58C47783
P 2300 6500
F 0 "#PWR8" H 2300 6350 50  0001 C CNN
F 1 "+BATT" H 2300 6640 50  0000 C CNN
F 2 "" H 2300 6500 50  0000 C CNN
F 3 "" H 2300 6500 50  0000 C CNN
	1    2300 6500
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR9
U 1 1 58C4779D
P 2300 7600
F 0 "#PWR9" H 2300 7350 50  0001 C CNN
F 1 "GND" H 2300 7450 50  0000 C CNN
F 2 "" H 2300 7600 50  0000 C CNN
F 3 "" H 2300 7600 50  0000 C CNN
	1    2300 7600
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
P 1300 5850
F 0 "P2" H 1300 6100 50  0000 C CNN
F 1 "OLED" V 1400 5850 50  0000 C CNN
F 2 "" H 1300 5850 50  0000 C CNN
F 3 "" H 1300 5850 50  0000 C CNN
	1    1300 5850
	-1   0    0    -1  
$EndComp
$Comp
L +5V #PWR7
U 1 1 58E26794
P 2300 1900
F 0 "#PWR7" H 2300 1750 50  0001 C CNN
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
L POT 200k1
U 1 1 59EF9BBB
P 8900 1100
F 0 "200k1" H 8900 1020 50  0000 C CNN
F 1 "POT" H 8900 1100 50  0000 C CNN
F 2 "" H 8900 1100 50  0000 C CNN
F 3 "" H 8900 1100 50  0000 C CNN
	1    8900 1100
	1    0    0    -1  
$EndComp
$Comp
L +3.3V #PWR12
U 1 1 59EF9CF1
P 8000 900
F 0 "#PWR12" H 8000 750 50  0001 C CNN
F 1 "+3.3V" H 8000 1040 50  0000 C CNN
F 2 "" H 8000 900 50  0000 C CNN
F 3 "" H 8000 900 50  0000 C CNN
	1    8000 900 
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR14
U 1 1 59EF9DE5
P 8650 2400
F 0 "#PWR14" H 8650 2150 50  0001 C CNN
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
$Comp
L GND #PWR5
U 1 1 59EFD850
P 1700 6050
F 0 "#PWR5" H 1700 5800 50  0001 C CNN
F 1 "GND" H 1700 5900 50  0000 C CNN
F 2 "" H 1700 6050 50  0000 C CNN
F 3 "" H 1700 6050 50  0000 C CNN
	1    1700 6050
	1    0    0    -1  
$EndComp
$Comp
L +3.3V #PWR3
U 1 1 59EFDCBF
P 1600 5600
F 0 "#PWR3" H 1600 5450 50  0001 C CNN
F 1 "+3.3V" H 1600 5740 50  0000 C CNN
F 2 "" H 1600 5600 50  0000 C CNN
F 3 "" H 1600 5600 50  0000 C CNN
	1    1600 5600
	1    0    0    -1  
$EndComp
$Comp
L +3.3V #PWR11
U 1 1 59EFF75B
P 3900 1900
F 0 "#PWR11" H 3900 1750 50  0001 C CNN
F 1 "+3.3V" H 3900 2040 50  0000 C CNN
F 2 "" H 3900 1900 50  0000 C CNN
F 3 "" H 3900 1900 50  0000 C CNN
	1    3900 1900
	1    0    0    -1  
$EndComp
$Comp
L SFH9202 P5
U 1 1 59F014B7
P 8400 4050
F 0 "P5" H 8400 4300 50  0000 C CNN
F 1 "SFH9202" V 8400 4050 50  0000 C CNN
F 2 "" H 8300 3250 50  0000 C CNN
F 3 "" H 8300 3250 50  0000 C CNN
	1    8400 4050
	1    0    0    -1  
$EndComp
$Comp
L R R10
U 1 1 59F014BD
P 9250 3700
F 0 "R10" V 9330 3700 50  0000 C CNN
F 1 "680" V 9250 3700 50  0000 C CNN
F 2 "" V 9180 3700 50  0000 C CNN
F 3 "" H 9250 3700 50  0000 C CNN
	1    9250 3700
	1    0    0    -1  
$EndComp
$Comp
L R R11
U 1 1 59F014C3
P 9250 4100
F 0 "R11" V 9330 4100 50  0000 C CNN
F 1 "10k" V 9250 4100 50  0000 C CNN
F 2 "" V 9180 4100 50  0000 C CNN
F 3 "" H 9250 4100 50  0000 C CNN
	1    9250 4100
	1    0    0    -1  
$EndComp
$Comp
L R R12
U 1 1 59F014C9
P 9600 3700
F 0 "R12" V 9680 3700 50  0000 C CNN
F 1 "1M" V 9600 3700 50  0000 C CNN
F 2 "" V 9530 3700 50  0000 C CNN
F 3 "" H 9600 3700 50  0000 C CNN
	1    9600 3700
	1    0    0    -1  
$EndComp
$Comp
L C C3
U 1 1 59F014CF
P 9250 4500
F 0 "C3" H 9275 4600 50  0000 L CNN
F 1 "100n" H 9275 4400 50  0000 L CNN
F 2 "" H 9288 4350 50  0000 C CNN
F 3 "" H 9250 4500 50  0000 C CNN
	1    9250 4500
	1    0    0    -1  
$EndComp
$Comp
L C C4
U 1 1 59F014D5
P 10400 4500
F 0 "C4" H 10425 4600 50  0000 L CNN
F 1 "100n" H 10425 4400 50  0000 L CNN
F 2 "" H 10438 4350 50  0000 C CNN
F 3 "" H 10400 4500 50  0000 C CNN
	1    10400 4500
	1    0    0    -1  
$EndComp
$Comp
L POT 200k2
U 1 1 59F014DB
P 8900 3550
F 0 "200k2" H 8900 3470 50  0000 C CNN
F 1 "POT" H 8900 3550 50  0000 C CNN
F 2 "" H 8900 3550 50  0000 C CNN
F 3 "" H 8900 3550 50  0000 C CNN
	1    8900 3550
	1    0    0    -1  
$EndComp
$Comp
L +3.3V #PWR13
U 1 1 59F014E1
P 8000 3350
F 0 "#PWR13" H 8000 3200 50  0001 C CNN
F 1 "+3.3V" H 8000 3490 50  0000 C CNN
F 2 "" H 8000 3350 50  0000 C CNN
F 3 "" H 8000 3350 50  0000 C CNN
	1    8000 3350
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR15
U 1 1 59F014E7
P 8650 4850
F 0 "#PWR15" H 8650 4600 50  0001 C CNN
F 1 "GND" H 8650 4700 50  0000 C CNN
F 2 "" H 8650 4850 50  0000 C CNN
F 3 "" H 8650 4850 50  0000 C CNN
	1    8650 4850
	1    0    0    -1  
$EndComp
$Comp
L MCP6001 U1
U 1 1 59F014ED
P 9950 4000
F 0 "U1" H 10000 4200 50  0000 C CNN
F 1 "MCP6001" H 10150 3800 50  0000 C CNN
F 2 "" H 9900 4100 50  0000 C CNN
F 3 "" H 10000 4200 50  0000 C CNN
	1    9950 4000
	1    0    0    -1  
$EndComp
$Comp
L R R9
U 1 1 59F014F3
P 8000 3750
F 0 "R9" V 8080 3750 50  0000 C CNN
F 1 "680" V 8000 3750 50  0000 C CNN
F 2 "" V 7930 3750 50  0000 C CNN
F 3 "" H 8000 3750 50  0000 C CNN
	1    8000 3750
	1    0    0    -1  
$EndComp
Text Notes 8650 850  0    120  ~ 0
Encoder links
Text Notes 8650 3350 0    120  ~ 0
Encoder rechts
$Comp
L GND #PWR10
U 1 1 59F02393
P 3600 3100
F 0 "#PWR10" H 3600 2850 50  0001 C CNN
F 1 "GND" H 3600 2950 50  0000 C CNN
F 2 "" H 3600 3100 50  0000 C CNN
F 3 "" H 3600 3100 50  0000 C CNN
	1    3600 3100
	1    0    0    -1  
$EndComp
$Comp
L +3.3V #PWR2
U 1 1 59F03941
P 1100 1900
F 0 "#PWR2" H 1100 1750 50  0001 C CNN
F 1 "+3.3V" H 1100 2040 50  0000 C CNN
F 2 "" H 1100 1900 50  0000 C CNN
F 3 "" H 1100 1900 50  0000 C CNN
	1    1100 1900
	1    0    0    -1  
$EndComp
$Comp
L R R8
U 1 1 59EFB886
P 2300 7350
F 0 "R8" V 2400 7350 50  0000 C CNN
F 1 "11k" V 2300 7350 50  0000 C CNN
F 2 "" V 2230 7350 50  0000 C CNN
F 3 "" H 2300 7350 50  0000 C CNN
	1    2300 7350
	-1   0    0    1   
$EndComp
$Comp
L Adafruit_ADS1015 P4
U 1 1 59EFBC85
P 1300 7000
F 0 "P4" H 1300 7550 50  0000 C CNN
F 1 "Adafruit_ADS1015" V 1300 7000 50  0000 C CNN
F 2 "" H 1300 6050 50  0000 C CNN
F 3 "" H 1300 6050 50  0000 C CNN
	1    1300 7000
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR6
U 1 1 59EFBE21
P 1700 7600
F 0 "#PWR6" H 1700 7350 50  0001 C CNN
F 1 "GND" H 1700 7450 50  0000 C CNN
F 2 "" H 1700 7600 50  0000 C CNN
F 3 "" H 1700 7600 50  0000 C CNN
	1    1700 7600
	1    0    0    -1  
$EndComp
$Comp
L +3.3V #PWR4
U 1 1 59EFC1F5
P 1600 6450
F 0 "#PWR4" H 1600 6300 50  0001 C CNN
F 1 "+3.3V" H 1600 6590 50  0000 C CNN
F 2 "" H 1600 6450 50  0000 C CNN
F 3 "" H 1600 6450 50  0000 C CNN
	1    1600 6450
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR1
U 1 1 59EFC7EC
P 1000 5100
F 0 "#PWR1" H 1000 4850 50  0001 C CNN
F 1 "GND" H 1000 4950 50  0000 C CNN
F 2 "" H 1000 5100 50  0000 C CNN
F 3 "" H 1000 5100 50  0000 C CNN
	1    1000 5100
	1    0    0    -1  
$EndComp
$Comp
L C C5
U 1 1 5A7631BD
P 1950 7350
F 0 "C5" H 1975 7450 50  0000 L CNN
F 1 "100n" H 1975 7250 50  0000 L CNN
F 2 "" H 1988 7200 50  0000 C CNN
F 3 "" H 1950 7350 50  0000 C CNN
	1    1950 7350
	1    0    0    -1  
$EndComp
Wire Wire Line
	1700 5700 1700 6050
Wire Wire Line
	1600 3700 2650 3700
Wire Wire Line
	3050 3700 3200 3700
Wire Wire Line
	1100 1900 1100 3200
Wire Wire Line
	1600 3200 2300 3200
Wire Wire Line
	2300 3200 2300 1900
Wire Wire Line
	3050 3000 3200 3000
Wire Wire Line
	1700 5700 1500 5700
Wire Wire Line
	1600 5800 1500 5800
Wire Wire Line
	1100 3400 600  3400
Wire Wire Line
	600  3350 600  5350
Wire Wire Line
	600  5350 1900 5350
Wire Wire Line
	1900 5350 1900 6750
Wire Wire Line
	1900 5900 1500 5900
Wire Wire Line
	1100 3300 700  3300
Wire Wire Line
	700  3300 700  5300
Wire Wire Line
	700  5300 2000 5300
Wire Wire Line
	2000 5300 2000 6850
Wire Wire Line
	2000 6000 1500 6000
Wire Wire Line
	8000 1450 8150 1450
Wire Wire Line
	8000 900  8000 1150
Wire Wire Line
	8000 2250 10400 2250
Wire Wire Line
	9250 1350 9250 1500
Wire Wire Line
	8650 1450 9650 1450
Connection ~ 9250 1450
Wire Wire Line
	9250 1100 9050 1100
Wire Wire Line
	8000 950  10400 950 
Connection ~ 8000 950 
Wire Wire Line
	9250 1800 9250 1900
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
	8650 1750 8650 2400
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
Wire Wire Line
	3900 1900 3900 3700
Wire Wire Line
	3900 3700 3500 3700
Wire Wire Line
	2300 6500 2300 6550
Wire Wire Line
	8000 3900 8150 3900
Wire Wire Line
	8000 3350 8000 3600
Wire Wire Line
	8000 4700 10400 4700
Wire Wire Line
	9250 3800 9250 3950
Wire Wire Line
	8650 3900 9650 3900
Connection ~ 9250 3900
Wire Wire Line
	9250 3550 9050 3550
Wire Wire Line
	8000 3400 10400 3400
Connection ~ 8000 3400
Wire Wire Line
	9250 4250 9250 4350
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
	8650 4200 8650 4850
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
Wire Wire Line
	1600 5600 1600 5800
Wire Wire Line
	2300 7500 2300 7600
Wire Wire Line
	1100 5100 1000 5100
Wire Wire Line
	1600 6550 1600 6450
Wire Wire Line
	1550 6650 1700 6650
Wire Wire Line
	1700 6650 1700 7600
Wire Wire Line
	1900 6750 1550 6750
Connection ~ 1900 5900
Wire Wire Line
	2000 6850 1550 6850
Connection ~ 2000 6000
Wire Wire Line
	2300 6850 2300 7200
Wire Wire Line
	1550 6550 1600 6550
Wire Wire Line
	1550 7150 2300 7150
Connection ~ 2300 7150
Wire Wire Line
	1950 7200 1950 7150
Connection ~ 1950 7150
Wire Wire Line
	1950 7500 1950 7550
Wire Wire Line
	1950 7550 2300 7550
Connection ~ 2300 7550
$Comp
L Crystal Piezo
U 1 1 5AAD9521
P 3350 4200
F 0 "Piezo" H 3350 4350 50  0000 C CNN
F 1 "Buzzer" H 3350 4050 50  0000 C CNN
F 2 "" H 3350 4200 50  0000 C CNN
F 3 "" H 3350 4200 50  0000 C CNN
	1    3350 4200
	1    0    0    -1  
$EndComp
Wire Wire Line
	1600 4200 3200 4200
$Comp
L GND #PWR?
U 1 1 5AAD96F4
P 3600 4400
F 0 "#PWR?" H 3600 4150 50  0001 C CNN
F 1 "GND" H 3600 4250 50  0000 C CNN
F 2 "" H 3600 4400 50  0000 C CNN
F 3 "" H 3600 4400 50  0000 C CNN
	1    3600 4400
	1    0    0    -1  
$EndComp
Wire Wire Line
	3500 4200 3600 4200
Wire Wire Line
	3600 4200 3600 4400
$EndSCHEMATC
