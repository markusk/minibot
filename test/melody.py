#!/usr/bin/python
# coding=utf-8

"""
Testing a piezo buzzer.
Ported from C by https://github.com/leon-anavi/rpi-examples


The MIT License (MIT)

Copyright (c) 2016 Leon Anavi

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
"""


# the notes (frequencies)
cL = 129
cLS = 139
dL = 146
dLS = 156
eL = 163
fL = 173
fLS = 185
gL = 194
gLS = 207
aL = 219
aLS = 228
bL = 232

c = 261
cS = 277
d = 294
dS = 311
e = 329
f = 349
fS = 370
g = 391
gS = 415
a = 440
aS = 455
b = 466

cH = 523
cHS = 554
dH = 587
dHS = 622
eH = 659
fH = 698
fHS = 740
gH = 784
gHS = 830
aH = 880
aHS = 910
bH = 933


# for GPIO pin usage
try:
    import RPi.GPIO as GPIO
except RuntimeError:
    print("Error importing RPi.GPIO!")

# for sleep
import time

##
## GPIO stuff
##
GPIO.setmode(GPIO.BCM) # use the GPIO names, _not_ the pin numbers on the board
# Raspberry Pi pin configuration:
# pins	    BCM   BOARD
piezoPin  = 25 # pin

# GPIO setup
print('GPIO setup...')
GPIO.setup(piezoPin, GPIO.OUT)


# for signal handling
import signal
import sys

# my signal handler
def sig_handler(_signo, _stack_frame):
    # GPIO cleanup
    GPIO.cleanup()
    print "piezo terminated clean."
    sys.exit(0)

# signals to be handled
signal.signal(signal.SIGINT,  sig_handler)
signal.signal(signal.SIGHUP,  sig_handler)
signal.signal(signal.SIGTERM, sig_handler)


# This function generates the square wave
# that makes the piezo speaker sound at a determinated frequency.
def beep(note, duration):
    # This is the semiperiod of each note.
    beepDelay = (1000000/note)
    # This is how much time we need to spend on the note.
    mytime = (duration / (beepDelay*2))
    for i in range (0, mytime):
        # 1st semiperiod
        # Piezo ON (low active!)
        GPIO.output(piezoPin, GPIO.LOW)
        time.sleep(beepDelay)
        # 2nd semiperiod
        # Piezo OFF
        GPIO.output(piezoPin, GPIO.HIGH)
        time.sleep(beepDelay)

    # Add a little delay to separate the single notes
    # Piezo OFF
    GPIO.output(piezoPin, GPIO.HIGH)
    time.sleep(0.02)




######
###### forever - or until ctrl+c  :)
######
print('Beeping...')
while (True):
    beep( a, 500)
    beep( a, 500)
    beep( f, 350)
    beep( cH, 150)

    beep( a, 500)
    beep( f, 350)
    beep( cH, 150)
    beep( a, 1000)
    beep( eH, 500)

    beep( eH, 500)
    beep( eH, 500)
    beep( fH, 350)
    beep( cH, 150)
    beep( gS, 500)

    beep( f, 350)
    beep( cH, 150)
    beep( a, 1000)
    beep( aH, 500)
    beep( a, 350)

    beep( a, 150)
    beep( aH, 500)
    beep( gHS, 250)
    beep( gH, 250)
    beep( fHS, 125)

    beep( fH, 125)
    beep( fHS, 250)

    time.sleep(0.250)

    beep( aS, 250)
    beep( dHS, 500)
    beep( dH, 250)
    beep( cHS, 250)
    beep( cH, 125)

    beep( b, 125)
    beep( cH, 250)

    time.sleep(0.250)

    beep( f, 125)
    beep( gS, 500)
    beep( f, 375)
    beep( a, 125)
    beep( cH, 500)

    beep( a, 375)
    beep( cH, 125)
    beep( eH, 1000)
    beep( aH, 500)
    beep( a, 350)

    beep( a, 150)
    beep( aH, 500)
    beep( gHS, 250)
    beep( gH, 250)
    beep( fHS, 125)

    beep( fH, 125)
    beep( fHS, 250)

    time.sleep(0.250)

    beep( aS, 250)
    beep( dHS, 500)
    beep( dH, 250)
    beep( cHS, 250)
    beep( cH, 125)

    beep( b, 125)
    beep( cH, 250)

    time.sleep(0.250)

    beep( f, 250)
    beep( gS, 500)
    beep( f, 375)
    beep( cH, 125)
    beep( a, 500)

    beep( f, 375)
    beep( c, 125)
    beep( a, 1000)

    # wait 1 second
    time.sleep(1)
