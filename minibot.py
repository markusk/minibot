#!/usr/bin/python

try:
    import RPi.GPIO as GPIO
except RuntimeError:
    print("Error importing RPi.GPIO!  This is probably because you need superuser privileges.  You can achieve this by using 'sudo' to run your script")

import time # for sleep

# print some infos
# print GPIO.RPI_INFO

# timings
secs = 5

# init
GPIO.setmode(GPIO.BCM) # use the GPIO names, _not_ the pin numbers on the board

# pins				BOARD	 BCM
motor1A   = 17 # Motor 1 A	pin 11 = GPIO 17
motor1B   = 27 # Motor 1 B	pin 13 = GPIO 27
motor1PWM = 22 # Motor 1 PWM	pin 15 = GPIO 22
motor2A   = 25 # Motor 2 A	pin 22 = GPIO 25
motor2B   = 8  # Motor 2 B	pin 24 = GPIO  8
motor2PWM = 7  # Motor 2 PWM	pin 26 = GPIO  7

# speed in percent
motor1speed = 128 # 128
motor2speed = 128 # 128
# PWM duty cycle
motor1dc = 1.0
motor2dc = 1.0

# PWM frequency in Hz
motor1freq = 100
motor2freq = 100

# setup GPIO outputs
print('starting setup...')
GPIO.setup(motor1A,   GPIO.OUT)
GPIO.setup(motor1B,   GPIO.OUT)
GPIO.setup(motor1PWM, GPIO.OUT)
GPIO.setup(motor2A,   GPIO.OUT)
GPIO.setup(motor2B,   GPIO.OUT)
GPIO.setup(motor2PWM, GPIO.OUT)

# setup PWM Hz
# pwm1 = GPIO.PWM(motor1PWM, motor1freq)
# pwm2 = GPIO.PWM(motor2PWM, motor2freq)
# neu neu neu
GPIO.output(motor1PWM, GPIO.HIGH)
GPIO.output(motor2PWM, GPIO.HIGH)

# start pwm
print('starting PWM...')
#pwm1.start(motor1dc)
#pwm2.start(motor2dc)

# --- drive ---
print('!!driving forward for ' + str(secs) + ' seconds !!')
# motor 1 forward
GPIO.output(motor1A, GPIO.HIGH)
GPIO.output(motor1B, GPIO.LOW)
# motor 2 forward
GPIO.output(motor2A, GPIO.HIGH)
GPIO.output(motor2B, GPIO.LOW)

# delay x seconds
time.sleep(secs)
	
# --- stop ---
print('motor off...')
# motor 1
GPIO.output(motor1A, GPIO.LOW)
GPIO.output(motor1B, GPIO.LOW)
# motor 2
GPIO.output(motor2A, GPIO.LOW)
GPIO.output(motor2B, GPIO.LOW)
	
# stop pwm
print('stopping PWM...')
# pwm1.stop()
# pwm2.stop()
# neu neu neu
GPIO.output(motor1PWM, GPIO.LOW)
GPIO.output(motor2PWM, GPIO.LOW)

# cleanup
GPIO.cleanup()
