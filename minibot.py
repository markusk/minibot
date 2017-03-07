#!/usr/bin/python
# coding=utf-8

# for getting the hostname of the underlying system
import socket
# for using sensor checks in a thread
import threading
# for catching signals like keyboard interrupt (to stop threads, started here)
import signal

###### AD converter stuff
from MCP3008 import MCP3008
adc = MCP3008()
voltage = 0
voltageThreadRunning = True


###### motor stuff
from Adafruit_MotorHAT import Adafruit_MotorHAT, Adafruit_DCMotor
import time
import atexit


###### LCD stuff
import Adafruit_GPIO.SPI as SPI
import Adafruit_SSD1306

from PIL import Image
from PIL import ImageDraw
from PIL import ImageFont
# Raspberry Pi pin configuration:
RST = 24

# create a 128x32 display with hardware I2C
disp = Adafruit_SSD1306.SSD1306_128_32(rst=RST)

# showing hostname
hostname = socket.gethostname()
print("Running on host " + hostname + ".")


# run some parts only on the real robot
if hostname == 'minibot':
    # create a default motor object, no changes to I2C address or frequency
    mh = Adafruit_MotorHAT(addr=0x60)

    # recommended for auto-disabling motors on shutdown!
    def turnOffMotors():
        mh.getMotor(1).run(Adafruit_MotorHAT.RELEASE)
        mh.getMotor(2).run(Adafruit_MotorHAT.RELEASE)

    # user motor 1 and 2 on RasPi hat
    myMotor1 = mh.getMotor(1)
    myMotor2 = mh.getMotor(2)

    # turn off motors
    myMotor1.run(Adafruit_MotorHAT.RELEASE);
    myMotor2.run(Adafruit_MotorHAT.RELEASE);

    # set the speed (from 0 (off) to 255 (max speed))
    startSpeed = 100
    myMotor1.setSpeed(startSpeed)
    myMotor2.setSpeed(startSpeed)


##
## what to do on exit pgm
##
def exitMinibot():
  # run some parts only on the real robot
  if hostname == 'minibot':
    turnOffMotors();
  # stop threads
  voltageCheckThread.stop()


##
## what to do at program exit
##
atexit.register(exitMinibot)

# also catch keyboar interrupts
def signal_handler(signal, frame):
  print 'You pressed Ctrl+C!'
  exitMinibot()
  # sys.exit(0)

# establish signal handler
signal.signal(signal.SIGINT, signal_handler)



################## LCD
# Initialize library.
disp.begin()

# Clear display.
disp.clear()
disp.display()

# Create blank image for drawing.
# Make sure to create image with mode '1' for 1-bit color.
width = disp.width    # 128
height = disp.height  #  64
image = Image.new('1', (width, height))

# Get drawing object to draw on image.
draw = ImageDraw.Draw(image)

# Draw a black filled box to clear the image.
draw.rectangle((0,0,width,height), outline=0, fill=0)

# load my favorite TTF font
fontText   = ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf', 20)
fontSymbol = ImageFont.truetype('Digital Camera Symbols.ttf', 36)

# Display image.
disp.image(image)
disp.display()


######
###### Threads
######
def voltageThread():
    # Thread zum Auslesen der Sensoren
    global voltageThreadRunning, voltage
    print "Voltage sensor thread started."
    while voltageThreadRunning:
        # read AD converter (battery voltage)
        # use channel 0 on IC
        voltage = adc.read(channel = 0)
        print("Voltage: %.2f" % (voltage / 1023.0 * 3.3))
        # displaySensorwertAusgabe()
        time.sleep(1)

# start thread
voltageCheckThread = threading.Thread(target=voltageThread)
voltageCheckThread.start()


######
###### forever - or until ctrl+c  :)
######
while (True):
    # clear LCD
    draw.rectangle((0,0, width, height), outline=0, fill=0)
    # LCD battery symbol
    draw.text(( 0, 0), "d", font=fontSymbol, fill=255)
    # draw.text((55, 4), "11.0 V", font=fontText, fill=255)
    # LCD voltage
    draw.text((55, 4), str("%.2f V" % (voltage / 1023.0 * 3.3)), font=fontText, fill=255)
    # go
    disp.image(image)
    disp.display()

    # test test test "empty battery" (clear part of battery symbol)
    time.sleep(1)
    rectw = 18
    recth = 22

    draw.rectangle((8, 10, rectw, recth), outline=0, fill=0)
    time.sleep(1)
    disp.image(image)
    disp.display()

    draw.rectangle((8+1*rectw, 10, rectw, recth), outline=0, fill=0)
    time.sleep(1)
    disp.image(image)
    disp.display()

    draw.rectangle((8+2*rectw, 10, rectw, recth), outline=0, fill=0)
    time.sleep(1)
    disp.image(image)
    disp.display()


    ### drive
    # run some parts only on the real robot
    if hostname == 'minibot':
        # drive
        print("Forward! ")
        myMotor1.run(Adafruit_MotorHAT.FORWARD)
        myMotor2.run(Adafruit_MotorHAT.FORWARD)

        print("\tSpeed up...")
        for i in range(startSpeed, 255):
            myMotor1.setSpeed(i)
            myMotor2.setSpeed(i)
            time.sleep(0.01)

        print("\tSlow down...")
        for i in range(255, startSpeed, -1):
            myMotor1.setSpeed(i)
            myMotor2.setSpeed(i)
            time.sleep(0.01)

        print("Backward! ")
        myMotor1.run(Adafruit_MotorHAT.BACKWARD)
        myMotor2.run(Adafruit_MotorHAT.BACKWARD)

        print("\tSpeed up...")
        for i in range(startSpeed, 255):
            myMotor1.setSpeed(i)
            myMotor2.setSpeed(i)
            time.sleep(0.01)

        print("\tSlow down...")
        for i in range(255, startSpeed, -1):
            myMotor1.setSpeed(i)
            myMotor2.setSpeed(i)
            time.sleep(0.01)

        print("Release")
        myMotor1.run(Adafruit_MotorHAT.RELEASE)
        myMotor2.run(Adafruit_MotorHAT.RELEASE)

    # sleep
    #time.sleep(1.0)
