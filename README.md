# minibot
My little Raspberry Pi and ROS robot - with some python stuff

## Setup
# Motor Hat
cd ~
git clone https://github.com/adafruit/Adafruit-Motor-HAT-Python-Library.git
cd Adafruit-Motor-HAT-Python-Library
sudo apt-get install python-dev
sudo python setup.py install

# OLED LCD
cd ~
sudo apt-get install build-essential python-dev python-pip
sudo pip install RPi.GPIO
sudo apt-get install python-imaging python-smbus
git clone https://github.com/adafruit/Adafruit_Python_SSD1306.git
cd Adafruit_Python_SSD1306
sudo python setup.py install

# AD converter (MCP3008)
cd ~
git clone https://github.com/doceme/py-spidev/
cd py-spidev
sudo python setup.py install

## Run/Test
cd ~/minibot
./minibot
