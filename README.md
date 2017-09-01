# minibot
My little Raspberry Pi and ROS robot - with some python stuff

## Setup
### Motor Hat
```
cd ~
git clone  https://github.com/adafruit/Adafruit-Motor-HAT-Python-Library.git
cd Adafruit-Motor-HAT-Python-Library
sudo apt-get install python-dev
sudo python setup.py install
```

### OLED LCD (I2C)
```
cd ~
sudo apt-get install build-essential python-dev python-pip
sudo pip install RPi.GPIO
sudo apt-get install python-imaging python-smbus
git clone https://github.com/adafruit/Adafruit_Python_SSD1306.git
cd Adafruit_Python_SSD1306
sudo python setup.py install
```

### AD converter (MCP3008 via SPI)
```
cd ~
git clone https://github.com/doceme/py-spidev/
cd py-spidev
sudo python setup.py install
```

## Run/Test
```
cd ~/minibot/test
./minibot
```

## ROS Setup
- Install ROS on Raspberry Pi
- Install necessary ROS packages:

```
sudo apt-get install ros-kinetic-urg-node ros-kinetic-teleop-twist-keyboard
```
- create a catkin workspace without 'src' folder:
```
mkdir catkin_ws<br>
cd ~/catkin_ws
```
- create symbolic link with the name 'src' point to the 'src' folder in the ROS directory in this repository:
```
ln -s /home/$USERNAME/minibot/ROS/catkin_workspace/src/ src
catkin_make
```


## Run/Test ROS
- Run master on Raspi/other computer
- On the corresponding other computer, the "ROS slave":
```
export ROS_MASTER_URI=http://hostname:11311
```
