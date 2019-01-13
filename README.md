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


### AD converter ~~(MCP3008 via SPI)~~ (ADS1015 via I2C)
```
cd ~
sudo apt-get install build-essential python-dev python-smbus
git clone https://github.com/adafruit/Adafruit_Python_ADS1x15.git
cd Adafruit_Python_ADS1x15
sudo python setup.py install
```


### IMU (BNO055 via I2C)
```
cd ~
sudo apt-get install build-essential python-dev python-smbus
git clone https://github.com/adafruit/Adafruit_Python_BNO055.git
cd Adafruit_Python_BNO055
sudo python setup.py install
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

## Other helpful stuff
- joystick/gamepad driver/test programm
```
sudo apt-get install joystick
```



## Run/Test ROS
### The main launch file
On the robot (Raspberry Pi):
```
roslaunch minibot.launch
```
On another computer:
```
export ROS_MASTER_URI=http://hostname:11311
roslaunch ground_control.launch
```


### The different launch files
#### battery.launch
Observes the battery voltage. Launch file for demo purposes only.
:arrow_right: nodes/battery_publisher.py

### camera.launch
Streams the RasPi camera image. Can be views with the ROS image_view component.
:arrow_right: src/camera.cpp

### joystick_control.launch
Listens to a joystick and controlls the robot directly (remote).
This part has to be started on a different computer than the robot.
The joystick/gamepad has to be connected to that computer, not the robot.
:arrow_right: nodes/joy_motor_listener.py
