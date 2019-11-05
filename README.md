# minibot
My little Raspberry Pi and ROS robot - with some python stuff

[![GitHub issues](https://img.shields.io/github/issues/markusk/minibot)](https://github.com/markusk/minibot/issues)
[![GitHub stars](https://img.shields.io/github/stars/markusk/minibot)](https://github.com/markusk/minibot/stargazers)
[![GitHub license](https://img.shields.io/github/license/markusk/minibot)](https://github.com/markusk/minibot/blob/master/LICENSE)

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
- joystick/gamepad driver/test program
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
#### battery
Observes the battery voltage. For demo/test purpose only. Uses:
- _nodes/battery_publisher.py_

#### camera
Streams the RasPi camera image. Can be viewed with the ROS image_view component. Uses:
- _src/camera.cpp_

#### ground_control
Controls and observes the robots status or sensors (remote). This part has to be started on a different computer than the robot. Uses:
- _rqt_plot_
- _joystick_control.launch_
- _image_view_
- _rviz_

#### joystick_control
Listens to a joystick and controls the robot directly (remote). This part has to be started on a different computer than the robot. The joystick/gamepad has to be connected to that computer, not the robot. Uses:
- _nodes/joy_motor_listener.py_

#### keyboard_control
Listens to a keyboard and controls the robot directly (remote). This part has to be started on a different computer than the robot. The joystick/gamepad has to be connected to that computer, not the robot. Uses:
- _nodes/motor_server.py_
- _teleop_twist_keyboard_
- _nodes/keyboard_motor_listener.py_

#### keyboard_control_test
Listens to a teleop_twist_keyboard node and prints out the data/messages. Uses:
- _teleop_twist_keyboard_
- _nodes/keyboard_listener.py_

#### laser
Starts the urg_node for the laser range finder. This launch file is for convenience, to have only one ssh session open. For demo/test purpose only.

#### led_server
Turns LEDs on the robot ON or OFF. This is the node for the robot. For demo/test purpose only. Uses:
- _srv/Led.srv_

#### macbook_test
[ ] (details to be checked). Uses:
- _minibot.urdf.xacro_
- _tf_broadcaster.py_
- _battery_publisher.py_
- _imu_bno055.py_
- _motor_server.py_
- _base_controller.py_

#### mapping
[ ] (robot launch file, details to be checked). Uses:
- _tf_broadcaster.py_
- _battery_publisher.py_
- _imu_bno055.py_
- _motor_server.py_
- _base_controller.py_
- _urg_node_

#### minibot
Controls the whole robot. To be started on the robot. Uses:
- _tf_broadcaster.py_
- _battery_publisher.py_
- _imu_bno055.py_
- _motor_server.py_
- _base_controller.py_
- _minibot_camera_
- _urg_node_

#### motor_server
Controls the motors on the robot. Uses:
- _motor_server.py_
