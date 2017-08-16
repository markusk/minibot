# minibot
My little Raspberry Pi and ROS robot - with some python stuff

## Setup
### Motor Hat
<ol>
<li>cd ~</li>
<li>git clone https://github.com/adafruit/Adafruit-Motor-HAT-Python-Library.git</li>
<li>cd Adafruit-Motor-HAT-Python-Library</li>
<li>sudo apt-get install python-dev</li>
<li>sudo python setup.py install</li>
</ol>

### OLED LCD
<ol>
<li>cd ~</li>
<li>sudo apt-get install build-essential python-dev python-pip</li>
<li>sudo pip install RPi.GPIO</li>
<li>sudo apt-get install python-imaging python-smbus</li>
<li>git clone https://github.com/adafruit/Adafruit_Python_SSD1306.git</li>
<li>cd Adafruit_Python_SSD1306</li>
<li>sudo python setup.py install</li>
</ol>

### AD converter (MCP3008)
<ol>
<li>cd ~</li>
<li>git clone https://github.com/doceme/py-spidev/</li>
<li>cd py-spidev</li>
<li>sudo python setup.py install</li>
</ol>

## Run/Test
<ol>
<li>cd ~/minibot/test</li>
<li>./minibot</li>
</ol>

## ROS Setup
<ol>
<li>Install ROS on Raspberry Pi</li>
<li>sudo apt-get install ros-kinetic-urg-node ros-kinetic-teleop-twist-keyboard</li>
<li>create a catkin workspace without 'src' folder:<br>
$ mkdir catkin_ws<br>
$ cd ~/catkin_ws
</li>
<li>create symbolic link with the name 'src' point to the 'src' folder in the ROS directory in this repository.<br>
$ ln -s /home/$USERNAME/minibot/ROS/catkin_workspace/src/ src
</li>
<li>$ catkin_make</li>
</ol>

## Run/Test ROS
<ol>
<li>Run master on Raspi/other computer</li>
<li>On the corresponding other computer, the "ROS slave":</li>
$ export ROS_MASTER_URI=http://name-or-IP:11311
</ol>
