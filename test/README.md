# Low level test files
Several programs for my little robot "minibot"

## battery.py
Shows the battery voltage (in the shell).

## beep.py
Sends n beeps to a piezo buzzer.

## buttontest.py
Checks almost any input GPIO for a LOW signal.

## cpu_temp.py
Shows the CPU temperature of the Raspberry Pi in the shell.

## hostinfo.py
Prints out the hostname and the IP4 address.

## lcd_test.py
Sends a default/text text to the OLED.

## lcd_text.py
Sends text from the command line to the OLED. Usage:  
<code>lcd_test.py Text1 Text2</code>

## led_blinky.py
Turns a LED, connected to the Raspberry Pi GPIO #18 on and off.

## minibot.py
(work in progress)

## motor_test.py
Starts the robot to drive. It will always turn left and right for one second.

## piezo.py
Lets a piezo beep, connected to the Raspberry Pi GPIO #25. Also turns a LED, connected to GPIO #18 on and off with the same frequency.

## power_monitor.py
Observes and shows the battery voltage (on the OLED). Also checks a push button and shuts down the Raspberry Pi if pressed (after 5 seconds).
