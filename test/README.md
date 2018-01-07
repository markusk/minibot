# Low level test files
Several programs for my little robot "minibot"

## battery.py
Shows the battery voltage (in the shell).

## cpu_temp.py
Shows the CPU temperature of the Raspberry Pi in the shell.

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
Observes and shows the battery voltage (on the OLED).
