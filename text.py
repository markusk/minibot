#!/usr/bin/python

import Adafruit_GPIO.SPI as SPI
import Adafruit_SSD1306

# 128x32 display with hardware I2C:
disp = Adafruit_SSD1306.SSD1306_128_32(rst=RST)

# Initialize library.
disp.begin()

# Clear display.
disp.clear()
disp.display()

# x,y
disp.setCursor(0, 0);
# disp.setTextColor(WHITE;
# void setTextColor(uint16_t color, uint16_t backgroundcolor);
disp.setTextSize(12);
disp.setTextWrap(false);

# Display image.
# disp.image(image)
disp.display()
