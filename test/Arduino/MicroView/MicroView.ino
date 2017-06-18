// MicroView OLED
#include <MicroView.h>

// Adafruit BNO055
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>


void setup()
{
        // begin of MicroView
	uView.begin();		
	uView.clear(ALL);	// erase hardware memory inside the OLED controller
	uView.display();	// display the content in the buffer memory, by default it is the MicroView logo
	delay(700);
	uView.clear(PAGE);	// erase the memory buffer, when next uView.display() is called, the OLED will be cleared.

}


void loop()
{
        // MicroView
        // TEXT Font 0
	uView.clear(PAGE);
	uView.setCursor(0,40);
	uView.print("  Font 0  ");    
	uView.display();

	uView.setFontType(0);
	uView.setCursor(0,0);
	uView.print("01234567890ABCDabcd01234567890ABCDabcd");
	uView.display();
	delay(1500);


        // TEXT Font 1
	uView.clear(PAGE);
	uView.setCursor(0,40);
	uView.print("  Font 1  ");    
	uView.display();

	uView.setFontType(1);
	uView.setCursor(0,0);
	uView.print("0123ABCDabcd");
	uView.display();
	delay(1500);
	uView.clear(PAGE);
}


