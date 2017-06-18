#include <MicroView.h>
#include <Time.h>

#define clocksize 24

uint16_t 	onDelay=5;		// this is the on delay in milliseconds, if there is no on delay, the erase will be too fast to clean up the screen.


void setup() {
	uView.begin();		// begin of MicroView
	uView.clear(ALL);	// erase hardware memory inside the OLED controller
	uView.display();	// display the content in the buffer memory, by default it is the MicroView logo
	setTime(10,10,01,17,1,2014);
	delay(700);
	uView.clear(PAGE);	// erase the memory buffer, when next uView.display() is called, the OLED will be cleared.
}


void loop() {
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


