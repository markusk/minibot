// Sensor stuff
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

// MicroView OLED
#include <MicroView.h>

const int fontHeight = 10;

/* This driver uses the Adafruit unified sensor library (Adafruit_Sensor),
 which provides a common 'type' for sensor data and some helper functions.
 
 To use this driver you will also need to download the Adafruit_Sensor
 library and include it in your libraries folder.
 
 You should also assign a unique ID to this sensor for use with
 the Adafruit Sensor API so that you can identify this particular
 sensor in any data logs, etc.  To assign a unique ID, simply
 provide an appropriate value in the constructor below (12345
 is used by default in this example).
 
 Connections
 ===========
 Connect SCL to analog 5
 Connect SDA to analog 4
 Connect VDD to 3-5V DC
 Connect GROUND to common ground
 
 History
 =======
 2015/MAR/03  - First release (KTOWN)
 2015/AUG/27  - Added calibration and system status helpers
 */

/* Set the delay between fresh samples */
#define BNO055_SAMPLERATE_DELAY_MS (100)

Adafruit_BNO055 bno = Adafruit_BNO055(55);

/**************************************************************************/
/*
    Displays some basic information on this sensor from the unified
 sensor API sensor_t type (see Adafruit_Sensor for more information)
 */
/**************************************************************************/
void displaySensorDetails(void)
{
  sensor_t sensor;
  bno.getSensor(&sensor);
  char buffer[7];         //the ASCII of the integer will be stored in this char array

  uView.clear(PAGE);

  //Serial.print  ("Sensor:       "); 
  uView.setCursor(0, 0*fontHeight);
  uView.print(sensor.name);

  //Serial.print  ("Driver Ver:   "); 
  uView.setCursor(0, 1*fontHeight);
  uView.print("Version: ");
  itoa(sensor.version, buffer, 10);
  uView.print(buffer);

  //Serial.print  ("Unique ID:    ");
  uView.setCursor(0, 2*fontHeight);
  uView.print("ID: ");
  itoa(sensor.sensor_id, buffer, 10);
  uView.print(buffer);

  //Serial.print  ("Max Value:    "); 
  Serial.print(sensor.max_value); 
  //Serial.println(" xxx");
  Serial.print  ("Min Value:    "); 
  Serial.print(sensor.min_value); 
  //Serial.println(" xxx");
  Serial.print  ("Resolution:   "); 
  Serial.print(sensor.resolution); 
  //Serial.println(" xxx");
  //Serial.println("------------------------------------");
  //Serial.println("");

  uView.display();

  delay(500);
}

/**************************************************************************/
/*
    Display some basic info about the sensor status
 */
/**************************************************************************/
void displaySensorStatus(void)
{
  /* Get the system status values (mostly for debugging purposes) */
  uint8_t system_status, self_test_results, system_error;
  system_status = self_test_results = system_error = 0;
  bno.getSystemStatus(&system_status, &self_test_results, &system_error);

  /* Display the results in the Serial Monitor */
  Serial.println("");
  Serial.print("System Status: 0x");
  Serial.println(system_status, HEX);
  Serial.print("Self Test:     0x");
  Serial.println(self_test_results, HEX);
  Serial.print("System Error:  0x");
  Serial.println(system_error, HEX);
  Serial.println("");
  delay(500);
}

/**************************************************************************/
/*
    Display sensor calibration status
 */
/**************************************************************************/
void displayCalStatus(void)
{
  /* Get the four calibration values (0..3) */
  /* Any sensor data reporting 0 should be ignored, */
  /* 3 means 'fully calibrated" */
  uint8_t system, gyro, accel, mag;
  system = gyro = accel = mag = 0;
  bno.getCalibration(&system, &gyro, &accel, &mag);

  /* The data should be ignored until the system calibration is > 0 */
  Serial.print("\t");
  if (!system)
  {
    Serial.print("! ");
  }

  /* Display the individual values */
  Serial.print("Sys:");
  Serial.print(system, DEC);
  Serial.print(" G:");
  Serial.print(gyro, DEC);
  Serial.print(" A:");
  Serial.print(accel, DEC);
  Serial.print(" M:");
  Serial.print(mag, DEC);
}


void setup(void)
{
  // begin of MicroView
  uView.begin();		
  uView.clear(ALL);	// erase hardware memory inside the OLED controller
  uView.display();	// display the content in the buffer memory, by default it is the MicroView logo
  delay(700);
  uView.clear(PAGE);	// erase the memory buffer, when next uView.display() is called, the OLED will be cleared.

  // set font and cursor
  uView.setFontType(0);
  uView.setCursor(0,0);

  // 10x6 lines availabe
  //           1234567890
  uView.print(" Robotik- ");
  uView.print(" labor :) ");
  uView.print("----------");
  uView.print(" Adafruit ");
  uView.print("  BNO055  ");
  uView.print("Sensortest");
  uView.display();

  //Serial.begin(9600);
  //Serial.println("Orientation Sensor Test"); 
  //Serial.println("");

  /* Initialise the sensor */
  if(!bno.begin())
  {
    uView.clear(PAGE);
    uView.setCursor(0,0);
    /* There was a problem detecting the BNO055 ... check your connections */
    //Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    uView.println("Ooops, no    BNO055 detected!");
    uView.print("Please reset HW.");
    uView.display();

    while(1);
  }

  delay(1000);

  /* Display some basic information on this sensor */
  displaySensorDetails();

  /* Optional: Display current status */
  displaySensorStatus();

  bno.setExtCrystalUse(true);
}


void loop(void)
{
  /*
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
   */

  /* Get a new sensor event */
  sensors_event_t event;
  bno.getEvent(&event);

  /* Display the floating point data */
  Serial.print("X: ");
  Serial.print(event.orientation.x, 4);
  Serial.print("\tY: ");
  Serial.print(event.orientation.y, 4);
  Serial.print("\tZ: ");
  Serial.print(event.orientation.z, 4);

  /* Optional: Display calibration status */
  displayCalStatus();

  /* Optional: Display sensor status (debug only) */
  //displaySensorStatus();

  /* New line for the next sample */
  Serial.println("");

  /* Wait the specified delay before requesting nex data */
  delay(BNO055_SAMPLERATE_DELAY_MS);
}



