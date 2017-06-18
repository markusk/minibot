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

// stores the callibration status data
uint8_t system = 0;
uint8_t gyro   = 0;
uint8_t accel  = 0;
uint8_t mag    = 0;



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


  // 10x6 lines availabe
  //           1234567890
  uView.clear(PAGE);

  // Name
  //uView.setCursor(0, 0*fontHeight);
  //uView.print(sensor.name);

  // Driver Version
  uView.setCursor(0, 0*fontHeight);
  uView.print("Version: ");
  uView.print(sensor.version);

  // Unique ID
  uView.setCursor(0, 1*fontHeight);
  uView.print("ID:  ");
  uView.print(sensor.sensor_id);

  // Min Value
  uView.setCursor(0, 3*fontHeight);
  uView.print("Min: ");
  uView.print(sensor.min_value);

  // Max Value
  uView.setCursor(0, 2*fontHeight);
  uView.print("Max: ");
  uView.print(sensor.max_value);

  // Resolution
  uView.setCursor(0, 4*fontHeight);
  uView.print("Res: ");
  uView.print(sensor.resolution); 

  // show it
  uView.display();
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

  /* Display the results */
  uView.clear(PAGE);

  uView.setCursor(0, 0*fontHeight);
  uView.print("System:0x");
  uView.print(system_status, HEX);

  uView.setCursor(0, 1*fontHeight);
  uView.print("Test:  0x");
  uView.print(self_test_results, HEX);

  uView.setCursor(0, 2*fontHeight);
  uView.print("Error: 0x");
  uView.print(system_error, HEX);

  // show it
  uView.display();
}


void getCalStatus(void)
{
  /* Get the four calibration values (0..3) */
  /* Any sensor data reporting 0 should be ignored, */
  /* 3 means 'fully calibrated" */
  bno.getCalibration(&system, &gyro, &accel, &mag);
}


/**************************************************************************/
/*
    Display sensor calibration status
 */
/**************************************************************************/
void displayCalStatus(void)
{
  // get status
  getCalStatus();
  
  
  /* The data should be ignored until the system calibration is > 0 */
  //Serial.print("\t");
  if (!system)
  {
    uView.clear(PAGE);
    uView.setCursor(0, 0*fontHeight);
    uView.print("Waiting...");
    // show it
    uView.display();
  }


  uView.clear(PAGE);

  uView.setCursor(0, 0*fontHeight);
  uView.print("Callibrat:");

  // Sys
  uView.setCursor(0, 1*fontHeight);
  uView.print("Sys:  ");
  uView.print(system, DEC);

  // Gyrocope
  uView.setCursor(0, 2*fontHeight);
  uView.print("Gyro: ");
  uView.print(gyro, DEC);

  // Accelerometer
  uView.setCursor(0, 3*fontHeight);
  uView.print("Accel:");
  uView.print(accel, DEC);

  // Magnetometer
  uView.setCursor(0, 4*fontHeight);
  uView.print("Mag:  ");
  uView.print(mag, DEC);

  // show it
  uView.display();
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


  /* Initialise the sensor */
  if(!bno.begin())
  {
    uView.clear(PAGE);
    uView.setCursor(0,0);
    /* There was a problem detecting the BNO055 ... check your connections */
    uView.println("Ooops, no    BNO055 detected!");
    uView.print("Please check I2C.");
    uView.display();

    while(1);
  }

  delay(1000);

  /* Display some basic information on this sensor */
  displaySensorDetails();
  delay(1000);

  /* Optional: Display current status */
  displaySensorStatus();
  delay(1000);

  bno.setExtCrystalUse(true);
}


void loop(void)
{
  /* Get a new sensor event */
  sensors_event_t event;
  bno.getEvent(&event);


  /* Optional: Display calibration status * /
  displayCalStatus();
  delay(250);
  */


  uView.clear(PAGE);

  uView.setCursor(0, 0*fontHeight);
  uView.print("9DOF Live!");

  //uView.setCursor(0, 1*fontHeight);
  //uView.print("----------");

  uView.setCursor(0, 1*fontHeight);
  uView.print("X:");
  if (event.orientation.x < 99)
    uView.print(" ");
  if (event.orientation.x < 9)
    uView.print(" ");
  uView.print(event.orientation.x, 4);

  uView.setCursor(0, 2*fontHeight);
  uView.print("Y:");
  if (event.orientation.y < 99)
    uView.print(" ");
  if (event.orientation.y < 9)
    uView.print(" ");
  uView.print(event.orientation.y, 4);

  uView.setCursor(0, 3*fontHeight);
  uView.print("Z:");
  if (event.orientation.z < 99)
    uView.print(" ");
  if (event.orientation.z < 9)
    uView.print(" ");
  uView.print(event.orientation.z, 4);

  // check callibration
  getCalStatus();

  uView.setCursor(0, 4*fontHeight);
  uView.print("System: ");
  uView.print(system);

  uView.display();


  /* Wait the specified delay before requesting nex data */
  delay(BNO055_SAMPLERATE_DELAY_MS);
}


