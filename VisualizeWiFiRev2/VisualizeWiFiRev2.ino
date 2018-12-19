/************************************************************

Visualize

This example wraps the official implementation of MadgwickAHRS algorithm to get orientation of an object based on accelerometer and gyroscope readings

Development environment specifics:
  IDE: Arduino 1.8.4
  Hardware Platform:
  - Arduino WiFi Rev2

Distributed as-is; no warranty is given.
************************************************************************/
#include <SPI.h>
#include "SparkFunLSM6DS3.h"
#include <MadgwickAHRS.h>

LSM6DS3 myIMU( SPI_MODE, SPIIMU_SS );  //SPIIMU_SS is the CS pin for Arduino WiFi Rev2

Madgwick filter;
unsigned long microsPerReading, microsPrevious;
float accelScale, gyroScale;

void setup() {
  Serial.begin(9600);

  // start the IMU and filter
  myIMU.settings.gyroSampleRate = 26;   //Hz.  Can be: 13, 26, 52, 104, 208, 416, 833, 1666
  myIMU.settings.accelSampleRate = 26;  //Hz.  Can be: 13, 26, 52, 104, 208, 416, 833, 1666, 3332, 6664, 13330
  
  filter.begin(26);

  // Set the accelerometer range to 2G
  myIMU.settings.accelRange = 2;      //Max G force readable.  Can be: 2, 4, 8, 16
  // Set the gyroscope range to 250 degrees/second
  myIMU.settings.gyroRange = 245;   //Max deg/s.  Can be: 125, 245, 500, 1000, 2000
  
  //Call .beginCore() to configure the IMU
  if( myIMU.beginCore() != 0 )
  {
    Serial.print("Error at beginCore().\n");
  }
  else
  {
    Serial.print("\nbeginCore() passed.\n");
  }

  //Call .begin() to configure the IMU
  myIMU.begin();

  // initialize variables to pace updates to correct rate
  microsPerReading = 1000000 / 26;
  microsPrevious = micros();
}

void loop() {
  int aix, aiy, aiz;
  int gix, giy, giz;
  float ax, ay, az;
  float gx, gy, gz;
  float roll, pitch, heading;
  unsigned long microsNow;

  // check if it's time to read data and update the filter
  microsNow = micros();
  if (microsNow - microsPrevious >= microsPerReading) {

    // read raw data from CurieIMU
    ax = myIMU.readFloatAccelX();
    ay = myIMU.readFloatAccelY();
    az = myIMU.readFloatAccelZ();
    gx = myIMU.readFloatGyroX();
    gy = myIMU.readFloatGyroY();
    gz = myIMU.readFloatGyroZ();
    
    // update the filter, which computes orientation
    filter.updateIMU(gx, gy, gz, ax, ay, az);

    // print the heading, pitch and roll
    roll = filter.getRoll();
    pitch = filter.getPitch();
    heading = filter.getYaw();
    Serial.print("Orientation: ");
    Serial.print(heading);
    Serial.print(" ");
    Serial.print(pitch);
    Serial.print(" ");
    Serial.println(roll);

    // increment previous time, so we keep proper pace
    microsPrevious = microsPrevious + microsPerReading;
  }
}
