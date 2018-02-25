#include <Encoder.h>
#include <Wire.h>
#include "NAxisMotion.h"

//IMU object
NAxisMotion IMU;

//motor speeds
Encoder motorEncL(2, 3);
Encoder motorEncR(18, 19);
float leftMotorRevsPrevious;
float rightMotorRevsPrevious;
float loopTime;
float lastTime;

bool humanReadable = false;
String cc = String(",");
String imu = String("i");
String motor = String("m");

void setup() 
{
  //start serial communication with computer
  Serial.begin(115200);
    
  //Initialize I2C communication to the let the library communicate with the sensor.
  I2C.begin();
  
  //IMU Sensor Initialization
  IMU.initSensor();          //The I2C Address can be changed here inside this function in the library
  IMU.setOperationMode(OPERATION_MODE_NDOF);   //Can be configured to other operation modes as desired
  IMU.setUpdateMode(MANUAL);  //The default is AUTO. Changing to MANUAL requires calling the relevant update functions prior to calling the read functions
}

void loop() 
{    
    //read all sensor data and print to serial//

    //update IMU information
    IMU.updateEuler();     
    IMU.updateCalibStatus();  //Update the Calibration Status
    
    //update motor positions
    long leftMotorPosition = motorEncL.read();
    long rightMotorPosition = motorEncR.read();

    float leftMotorRevs = float(leftMotorPosition) / 240; //240 is the number of encoder ticks per revolution of output shaft
    float rightMotorRevs = float(rightMotorPosition) / 240;
    
    //approximate wheel velocities
    loopTime = (millis()-lastTime)/1000.0;
    float leftMotorVel =  ((leftMotorRevs - leftMotorRevsPrevious)/ loopTime) * 0.1524 * 3.14159; // [m/s]
    float rightMotorVel = ((rightMotorRevs - rightMotorRevsPrevious)/ loopTime) * 0.1524 * 3.14159;
    lastTime = millis();
    
    if(humanReadable) {
      //print everything to serial
      Serial.print(" Yaw: ");
      Serial.print(IMU.readEulerHeading()); //Heading data
      Serial.print(" deg ");
      
      Serial.print(" Motor L Vel: ");
      Serial.print(leftMotorVel);
      Serial.print(" m/s ");
      
      Serial.print(" Motor R Vel: ");
      Serial.print(rightMotorVel);
      Serial.print(" m/s ");
      
      Serial.println();
      
    } else {
      Serial.println(imu + IMU.readEulerHeading() + cc + IMU.readEulerRoll() + cc + IMU.readEulerPitch());
      Serial.println(motor + leftMotorRevs + cc + rightMotorRevs + cc + leftMotorVel + cc + rightMotorVel);
    }
    
    leftMotorRevsPrevious = leftMotorRevs; //save last wheel position
    rightMotorRevsPrevious = rightMotorRevs;
    
    delay(50);
}
