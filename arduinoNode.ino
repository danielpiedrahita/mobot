#include <Encoder.h>
#include <Wire.h>
#include "NAxisMotion.h"

//IMU object
NAxisMotion IMU;

//motor speeds
Encoder motorEnc(18, 19); //CHANGE THESE
float MotorRevsPrevious = 0.0;
float loopTime = 0.0;
float lastLoopStart = 0.0;
float lastTime = 0.0;
float lastMotorVel = 0.0;

//params
bool humanReadable = true;
const float loopRate = 1000.0; //hz
const float alpha = 0.8;

//for printing
String cc = String(",");
String imu = String("i");
String motor = String("m");
String times = String("t");

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
    if ((millis()-lastLoopStart)>(1000/loopRate)) {
      lastLoopStart = millis();
      
      //read all sensor data and print to serial//
      //update IMU information
      IMU.updateEuler();     
      IMU.updateCalibStatus();  //Update the Calibration Status
      
      //update motor positions
      long MotorPosition = motorEnc.read();
      float MotorRevs = float(MotorPosition) / 52.62; //52.62 is the number of encoder ticks per revolution of output shaft
      
      //approximate wheel velocities
      loopTime = (millis()-lastTime)/1000.0;
      float MotorVel =  ((MotorRevs - MotorRevsPrevious)/ loopTime) * 0.1524 * 3.14159; // [m/s]
      lastTime = millis();

      //filter wheel velocities
      float filteredMotorVel = alpha*MotorVel + (1-alpha)*lastMotorVel;
      lastMotorVel = filteredMotorVel;
      
      if(humanReadable) {
        //print everything to serial
        Serial.print(" Roll: ");
        Serial.print(IMU.readEulerRoll()); //Heading data
        Serial.print(" deg ");
  
        Serial.print(" Pitch: ");
        Serial.print(IMU.readEulerPitch()); //Heading data
        Serial.print(" deg ");
        
        Serial.print(" Yaw: ");
        Serial.print(IMU.readEulerHeading()); //Heading data
        Serial.print(" deg ");
        
        Serial.print(" Motor Vel: ");
        Serial.print(filteredMotorVel);
        Serial.print(" m/s ");

        Serial.print(" Motor Revs: ");
        Serial.print(MotorRevs);
        Serial.print(" m/s ");
  
        Serial.print(" Loop Speed: ");
        Serial.print((float)1/loopTime);
        Serial.print(" hz ");
        
        Serial.println();
        
      } else {
        Serial.println(imu + IMU.readEulerHeading() + cc + IMU.readEulerRoll() + cc + IMU.readEulerPitch());
        Serial.println(motor + MotorRevs + cc + filteredMotorVel);
        Serial.println(times + (1/loopTime));
      }
      
      MotorRevsPrevious = MotorRevs; //save last wheel position
    }
}
