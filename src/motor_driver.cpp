/***************************************************************
   Motor driver definitions
   
   Add a "#elif defined" block to this file to include support
   for a particular motor driver.  Then add the appropriate
   #define near the top of the main ROSArduinoBridge.ino file.
   
   *************************************************************/

#include "motor_driver.h"  

void initMotorController() {
  pinMode(LEFT_MOTOR_FORWARD, OUTPUT);
  pinMode(LEFT_MOTOR_BACKWARD, OUTPUT);
  pinMode(RIGHT_MOTOR_FORWARD, OUTPUT);
  pinMode(RIGHT_MOTOR_BACKWARD, OUTPUT);
}

void setMotorSpeed(int i, int spd) {
  unsigned char reverse = 0;

  if (spd < 0)
  {
    spd = -spd;
    reverse = 1;
  }
  if (spd > 255)
    spd = 255;
  
  if (i == LEFT) { 
    if      (reverse == 0) {digitalWrite(LEFT_MOTOR_FORWARD, HIGH); digitalWrite(LEFT_MOTOR_BACKWARD, LOW); analogWrite(LEFT_MOTOR_PWM, spd);}//{ analogWrite(LEFT_MOTOR_FORWARD, spd); analogWrite(LEFT_MOTOR_BACKWARD, 0); }
    else if (reverse == 1) {digitalWrite(LEFT_MOTOR_FORWARD, LOW); digitalWrite(LEFT_MOTOR_BACKWARD, HIGH); analogWrite(LEFT_MOTOR_PWM, spd);}//{ analogWrite(LEFT_MOTOR_BACKWARD, spd); analogWrite(LEFT_MOTOR_FORWARD, 0); }
  }
  else /*if (i == RIGHT) //no need for condition*/ {
    if      (reverse == 0) {digitalWrite(RIGHT_MOTOR_FORWARD, HIGH); digitalWrite(RIGHT_MOTOR_BACKWARD, LOW); analogWrite(RIGHT_MOTOR_PWM, spd);}//{ analogWrite(RIGHT_MOTOR_FORWARD, spd); analogWrite(RIGHT_MOTOR_BACKWARD, 0); }
    else if (reverse == 1) {digitalWrite(RIGHT_MOTOR_FORWARD, LOW); digitalWrite(RIGHT_MOTOR_BACKWARD, HIGH); analogWrite(RIGHT_MOTOR_PWM, spd);}//{ analogWrite(RIGHT_MOTOR_BACKWARD, spd); analogWrite(RIGHT_MOTOR_FORWARD, 0); }
  }
}

void setMotorSpeeds(int leftSpeed, int rightSpeed) {
  setMotorSpeed(LEFT, leftSpeed);
  setMotorSpeed(RIGHT, rightSpeed);
}

long readEncoder(int long& encoderPosition) {
  return encoderPosition;
}

/* Wrap the encoder reset function */
void resetEncoder(long& encoderPosition) {
  encoderPosition = 0;
}

void resetEncoders(long& encoderPosition1, long& encoderPosition2) {
  resetEncoder(encoderPosition1);
  resetEncoder(encoderPosition2);
}
// #endif
