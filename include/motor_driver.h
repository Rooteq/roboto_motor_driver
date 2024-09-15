/***************************************************************
   Motor driver function definitions - by James Nugen
   *************************************************************/
#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

#include <Arduino.h>
#include "commands.h"

#define LEFT_MOTOR_FORWARD   14
#define LEFT_MOTOR_BACKWARD  15
#define RIGHT_MOTOR_FORWARD  8
#define RIGHT_MOTOR_BACKWARD 7

#define LEFT_MOTOR_PWM 9
#define RIGHT_MOTOR_PWM 10

#define MAX_PWM        255

void initMotorController();
void setMotorSpeed(int i, int spd);
void setMotorSpeeds(int leftSpeed, int rightSpeed);

void resetEncoders(long& encoderPosition1, long& encoderPosition2);
void resetEncoder(long& encoderPosition);
long readEncoder(long& encoderPosition);

#endif