/* Define single-letter commands that will be sent by the PC over the
   serial link.
*/

#ifndef COMMANDS_H
#define COMMANDS_H

#include "motor.h"

#define READ_ENCODERS  'e'
#define MOTOR_SPEEDS   'm'
#define MOTOR_RAW_PWM  'o'
#define RESET_ENCODERS 'r'
#define UPDATE_PID     'u'

#define AUTO_STOP_INTERVAL 2000

long lastMotorCommand = AUTO_STOP_INTERVAL;

int arg = 0;
int index = 0;

char chr;

char cmd;

char argv1[16];
char argv2[16];

long arg1;
long arg2;

void resetCommand() {
  cmd = NULL;
  memset(argv1, 0, sizeof(argv1));
  memset(argv2, 0, sizeof(argv2));
  arg1 = 0;
  arg2 = 0;
  arg = 0;
  index = 0;
}

int runCommand(Motor& leftMotor, Motor& rightMotor) {
  int i = 0;
  char *p = argv1;
  char *str;
  int pid_args[4];
  arg1 = atoi(argv1);
  arg2 = atoi(argv2);
  
  switch(cmd) {
  case READ_ENCODERS:
    Serial.print(leftMotor.getPosition());
    Serial.print(" ");
    Serial.println(rightMotor.getPosition());
    break;
   case RESET_ENCODERS:
    leftMotor.resetEncoder();
    leftMotor.resetPID();
    Serial.println("OK");
    break;
  case MOTOR_SPEEDS:
    lastMotorCommand = millis();
    if (arg1 == 0 && arg2 == 0) {
      leftMotor.setMotorSpeed(0);
      rightMotor.setMotorSpeed(0);
      leftMotor.resetPID();
      rightMotor.resetPID();
      leftMotor.moving = 0;
      rightMotor.moving = 0;
    }
    else
    {
      leftMotor.moving = 1;
      rightMotor.moving = 1;
    };
    leftMotor.setTargetTPF(arg1);
    rightMotor.setTargetTPF(arg2);
    Serial.println("OK"); 
    break;
  case MOTOR_RAW_PWM:
    lastMotorCommand = millis();
    leftMotor.resetPID();
    rightMotor.resetPID();
    leftMotor.moving = 0;
    rightMotor.moving = 0;
    leftMotor.setMotorSpeed(arg1);
    rightMotor.setMotorSpeed(arg2);
    Serial.println("OK"); 
    break;
  // case UPDATE_PID:
  //   while ((str = strtok_r(p, ":", &p)) != '\0') {
  //      pid_args[i] = atoi(str);
  //      i++;
  //   }
  //   Kp = pid_args[0];
  //   Kd = pid_args[1];
  //   Ki = pid_args[2];
  //   Ko = pid_args[3];
  //   Serial.println("OK");
  //   break;
  default:
    Serial.println("Invalid Command");
    break;
  }
}

void handleCommands(Motor& leftMotor, Motor& rightMotor){
  while (Serial.available() > 0) {
    
    chr = Serial.read();

    if (chr == 13) {
      if (arg == 1) argv1[index] = NULL;
      else if (arg == 2) argv2[index] = NULL;
      runCommand(leftMotor, rightMotor);
      resetCommand();
    }
    else if (chr == ' ') {
      if (arg == 0) arg = 1;
      else if (arg == 1)  {
        argv1[index] = NULL;
        arg = 2;
        index = 0;
      }
      continue;
    }
    else {
      if (arg == 0) {
        cmd = chr;
      }
      else if (arg == 1) {
        argv1[index] = chr;
        index++;
      }
      else if (arg == 2) {
        argv2[index] = chr;
        index++;
      }
    }
  }
}  
#endif