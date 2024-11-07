#ifndef COMMANDS_H
#define COMMANDS_H

#include "motor.h"
#include "SPI.h"
#include "mcp2515.h"

// CAN message IDs
#define MOTOR_COMMAND_ID    0x101
#define ENCODER_REQUEST_ID  0x102
#define ENCODER_RESPONSE_ID 0x103

#define AUTO_STOP_INTERVAL 2000

long lastMotorCommand = AUTO_STOP_INTERVAL;
#define CAN_CS_PIN 5

MCP2515 mcp2515(CAN_CS_PIN);

void handleMotorCommand(struct can_frame &frame, Motor& leftMotor, Motor& rightMotor) {
    int16_t motor1 = (frame.data[0] << 8) | frame.data[1];
    int16_t motor2 = (frame.data[2] << 8) | frame.data[3];

    lastMotorCommand = millis();

    if (motor1 == 0 && motor2 == 0) {
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
    leftMotor.setTargetTPF(motor1);
    rightMotor.setTargetTPF(motor2);
}

void handleEncoderRequest(Motor& leftMotor, Motor& rightMotor) {
    struct can_frame frame;

    frame.can_id = ENCODER_RESPONSE_ID;
    frame.can_dlc = 8;  // Now 8 bytes (4 bytes per encoder)
    
    frame.data[0] = (leftMotor.getPosition()>> 24) & 0xFF;  // Highest byte
    frame.data[1] = ( leftMotor.getPosition()>> 16) & 0xFF;
    frame.data[2] = ( leftMotor.getPosition()>> 8) & 0xFF;
    frame.data[3] =  leftMotor.getPosition()& 0xFF;          // Lowest byte
    
    frame.data[4] = ( rightMotor.getPosition()>> 24) & 0xFF;  // Highest byte
    frame.data[5] = ( rightMotor.getPosition()>> 16) & 0xFF;
    frame.data[6] = ( rightMotor.getPosition()>> 8) & 0xFF;
    frame.data[7] =  rightMotor.getPosition()& 0xFF;          // Lowest byte
    
    if(mcp2515.sendMessage(&frame) != MCP2515::ERROR_OK)
    {
      Serial.println("error sending message");
    }
}

void handleCanComms(Motor& leftMotor, Motor& rightMotor)
{
    struct can_frame canMsg;
    
    if (mcp2515.readMessage(&canMsg) == MCP2515::ERROR_OK) {
        switch(canMsg.can_id) {
            case MOTOR_COMMAND_ID:
                handleMotorCommand(canMsg, leftMotor, rightMotor);
                break;
                
            case ENCODER_REQUEST_ID:
                handleEncoderRequest(leftMotor, rightMotor);
                break;
        }
    }
}

#endif