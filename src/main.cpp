
#include "motor.h"
#include "commands.h"

#define BAUDRATE     57600
#define PID_RATE     30 

const int PID_INTERVAL = 1000 / PID_RATE;
unsigned long nextPID = PID_INTERVAL;

#define LEFT_ENC_PIN_A 2
#define LEFT_ENC_PIN_B 3 

#define RIGHT_ENC_PIN_A 18 
#define RIGHT_ENC_PIN_B 19  

#define LEFT_MOTOR_FORWARD   14
#define LEFT_MOTOR_BACKWARD  15
#define RIGHT_MOTOR_FORWARD  8
#define RIGHT_MOTOR_BACKWARD 7

#define LEFT_MOTOR_PWM 9
#define RIGHT_MOTOR_PWM 10

Motor leftMotor = Motor(LEFT_ENC_PIN_A, LEFT_ENC_PIN_B, LEFT_MOTOR_FORWARD, LEFT_MOTOR_BACKWARD, LEFT_MOTOR_PWM);
Motor rightMotor = Motor(RIGHT_ENC_PIN_A, RIGHT_ENC_PIN_B, RIGHT_MOTOR_FORWARD, RIGHT_MOTOR_BACKWARD, RIGHT_MOTOR_PWM);

void setup() {

  Serial.begin(BAUDRATE);
  
  leftMotor.begin();
  rightMotor.begin();

  PCMSK0 |= (1 << PCINT3)|(1 << PCINT4);
  PCMSK2 |= (1 << LEFT_ENC_PIN_A)|(1 << LEFT_ENC_PIN_B);
  PCICR |= (1 << PCIE0) | (1 << PCIE2);

  sei();
  
  leftMotor.resetPID();
  rightMotor.resetPID();

  SPI.begin();
  
  mcp2515.reset();
  mcp2515.setBitrate(CAN_500KBPS, MCP_8MHZ);
  mcp2515.setNormalMode();
  
  Serial.println("MCP2515 Initialized");
}

void loop() 
{

  handleCanComms(leftMotor, rightMotor);

  if (millis() > nextPID) {
    leftMotor.updatePID();
    rightMotor.updatePID();
    nextPID += PID_INTERVAL;
  }
  
  if ((millis() - lastMotorCommand) > AUTO_STOP_INTERVAL) {;
    leftMotor.setMotorSpeed(0);
    rightMotor.setMotorSpeed(0);
    leftMotor.moving = 0;
    rightMotor.moving = 0;
  }
}

ISR(PCINT2_vect) {
  leftMotor.InterruptUpdatePosition();
}
ISR(PCINT0_vect) {
  rightMotor.InterruptUpdatePosition();
}