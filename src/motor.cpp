#include "motor.h"

void Motor::updatePID()
{
    int motorSpeed = pid.updatePID(position, moving);
    setMotorSpeed(motorSpeed);
}

void Motor::setTargetTPF(double tpf)
{
    pid.setTargetTPF(tpf);
}

void Motor::resetPID()
{
    pid.resetPID(position);
}

void Motor::begin()
{
    encoder.begin();
    initMotorController();
}

void Motor::resetEncoder()
{
    position = 0;
}

void Motor::InterruptUpdatePosition()
{
    unsigned char result = encoder.process();
    if (result == DIR_NONE) {
        // do nothing
    }
    else if (result == DIR_CW) {
        position++;
    }
    else if (result == DIR_CCW) {
        position--;
    } 
}

void Motor::setMotorSpeed(int spd) {
    unsigned char reverse = 0;

    if (spd < 0)
    {
        spd = -spd;
        reverse = 1;
    }
    if (spd > 255)
        spd = 255;
    
    if      (reverse == 0) {digitalWrite(FORWARD_PIN, HIGH); digitalWrite(BACKWARD_PIN, LOW); analogWrite(PWM_PIN, spd);}
    else if (reverse == 1) {digitalWrite(FORWARD_PIN, LOW); digitalWrite(BACKWARD_PIN, HIGH); analogWrite(PWM_PIN, spd);}
}

void Motor::initMotorController() {
    pinMode(FORWARD_PIN, OUTPUT);
    pinMode(BACKWARD_PIN, OUTPUT);
}