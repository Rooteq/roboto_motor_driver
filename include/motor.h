#ifndef MOTOR_H
#define MOTOR_H

#include "rotary.h"
#include "pid.h"

class Motor
{
public:
    explicit Motor(const int pin1, const int pin2, const int MOTOR_FOWRAWD, const int MOTOR_BACKWARD, const int MOTOR_PWM) 
        : encoder(pin1, pin2), FORWARD_PIN(MOTOR_FOWRAWD), BACKWARD_PIN(MOTOR_BACKWARD), PWM_PIN(MOTOR_PWM)
    {
    }

    void updatePID()
    {
        int motorSpeed = pid.updatePID(position, moving);
        setMotorSpeed(motorSpeed);
    }

    void setTargetTPF(double tpf)
    {
        pid.setTargetTPF(tpf);
    }

    void resetPID()
    {
        pid.resetPID(position);
    }

    void begin()
    {
        encoder.begin();
        initMotorController();
    }

    void resetEncoder()
    {
        position = 0;
    }

    void InterruptUpdatePosition()
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





    inline long& getPosition() {return position;}

    void setMotorSpeed(int spd) {
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

        // if (i == LEFT) { 
        //     if      (reverse == 0) {digitalWrite(LEFT_MOTOR_FORWARD, HIGH); digitalWrite(LEFT_MOTOR_BACKWARD, LOW); analogWrite(LEFT_MOTOR_PWM, spd);}//{ analogWrite(LEFT_MOTOR_FORWARD, spd); analogWrite(LEFT_MOTOR_BACKWARD, 0); }
        //     else if (reverse == 1) {digitalWrite(LEFT_MOTOR_FORWARD, LOW); digitalWrite(LEFT_MOTOR_BACKWARD, HIGH); analogWrite(LEFT_MOTOR_PWM, spd);}//{ analogWrite(LEFT_MOTOR_BACKWARD, spd); analogWrite(LEFT_MOTOR_FORWARD, 0); }
        // }
        // else /*if (i == RIGHT) //no need for condition*/ {
        //     if      (reverse == 0) {digitalWrite(RIGHT_MOTOR_FORWARD, HIGH); digitalWrite(RIGHT_MOTOR_BACKWARD, LOW); analogWrite(RIGHT_MOTOR_PWM, spd);}//{ analogWrite(RIGHT_MOTOR_FORWARD, spd); analogWrite(RIGHT_MOTOR_BACKWARD, 0); }
        //     else if (reverse == 1) {digitalWrite(RIGHT_MOTOR_FORWARD, LOW); digitalWrite(RIGHT_MOTOR_BACKWARD, HIGH); analogWrite(RIGHT_MOTOR_PWM, spd);}//{ analogWrite(RIGHT_MOTOR_BACKWARD, spd); analogWrite(RIGHT_MOTOR_FORWARD, 0); }
        // }
    }

    void initMotorController() {
        pinMode(FORWARD_PIN, OUTPUT);
        pinMode(BACKWARD_PIN, OUTPUT);
    }

    unsigned char moving = 0; // is the base in motion?
private:
    long position;
    Rotary encoder;
    PID pid;

    const int FORWARD_PIN;
    const int BACKWARD_PIN;
    const int PWM_PIN;
};




#endif