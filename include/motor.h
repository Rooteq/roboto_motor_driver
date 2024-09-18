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

    void updatePID();
    void setTargetTPF(double tpf);
    void resetPID();
    void begin();
    void resetEncoder();
    void InterruptUpdatePosition();
    inline long& getPosition() {return position;}
    void setMotorSpeed(int spd);
    void initMotorController(); 

    unsigned char moving = 0;
private:
    long position;
    Rotary encoder;
    PID pid;

    const int FORWARD_PIN;
    const int BACKWARD_PIN;
    const int PWM_PIN;
};




#endif