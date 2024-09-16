#ifndef MOTOR_H
#define MOTOR_H

#include "rotary.h"

class Motor
{
public:
    explicit Motor(const int pin1, const int pin2) 
        : encoder(pin1, pin2), position(0)
    {
        
    }

    void begin()
    {
        encoder.begin();
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

private:
    long position;
    Rotary encoder;
};




#endif