#ifndef MOTOR_H
#define MOTOR_H

#include "rotary.h"

#define MAX_PWM 255
/* PID setpoint info For a Motor */
typedef struct {
  double TargetTicksPerFrame;    // target speed in ticks per frame
  long Encoder;                  // encoder count
  long PrevEnc;                  // last encoder count

  /*
  * Using previous input (PrevInput) instead of PrevError to avoid derivative kick,
  * see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-derivative-kick/
  */
  int PrevInput;                // last input
  //int PrevErr;                   // last error

  /*
  * Using integrated term (ITerm) instead of integrated error (Ierror),
  * to allow tuning changes,
  * see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-tuning-changes/
  */
  //int Ierror;
  int ITerm;                    //integrated term

  long output;                    // last motor setting
}
SetPointInfo;


class Motor
{
public:
    explicit Motor(const int pin1, const int pin2, const int MOTOR_FOWRAWD, const int MOTOR_BACKWARD, const int MOTOR_PWM) 
        : encoder(pin1, pin2), FORWARD_PIN(MOTOR_FOWRAWD), BACKWARD_PIN(MOTOR_BACKWARD), PWM_PIN(MOTOR_PWM)
    {
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

    void resetPID(){
        dataPID.TargetTicksPerFrame = 0.0;
        dataPID.Encoder = position;
        dataPID.PrevEnc = dataPID.Encoder;
        dataPID.output = 0;
        dataPID.PrevInput = 0;
        dataPID.ITerm = 0;
    }

    /* PID routine to compute the next motor commands */
    void doPID(SetPointInfo * p) {
        long Perror;
        long output;
        int input;

        //Perror = p->TargetTicksPerFrame - (p->Encoder - p->PrevEnc);
        input = p->Encoder - p->PrevEnc;
        Perror = p->TargetTicksPerFrame - input;


        /*
        * Avoid derivative kick and allow tuning changes,
        * see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-derivative-kick/
        * see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-tuning-changes/
        */
        //output = (Kp * Perror + Kd * (Perror - p->PrevErr) + Ki * p->Ierror) / Ko;
        // p->PrevErr = Perror;
        output = (Kp * Perror - Kd * (input - p->PrevInput) + p->ITerm) / Ko;
        p->PrevEnc = p->Encoder;

        output += p->output;
        // Accumulate Integral error *or* Limit output.
        // Stop accumulating when output saturates
        if (output >= MAX_PWM)
            output = MAX_PWM;
        else if (output <= -MAX_PWM)
            output = -MAX_PWM;
        else
        /*
        * allow turning changes, see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-tuning-changes/
        */
            p->ITerm += Ki * Perror;

        p->output = output;
        p->PrevInput = input;
        }

        /* Read the encoder values and call the PID routine */
        void updatePID() {
        /* Read the encoders */
        dataPID.Encoder = position;        
        /* If we're not moving there is nothing more to do */
        if (!moving){
            /*
            * Reset PIDs once, to prevent startup spikes,
            * see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-initialization/
            * PrevInput is considered a good proxy to detect
            * whether reset has already happened
            */
            if (dataPID.PrevInput != 0) resetPID();
            return;
        }

        /* Compute PID update for each motor */
        doPID(&dataPID);

        /* Set the motor speeds accordingly */
        // return {dataPID.output};
        setMotorSpeed(dataPID.output);
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
    SetPointInfo dataPID, rightPID;
private:
    long position;
    Rotary encoder;

    /* PID Parameters */
    int Kp = 20;
    int Kd = 12;
    int Ki = 0;
    int Ko = 50;


    const int FORWARD_PIN;
    const int BACKWARD_PIN;
    const int PWM_PIN;
};




#endif