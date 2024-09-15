
#define USE_BASE      // Enable the base controller code
//#undef USE_BASE     // Disable the base controller code

/* Define the motor controller and encoder library you are using */
#ifdef USE_BASE
   #define ARDUINO_ENC_COUNTER

   /* L298 Motor driver*/
   #define L298_MOTOR_DRIVER
#endif

/* Serial port baud rate */
#define BAUDRATE     57600

/* Maximum PWM signal */

// #if defined(ARDUINO) && ARDUINO >= 100
/* Include definition of serial commands */
#include <rotary.h>

/* Motor driver function definitions */

/* PID parameters and functions */
#include "diff_controller.h"

/* Run the PID loop at 30 times per second */
#define PID_RATE           30     // Hz

/* Convert the rate into an interval */
const int PID_INTERVAL = 1000 / PID_RATE;

/* Track the next time we make a PID calculation */
unsigned long nextPID = PID_INTERVAL;

/* Stop the robot if it hasn't received a movement command
  in this number of milliseconds */
#define AUTO_STOP_INTERVAL 2000
long lastMotorCommand = AUTO_STOP_INTERVAL;

#define LEFT_ENC_PIN_A 2  //pin 2
#define LEFT_ENC_PIN_B 3  //pin 3

// //below can be changed, but should be PORTC pins
#define RIGHT_ENC_PIN_A 11  //pin A4
#define RIGHT_ENC_PIN_B 12   //pin A5

/* Variable initialization */

// A pair of varibles to help parse serial commands (thanks Fergs)
int arg = 0;
int index = 0;

// Variable to hold an input character
char chr;

// Variable to hold the current single-character command
char cmd;

// Character arrays to hold the first and second arguments
char argv1[16];
char argv2[16];

// The arguments converted to integers
long arg1;
long arg2;

Rotary leftEnc = Rotary(LEFT_ENC_PIN_A, LEFT_ENC_PIN_B);
Rotary rightEnc = Rotary(RIGHT_ENC_PIN_A, RIGHT_ENC_PIN_B);

/* Clear the current command parameters */
void resetCommand() {
  cmd = NULL;
  memset(argv1, 0, sizeof(argv1));
  memset(argv2, 0, sizeof(argv2));
  arg1 = 0;
  arg2 = 0;
  arg = 0;
  index = 0;
}

/* Run a command.  Commands are defined in commands.h */
int runCommand() {
  int i = 0;
  char *p = argv1;
  char *str;
  int pid_args[4];
  arg1 = atoi(argv1);
  arg2 = atoi(argv2);
  
  switch(cmd) {
  case GET_BAUDRATE:
    Serial.println(BAUDRATE);
    break;
  case ANALOG_READ:
    Serial.println(analogRead(arg1));
    break;
  case DIGITAL_READ:
    Serial.println(digitalRead(arg1));
    break;
  case ANALOG_WRITE:
    analogWrite(arg1, arg2);
    Serial.println("OK"); 
    break;
  case DIGITAL_WRITE:
    if (arg2 == 0) digitalWrite(arg1, LOW);
    else if (arg2 == 1) digitalWrite(arg1, HIGH);
    Serial.println("OK"); 
    break;
  case PIN_MODE:
    if (arg2 == 0) pinMode(arg1, INPUT);
    else if (arg2 == 1) pinMode(arg1, OUTPUT);
    Serial.println("OK");
    break;
    
#ifdef USE_BASE
  case READ_ENCODERS:
    Serial.print(leftEnc.position);
    Serial.print(" ");
    Serial.println(rightEnc.position);
    break;
   case RESET_ENCODERS:
    resetEncoders(leftEnc.position, rightEnc.position);
    resetPID(leftEnc.position, rightEnc.position);
    Serial.println("OK");
    break;
  case MOTOR_SPEEDS:
    /* Reset the auto stop timer */
    lastMotorCommand = millis();
    if (arg1 == 0 && arg2 == 0) {
      setMotorSpeeds(0, 0);
      resetPID(leftEnc.position, rightEnc.position);
      moving = 0;
    }
    else moving = 1;
    leftPID.TargetTicksPerFrame = arg1;
    rightPID.TargetTicksPerFrame = arg2;
    Serial.println("OK"); 
    break;
  case MOTOR_RAW_PWM:
    /* Reset the auto stop timer */
    lastMotorCommand = millis();
    resetPID(leftEnc.position, rightEnc.position);
    moving = 0; // Sneaky way to temporarily disable the PID
    setMotorSpeeds(arg1, arg2);
    Serial.println("OK"); 
    break;
  case UPDATE_PID:
    while ((str = strtok_r(p, ":", &p)) != '\0') {
       pid_args[i] = atoi(str);
       i++;
    }
    Kp = pid_args[0];
    Kd = pid_args[1];
    Ki = pid_args[2];
    Ko = pid_args[3];
    Serial.println("OK");
    break;
#endif
  default:
    Serial.println("Invalid Command");
    break;
  }
}


void setup() {

  Serial.begin(BAUDRATE);
  
  leftEnc.begin();
  rightEnc.begin();

  PCMSK0 |= (1 << PCINT3)|(1 << PCINT4);
  PCMSK2 |= (1 << LEFT_ENC_PIN_A)|(1 << LEFT_ENC_PIN_B);
  PCICR |= (1 << PCIE0) | (1 << PCIE2);

  sei();
  
  initMotorController();
  resetPID(leftEnc.position, rightEnc.position);
}

void loop() 
{
  while (Serial.available() > 0) {
    
    // Read the next character
    chr = Serial.read();

    // Terminate a command with a CR
    if (chr == 13) {
      if (arg == 1) argv1[index] = NULL;
      else if (arg == 2) argv2[index] = NULL;
      runCommand();
      resetCommand();
    }
    // Use spaces to delimit parts of the command
    else if (chr == ' ') {
      // Step through the arguments
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
        // The first arg is the single-letter command
        cmd = chr;
      }
      else if (arg == 1) {
        // Subsequent arguments can be more than one character
        argv1[index] = chr;
        index++;
      }
      else if (arg == 2) {
        argv2[index] = chr;
        index++;
      }
    }
  }
  
  if (millis() > nextPID) {
    updatePID(leftEnc.position, rightEnc.position);
    nextPID += PID_INTERVAL;
  }
  
  // Check to see if we have exceeded the auto-stop interval
  if ((millis() - lastMotorCommand) > AUTO_STOP_INTERVAL) {;
    setMotorSpeeds(0, 0);
    moving = 0;
  }

}

ISR(PCINT2_vect) {
  unsigned char result = leftEnc.process();
  if (result == DIR_NONE) {
    // do nothing
  }
  else if (result == DIR_CW) {
    leftEnc.position++;
  }
  else if (result == DIR_CCW) {
    leftEnc.position--;
  }
}
ISR(PCINT0_vect) {
  unsigned char result = rightEnc.process();
  if (result == DIR_NONE) {
    // do nothing
  }
  else if (result == DIR_CW) {
    rightEnc.position++;
  }
  else if (result == DIR_CCW) {
    rightEnc.position--;
  }
}
