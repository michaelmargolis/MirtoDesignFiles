/*
 * Global defines for constants used by other modules
 */
#ifndef RobotDefines_h 
#define RobotDefines_h 

#define SerialDebug Serial   // Stream to send serial debug messages
#define SerialLink  Serial1  // Stream to send real time messages
#define SerialInput Serial1

#define MIRTO2016  // for Teensy 3.2 2016 board
//#define MIRTO2016_RED_PCB   // define this if using the red prototype PCb (Right encoder pins are swapped)
#define SWAP_MOTORS  // if left and right motors are swapped (board turned around)

#if defined(__AVR_ATmega328P__)
const int MAX_PWM                      = 255;
const int nbrIrSensors                 = 3;
const int irSensorPins[nbrIrSensors]   = {17,15,16}; // note order is: Left,Right,Center
const int irControlPin                 =  14; 
const int nbrSwitches                  = 2;
const int bumpSwitchPins[nbrSwitches]  = {9,10};
const int servoPin                     = 9; // same as left bump sensor (use 2.2k resistor)??
const int pingEchoPin                  = 18; // needs voltage divider on 3.3v Mirto board
const int pingTrigPin                  = 19;
const int leftMotorPins[]              = {8,11,5};
const int rightMotorPins[]             = {12,13,6};
// encoder pins
int wheel_1QeiAPin = 3; //external interrupt 0 used for wheel 1 encoder chanel A
int wheel_1QeiBPin = 7; //wheel 1 encoder chanel B input
int wheel_2QeiAPin = 2; //external interrupt 1 used for wheel 2 encoder chanel A
int wheel_2QeiBPin = 4; //wheel 2 encoder chanel B input
#elif defined(__MK20DX256__) // Teensy 3.1
const int MAX_PWM                      = 255;
#if defined MIRTO2016  // new pcb with pin assignments on 6, 24 and 33 changed
const int nbrIrSensors                 = 3;
const int irSensorPins[nbrIrSensors]   = { A1,A2,A3}; // analog pins, note order is: Left,Right,Center
const int irControlPin                 = 14;  // (A0); 
const int nbrSwitches                  = 2;
const int bumpSwitchPins[nbrSwitches]  = {6,24};
const int servoPin                     = 3;  // was pins 2&3 on proto board
const int pingTrigPin                  = 4; 
const int pingEchoPin                  = 4; // both pins connected together ?
const int neoPixelPin                  = 2; 
const int switchPin                    = 5; 
const int ledPin                       = 13;
// motor pins are: In1, In2, PWM, encoderA, encoderB
#ifdef MIRTO2016_RED_PCB
const int rightMotorPins[]             = {27,30,25, 26,28};  // marked wheel A on board
#else
const int rightMotorPins[]             = {27,30,25, 28,26};  // marked wheel A on board
#endif
const int leftMotorPins[]              = {33,20,32, 29,31};  // marked wheel B on board  

const int speakerPin                   = 9;
const int trimPotPin                   = 7; // pot on analog pin 7
#elif defined MIRTO2015
const int nbrIrSensors                 = 3;
const int irSensorPins[nbrIrSensors]   = { 9,12,13}; // analog pins, note order is: Left,Right,Center
const int irControlPin                 = 22;  // (A8); 
const int nbrSwitches                  = 2;
const int bumpSwitchPins[nbrSwitches]  = {24,33};
const int servoPin                     = 3;  // was pins 2&3 on proto board
const int pingTrigPin                  = 4; 
const int pingEchoPin                  = 4; // both pins connected together ?
const int neoPixelPin                  = 2; 
const int switchPin                    = 5; 
const int ledPin                       = 13;
#ifdef SWAP_MOTORS
// motor pins are: In1, In2, PWM, encoderA, encoderB
const int rightMotorPins[]             = {27,30,32, 26,28};  // marked wheel 1 on board
const int leftMotorPins[]              = {20,33, 25, 31,29};  // marked wheel 2 on board  (6 & 20 swapped in final brd)
#else
const int leftMotorPins[]              = {27,30,32, 26,28}; // marked wheel 1 on board
const int rightMotorPins[]             = {20,6, 25, 31,29};  // marked wheel 2 on board  (6 & 20 swapped in final brd)
#endif
const int speakerPin                   = 9;
const int trimPotPin                   = 7; // pot on analog pin 7
#else
// original teensy board 
const int nbrIrSensors                 = 3;
const int irSensorPins[nbrIrSensors]   = { 17,15,16}; // TODO  note order is: Left,Right,Center
const int irControlPin                 =  14; 
const int nbrSwitches                  = 2;
const int bumpSwitchPins[nbrSwitches]  = {1,2};
const int servoPin                     = 0;
const int pingTrigPin                  = 20; // A6;
const int pingEchoPin                  = 20; // both pins connected together ?
const int leftMotorPins[]              = {11,8,9};
const int rightMotorPins[]             = {6,5,10};
// encoder pins
int wheel_1QeiAPin = 3; //external interrupt 0 used for wheel 1 encoder chanel A
int wheel_1QeiBPin = 7; //wheel 1 encoder chanel B input
int wheel_2QeiAPin = 2; //external interrupt 1 used for wheel 2 encoder chanel A
int wheel_2QeiBPin = 4; //wheel 2 encoder chanel B input
#endif

// encoder pins
/*
const byte wheel_1QeiAPin = 3;  //external interrupt 0 used for wheel 1 encoder channel A
 const byte wheel_1QeiBPin = 7;  //wheel 1 encoder channel B input
 const byte wheel_2QeiAPin = 12; //external interrupt 1 used for wheel 2 encoder channel A
 const byte wheel_2QeiBPin = 4;  //wheel 2 encoder channel B input
 */
#else
#error "This version is for the ATmega328 or Teensy 3.x"
#endif

// defines for left and right wheels
const int WHEEL_LEFT  = 0;
const int WHEEL_RIGHT = 1;

 // defines for locations of sensors
const int SENSE_IR_LEFT   = 0;
const int SENSE_IR_RIGHT  = 1;
const int SENSE_IR_CENTER = 2;  // center is after left and right to avoid sparse arrays for wheels/bump sensors etc.

const int SENSE_BUMP_LEFT  = 0;
const int SENSE_BUMP_RIGHT = 1;

// defines for directions
const int DIR_LEFT   = 0;
const int DIR_RIGHT  = 1;
const int DIR_CENTER = 2; // center is after left and right for consistency with the sensor arrays

// todo - right and center may need to be swapped due to way reflectance sensor is connected ??
#ifdef USE_DEBUG_STRINGS
char const * locationString[] = {"Left", "Right", "Center"}; // labels for debug
char const * obstacleStr[] = {"none","left edge", "right edge", "front edge"};
#endif

// obstacles constants 
const int OBST_NONE       = 0;  // no obstacle detected
const int OBST_LEFT_EDGE  = 1;  // left edge detected 
const int OBST_RIGHT_EDGE = 2;  // right edge detected
const int OBST_FRONT_EDGE = 3;  // edge detect at both left & right sensors
const int OBST_FRONT      = 4;  // obstacle in front
const int OBST_REAR       = 5;  // obstacle behind 
const int OBST_LEFT_BUMP  = 6;
const int OBST_RIGHT_BUMP = 7;

// servo defines
const int sweepServoPin = 9;  // pin connected to servo
const int servoDelay    = 500; // time in ms for servo to move

const int  MIN_DISTANCE = 8;    // robot stops when object is nearer (in inches)
const int  CLEAR_DISTANCE = 24;  // distance in inches considered attractive to move
const int  MAX_DISTANCE = 150;   // the maximum range of the distance sensor

/// move states:
enum        {MOV_LEFT, MOV_RIGHT, MOV_FORWARD, MOV_BACK, MOV_ROTATE, MOV_STOP};
#ifdef USE_DEBUG_STRINGS
char const * states[] = {"Left", "Right", "Forward", "Back", "Rotate", "Stop"};
#endif

// following defines are for structured serial data to be displayed by a java app (ArduinoDataDisplay)
//#define SEND_SERIAL_DATA   // uncomment this to send data to serial monitor
#ifdef SEND_SERIAL_DATA
enum { DATA_start, DATA_LEFT, DATA_CENTER, DATA_RIGHT, DATA_DRIFT, DATA_DRIFT1, DATA_L_SPEED, DATA_R_SPEED,
       DATA_P_FACTOR, DATA_I_FACTOR, DATA_D_FACTOR, DATA_nbrItems};

char const * labels[] =   { "", "Left Line", "Center Line", "Right Line", "Error",    "Delta", "Left Speed", "Right Speed", "Kp", "Ki", "Kd"};
int minRange[] =    { 0,          0,             0,            0,      -1023,  -100,      0,           0,             0 ,  0,    0};
int maxRange[] =    { 0,        1023,         1023,         1023,       1023,   100,     100,         100,           50, 1000,  50};

#endif 
#endif // ifndef RobotDefines_h

