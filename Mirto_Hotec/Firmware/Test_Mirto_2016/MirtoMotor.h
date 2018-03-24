/* MirtoMotor.h
 * Supports motors using the Toshiba H-Bridge motor driver.
 * such as Mirto v2 board or Hub-ee 
 * This version has experimental PID speed control
 * Michael Margolis July 2016
 */

#ifndef MirtoMotor_h
#define MirtoMotor_h
#include "Arduino.h"

#include <Encoder.h>

enum motorPinIndex {in1Pin,in2Pin,PWMPin,EncApin,EncBpin};

const int PID_FRAME_HZ        = 30;                       //  frames per second  
const int AUTO_STOP_INTERVAL  = 2000; // turn off PWM if no encoder pulses after this many ms

const int NORMAL_DIRECTION    = 1;
const int REVERSED_DIRECTION  = -1;
const int DIR_FORWARD         = 1;
const int DIR_REVERSE         = -1;

const int  MAX_PWM_DELTA     = 80;  // max percent increase in power between intervals 
const int  POWER_RAMP_INTERVAL = 30;  // interval between incriments in ms
  
typedef struct {
  //see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-tuning-changes/
  long targetTicksPerSecond;               // target speed in ticks per second
  long encoder;                            // encoder count
 // long prevEnc;                            // last encoder count
 
 int prevInput;                           // last input 
 int iTerm;                               // integrated term
 int prevOutput;                             // last motor setting 

//  int iError;
//  int prevError;

  
  int encoderPPR;                          // encoder pulses per revolution  
  unsigned long prevPidService;            // time of last PID calculation 
  unsigned long prevPulseMillis;           // time of frame containing most recent encoder pulse (for auto shotoff)
  unsigned long startTime;                 // millis time of setMotorRPM call (for duration timeout)
  unsigned long duration;                  // number of milliseconds to run (-1 runs for 49+ days)
  boolean isActive;                        // true if PID is enabled and motor is moving
  int16_t Kp;
  int16_t Ki;
  int16_t Kd;
  int16_t Ko; // scaling parameter
  unsigned long frameInterval;             // 1000 / frame rate in Hz
} pidInfo;        



class MirtoMotor
{
    public:      
        MirtoMotor(const int pins[], Encoder *encoder);
        //MirtoMotor(int In1Pin, int In2Pin, int PWMPin, int STBYPin);
        void begin(int direction);  
        void setBrakeMode(boolean brakeMode);
        void stopMotor();
        void setStandbyMode(boolean standbyMode);
        void setDirectionMode(int mode);  // 1 normal dir, -1 dir inverted
        int getDirectionMode();
        void setMotorPower(int MPower); // range is -100 to 100               
        void setMotorRPM(float RPM, unsigned long duration); // duration in ms       
        void initPid(int encoderPPR );
        void setPid( int Kp, int Ki, int Kd, int Ko, unsigned long interval);
        void getPid( int &Kp, int &Ki, int &Kd, int &Ko,unsigned long &interval);  
        boolean isPidServiceNeeded();
        boolean isPidActive();
        boolean isRampingPwm(); // returns true if motor coming up to speed
        void setMotorLabel(char *label); // for debug print only
        void servicePid(long count);

        // encoder methods
        long encoderDelta();
        long encoderPos();
        void encoderResetCume();
    
    private:
        void initialise();
        void setMotorPwm(int pwm);
        //int motorPwm;
        int absPwm;
        int prevAbsPwm;
        int motorDirection;  // 1 is forward, -1 is reverse
        int motorDirectionMode;  //1 normal dir, -1 dir inverted
        boolean motorBrakeMode;
        boolean motorStandbyMode;
        //pin assignments: In1, In2, PWM, encoderA, encoderB
        const int *pins;    
        int standbyPin;
        //int maxPWM;
        pidInfo PID; 
        unsigned long prevPwmRampTime;           // time of most increse in PWM to control rate motor comes up to speed
        char *label; // only used for debug print to identify motor
        int powerToPWM(int power);
        Encoder *encoder;
        long prevPos; // previous encoder reading

};
#endif


