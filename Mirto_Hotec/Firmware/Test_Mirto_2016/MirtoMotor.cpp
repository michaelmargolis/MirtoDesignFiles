/* MirtoMotor.cpp
 * Supports motors using Toshiba H-Bridge motor driver ICs.
 * such as Mirto v2 board or Hub-ee 
 * This version has PID speed control
 * Michael Margolis July 2016
 */


#include "MirtoMotor.h" 
#include "robotDefines.h"  // for pins
#include "RobotDescription.h"  // for wheel circumference
#include "RobotConfig.h"

#define SerialDebug Serial   // Stream to send serial debug messages
#define SerialLink  Serial1  // Stream to send real time messages


MirtoMotor::MirtoMotor(const int pins[], Encoder *encoder)
{
   this->pins = pins;  
   this->encoder = encoder;
}

void MirtoMotor::begin(int direction)
{
  pinMode(pins[in1Pin], OUTPUT);
  pinMode(pins[in2Pin], OUTPUT);
  //pinMode(standbyPin, OUTPUT);
  motorDirectionMode = direction ; 
  absPwm = prevAbsPwm = 0;    
  motorDirection = DIR_FORWARD; 
  motorBrakeMode = false;  // freewheel
  motorStandbyMode = 0;   
  initPid(ENCODER_TICKS_PER_WHEEL_REV );
  setMotorPwm(0);
}

void MirtoMotor::setMotorLabel(char *labelString) // for debug print only
{
   label = labelString;
}

void MirtoMotor::setBrakeMode(boolean brakeMode)
{
  // true shorts motor when stopped, false freewheels
  motorBrakeMode = brakeMode;
}

void MirtoMotor::setDirectionMode(int directionMode)
{
  //set relationship between motor direction and forward movement   
   motorDirectionMode = directionMode;    
   SerialDebug.printf("set direction mode to %d\n", directionMode);   
}

int MirtoMotor::getDirectionMode()
{
  //returns the motor direction mode
  return motorDirectionMode;
}

inline int MirtoMotor::powerToPWM(int power) 
{
  // maps power from range of +-percent to +-MAX_PWM
  power = constrain(power, -100, 100);
  return map(power, -100, 100, -MAX_PWM, MAX_PWM);
}

void MirtoMotor::stopMotor()
{
   setMotorPwm(0);
  //stop the motor using the current braking mode 
  digitalWrite(pins[in1Pin], motorBrakeMode);
  digitalWrite(pins[in2Pin], motorBrakeMode);
  PID.isActive = false;
  prevAbsPwm = 0;
  //SerialDebug.printf("%s stopMotor\n", label);
}


void MirtoMotor::setStandbyMode(boolean standbyMode)
{
  //set the standby mode if a standby pin has been assigned.
  //invert the value because LOW activates standby on the IC
  if( standbyPin >= 0)
  {
    digitalWrite(standbyPin, !motorStandbyMode);
  }
}

void MirtoMotor::setMotorPower(int power)
{  
  int pwm = powerToPWM(power);  
  setMotorPwm( pwm ); 
}

void MirtoMotor::setMotorPwm(int pwm)
{
  //set the motor - private function 

  // SerialDebug.printf("\nStart setMotor, %s pwm request is %d,  prevAbsPwm = %d\n", label, pwm, prevAbsPwm);
  if( pwm >= 0)
     motorDirection = NORMAL_DIRECTION;
  else
     motorDirection = REVERSED_DIRECTION;   
 
  absPwm = abs(constrain(pwm, -MAX_PWM, MAX_PWM) );  

  digitalWrite(pins[in1Pin],  ! (motorDirection * motorDirectionMode  == 1) );
  //invert the direction for the second control line
  digitalWrite(pins[in2Pin], (motorDirection * motorDirectionMode == 1) );
  //SerialDebug.printf("in setMotor: %s pin %d is %d, %d is %d\n", label, pins[in1Pin], digitalRead(pins[in1Pin]),pins[in2Pin], digitalRead(pins[in2Pin])  );     
  isRampingPwm();  // control power ramp  
  //SerialDebug.printf("in setMotor: %s absPwm=%d, dir=%d, dir mode=%d, dir mask=%d\n", label, absPwm, motorDirection, motorDirectionMode, (motorDirection * motorDirectionMode ==1));     
}

/*
 * isRamping returns true if the current motor pwm is at least as much as the target PWM
 */
boolean MirtoMotor::isRampingPwm() // returns true if motor coming up to speed
{
  if( prevAbsPwm >= absPwm){
    //SerialDebug.printf("in isRampingPwm, %s writing %d but returning false because prevAbsPwm (%d) is >=  absPwm (%d)\n", label, absPwm,  prevAbsPwm, absPwm);
    analogWrite(pins[PWMPin], absPwm);
    return false;  // motor is getting requested pwm level
  }
  else if( millis() - prevPwmRampTime >= POWER_RAMP_INTERVAL){   
   // increase power at controlled rate   
   //SerialDebug.printf("%s PWM ramp from %d ", label, prevAbsPwm);
   prevAbsPwm += MAX_PWM_DELTA;
   if( prevAbsPwm > absPwm){
         prevAbsPwm = absPwm;
   }
   analogWrite(pins[PWMPin], prevAbsPwm);
   //SerialDebug.printf("to %d\n", prevAbsPwm);
   prevPwmRampTime = millis();
  }   
  return true;
}
   

void MirtoMotor::setMotorRPM(float RPM, unsigned long duration) 
{ 

 // PID.output = 0;  // todo check if this is needed  

  PID.targetTicksPerSecond = long((RPM * PID.encoderPPR) / 60 );  
  PID.isActive = abs(RPM) > 0.1;
  PID.startTime = PID.prevPulseMillis = PID.prevPidService = millis();
  PID.duration = duration;  
  PID.prevOutput = PID.prevInput = 0;
  SerialDebug.printf(label);    
  RPM < 0 ?  SerialDebug.printf(" reversed"): SerialDebug.printf(" forward");     
  SerialDebug.printf(", targetTicksPerSecond set to %d, is Active= %d, dur=%d\n", PID.targetTicksPerSecond,PID.isActive, duration );
  SerialDebug.printf("Direction mode is %d\n", motorDirectionMode);  
  
}

void  MirtoMotor::initPid(int encoderPPR )
{
  PID.targetTicksPerSecond = 0;  
  
  PID.prevOutput = PID.prevInput = 0;
  PID.encoderPPR = encoderPPR;
  // set values from eeprom
  pidCfg_t *pidConfigPtr  = getPidConfig();
  PID.Kp = pidConfigPtr->Kp; // 20;   5
  PID.Kd = pidConfigPtr->Kd; // 12;
  PID.Ki = pidConfigPtr->Ki; // 0;
  PID.Ko = pidConfigPtr->Ko;  // 20;
  PID.frameInterval = 1000/PID_FRAME_HZ;
  //SerialDebug.printf("PID vals: %d, %d, %d, %d\n", PID.Kp, PID.Kd, PID.Ki, PID.Ko );  
}

void MirtoMotor::setPid( int Kp, int Ki, int Kd, int Ko, unsigned long interval)
{
  PID.Kp = Kp;
  PID.Ki = Ki;   
  PID.Kd = Kd;
  PID.Ko = Ko;
  PID.frameInterval =  interval;
  SerialDebug.printf("(todo - save config here) PID vals: %d, %d, %d, %d\n", PID.Kp, PID.Kd, PID.Ki, PID.Ko ); 
}

void MirtoMotor::getPid( int &Kp, int &Ki, int &Kd, int &Ko, unsigned long &interval)
{
  Kp = PID.Kp;
  Ki = PID.Ki;   
  Kd = PID.Kd;
  Ko = PID.Ko;
  interval = PID.frameInterval;
}

boolean MirtoMotor::isPidServiceNeeded()
{
  return( PID.isActive  && (millis() - PID.prevPidService >= PID.frameInterval))  ;
}


boolean MirtoMotor::isPidActive()
{
  return PID.isActive;
}

void  MirtoMotor::servicePid(long encoderCount)
{
long pError;
long pid;
long input;
int output;
unsigned long timeDelta;

  if(!PID.isActive) {      
    stopMotor();     
    return;
  } 
  unsigned long timeNow = millis();
  if( timeNow - PID.startTime > PID.duration) {       
        SerialDebug.printf(" stopped- duration achieved\n"); 
        stopMotor(); 
        return;  
  } 
  if( isPidServiceNeeded() ) {     
     timeDelta = timeNow - PID.prevPidService;
     PID.prevPidService = timeNow;
  
    input = encoderCount; 
    if( input != 0) {
       PID.prevPulseMillis = timeNow;
    }
    else if( timeNow - PID.prevPulseMillis > AUTO_STOP_INTERVAL) {       
        SerialDebug.printf("\nauto stop\n"); 
        stopMotor();   
    }  
    long targetTicks = map(timeDelta, 0, 1000, 0, PID.targetTicksPerSecond);
    pError = targetTicks - input;
    pid = (PID.Kp * pError - PID.Kd * (input - PID.prevInput) + PID.iTerm) / PID.Ko; 

    // acceleration limit
    if( pid > MAX_PWM_DELTA)
       pid = MAX_PWM_DELTA;
    else if (pid < -MAX_PWM_DELTA)
       pid = -MAX_PWM_DELTA;
       
    output = pid + PID.prevOutput;
   
    // Accumulate Integral error *or* Limit output.
    // Stop accumulating when output saturates
    if (output >= MAX_PWM)
      output = MAX_PWM;
    else if (output <= -MAX_PWM)
      output = -MAX_PWM;
    else
      PID.iTerm += PID.Ki * pError;

    PID.prevOutput = output;
    PID.prevInput = input;
       
    setMotorPwm(output);    
    SerialDebug.printf("%s svc after %dms, in=%d, err=%d, out=%d\n", label, timeDelta, input, pError, output);
  } 
  else {
    SerialDebug.printf("!ERROR - PID service called too soon, check isPidServiceNeeded() method before calling\n");
    delay(500);
  }
}

/*
 *  Encoder code *  
 */


long MirtoMotor::encoderDelta()
{  
   long pos = encoderPos();
   long delta = pos - prevPos;
   prevPos = pos;
   return delta;  
}

long MirtoMotor::encoderPos()
{
  return encoder->read() * motorDirectionMode;
}

void MirtoMotor::encoderResetCume()
{
    encoder->write(0);
}

