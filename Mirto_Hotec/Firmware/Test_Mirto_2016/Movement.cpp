/* 
 *  Movement.cpp
 *  High level robot movement functions
 *  
 *  Durations in milliseconds 
 *  Power in percent (0-100)
 *  Distance in cm
 *  Speed in cm per second
 *  rotations in degrees per second
 *  
*/ 
 
#include "MirtoMotor.h"
#include "Movement.h"
#include "RobotDefines.h"  // global defines
#include "RobotDescription.h"  // for PID

// declare wheel objects
Encoder encoderLeftWheel(leftMotorPins[EncApin], leftMotorPins[EncBpin]);
Encoder encoderRightWheel(rightMotorPins[EncApin], rightMotorPins[EncBpin]);

MirtoMotor leftWheel(leftMotorPins, &encoderLeftWheel);
MirtoMotor rightWheel(rightMotorPins, &encoderRightWheel);

extern boolean checkAbort();

static int moveDirection = 1; // positive if last robot movement was forward, negative if backward

// set PID values for both motors (for testing only??)
void moveSetKp(int val) {
  int Kp, Ki, Kd, Ko;
  unsigned long interval;
  leftWheel.getPid( Kp, Ki, Kd, Ko, interval);
  leftWheel.setPid( val, Ki, Kd, Ko, interval);
  rightWheel.getPid( Kp, Ki, Kd, Ko, interval);
  rightWheel.setPid( val, Ki, Kd, Ko, interval);  
  SerialLink.printf("Kp=%d, Ki=%d, Kd=%d, Ko=%d\n", val, Ki, Kd, Ko);
}

void moveSetKi(int val) {
  int Kp, Ki, Kd, Ko;
  unsigned long interval;
  leftWheel.getPid( Kp, Ki, Kd, Ko, interval);
  leftWheel.setPid( Kp, val, Kd, Ko, interval);
  rightWheel.getPid( Kp, Ki, Kd, Ko, interval);
  rightWheel.setPid( Kp, val, Kd, Ko, interval);  
  SerialLink.printf("Kp=%d, Ki=%d, Kd=%d, Ko=%d\n", Kp, val, Kd, Ko);  
}

void moveSetKd(int val) {
  int Kp, Ki, Kd, Ko;
  unsigned long interval;
  leftWheel.getPid( Kp, Ki, Kd, Ko, interval);
  leftWheel.setPid( Kp, Ki, val, Ko, interval);
  rightWheel.getPid( Kp, Ki, Kd, Ko, interval);
  rightWheel.setPid( Kp, Ki, val, Ko, interval);  
  SerialLink.printf("Kp=%d, Ki=%d, Kd=%d, Ko=%d\n", Kp, Ki, val, Ko);
}

void moveBegin()
{
  leftWheel.setMotorLabel("left ");
  rightWheel.setMotorLabel("right");
  leftWheel.begin(NORMAL_DIRECTION);  // on the 2016 board, the left motor is wired to turn opposite to the right
  rightWheel.begin(NORMAL_DIRECTION);
  leftWheel.setBrakeMode(true); // short motor when stopped
  rightWheel.setBrakeMode(true); 
}

int moveGetDirection()
{
  return moveDirection;  // positive if moving forward, negative if back, zero if stopped or rotating in place
}

void moveSetPower(int leftPower, int rightPower)
{
  leftWheel.setMotorPower(leftPower);
  rightWheel.setMotorPower(rightPower);
  moveDirection = leftPower + rightPower;  
  while( leftWheel.isRampingPwm() || rightWheel.isRampingPwm() ) {
        ;// wait to get up to speed
  }
}

void moveSetSpeed(int cmps, unsigned long duration)
{
  moveSetSpeeds(cmps, cmps, duration);
  moveDirection = cmps;
}

void moveSetSpeeds(int leftCMPS, int rightCMPS, unsigned long duration)
{
  float leftRpm = (60 * leftCMPS) / (WHEEL_CIRCUMFERENCE / 10.0); // circumference is in mm
  float rightRpm = (60 * rightCMPS) / (WHEEL_CIRCUMFERENCE / 10.0); // circumference is in mm
  SerialDebug.printf("set cm per sec to %d, %d, dur=%ld\n", leftCMPS, rightCMPS, duration);
  leftWheel.setMotorRPM(leftRpm, duration);
  rightWheel.setMotorRPM(rightRpm, duration);
  moveDirection = leftCMPS + rightCMPS;
}

void moveDistance(int distanceCm, int speedCmps)
{
  unsigned long startMillis = millis();
  long dur = abs((distanceCm * 1000L) / speedCmps); // dur in ms
  SerialDebug.printf("\nMove Distance: distance= %dcm, speed= %dcm/s, dur= %dms\n",distanceCm, speedCmps, dur);
  moveSetSpeed(speedCmps, dur);


  while ( leftWheel.isPidActive() || rightWheel.isPidActive() ) { // todo  
    if (leftWheel.isPidServiceNeeded())  leftWheel.servicePid(leftWheel.encoderDelta());
    if (rightWheel.isPidServiceNeeded()) rightWheel.servicePid(rightWheel.encoderDelta());
    if(checkAbort())
      break;
  }

  SerialDebug.printf("Motor stopped after %d ms", millis() - startMillis);  
}

void moveTimed(int lPower, int rPower, unsigned long dur)
{
  SerialDebug.printf("in timed move\n");
  unsigned long startMillis = millis();
  SerialDebug.printf("\nTimed Move: lPower=%d, rPower=%d, dur=%d\n",lPower,rPower,dur);
  leftWheel.setMotorPower(lPower);
  rightWheel.setMotorPower(rPower);
  moveDirection = lPower + rPower;

  while (millis() - startMillis < dur) {
    // service pwm ramp up
    leftWheel.isRampingPwm();
    rightWheel.isRampingPwm();
    if( checkAbort())
        break;
  }
  moveStop();
  SerialDebug.printf("Motor stopped after %d ms\n", millis() - startMillis);
}

void moveSpeedTimed(int lSpeed, int rSpeed, unsigned long dur)
{  
  unsigned long startMillis = millis();
  SerialDebug.printf("\nTimed Move at speed: lSpeed=%d, rSpeed=%d, dur=%d\n",lSpeed,rSpeed,dur);
  moveSetSpeeds(lSpeed, rSpeed, dur);   
  while (millis() - startMillis < dur) {
    while ( leftWheel.isPidActive() || rightWheel.isPidActive() ) {
      if (leftWheel.isPidServiceNeeded())  leftWheel.servicePid(leftWheel.encoderDelta());
      if (rightWheel.isPidServiceNeeded()) rightWheel.servicePid(rightWheel.encoderDelta());
      if(checkAbort())
        break;
    }
  }
  moveStop();  
}

// todo fix this
void moveRotate(int dps, unsigned long dur) // degrees per second, duration in milliseconds
{
  int angle = (dps * dur) / 1000;
  SerialDebug.printf("\nmoveRotate: dps= %d, dur=%ld, angle=%d\n", dps, dur, angle);
  moveRotateAngle(dps, angle);  
}

void moveRotateAngle(int dps, int angle)
{
  long dur = abs((angle * 1000L) / dps);  
  int distance = TRACK_CIRCUMFERENCE * angle / 360.0; // distance in mm
  int speed = distance * 100L / dur; // absolute wheel speed in cm per sec 
  dur =  distance *100L / speed; // recalculated because speed in cm/sec is rounded down.
  SerialDebug.printf("\nRotateAngle: dps= %d, angle=%d, dur= %ld, speed=%d cm/sec, distance = %d cm\n", 
      dps,angle, dur, speed, distance); 
  moveSetSpeeds(speed, -speed, dur);   
  while ( leftWheel.isPidActive() || rightWheel.isPidActive() ) {
      if (leftWheel.isPidServiceNeeded())  leftWheel.servicePid(leftWheel.encoderDelta());
      if (rightWheel.isPidServiceNeeded()) rightWheel.servicePid(rightWheel.encoderDelta());
      if(checkAbort())
        break;
  }
  moveStop();  
}

void moveStop()
{
  leftWheel.stopMotor();
  rightWheel.stopMotor();
  moveDirection = 0; 
}

boolean moveisRampingPwm()
{
  return  leftWheel.isRampingPwm() || rightWheel.isRampingPwm() ;
}

void moveShowEncoder()
{
  static long prevMillis;
  if( millis() -33 > prevMillis) {
      long lDelta = leftWheel.encoderDelta();
      long rDelta = rightWheel.encoderDelta();
      if( lDelta != 0 || rDelta != 0 ) {
        prevMillis = millis();
        SerialDebug.printf(" % d: % d, % d: % d\n", lDelta, leftWheel.encoderPos(), rDelta, rightWheel.encoderPos());
      }
  }     
}


