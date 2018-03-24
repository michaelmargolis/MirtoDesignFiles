/* 
 *  Movement.h
 *  High level robot movement functions
 *  
 *  Durtions in milliseconds, power in percent
 *  rotations in degrees per second
 *  
*/ 

#ifndef Movement_h
#define Movement_h

void moveBegin();
int  moveGetDirection();
void moveSetPower(int leftPWM, int rightPWM);
void moveSetSpeed(int cmps, unsigned long duration);
void moveSetSpeeds(int leftCMPS, int rightCMPS, unsigned long duration);
void moveDistance(int distanceCm, int speedCmps);
void moveTimed(int lpwm, int rpwm, unsigned long dur); //duration in milliseconds
void moveSpeedTimed(int lSpeed, int rSpeed, unsigned long dur);
void moveRotate(int dps, unsigned long dur);  // degrees per second, ms
void moveRotateAngle(int dps, int angle);
void moveStop();
void moveShowEncoder();
boolean moveisRampingPwm();
void moveSetKp(int val);
void moveSetKi(int val);
void moveSetKd(int val);

#endif
