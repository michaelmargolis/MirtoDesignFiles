/*
   code to look for obstacles
*/


// servo angles             left, right, center
const int servoAngles[] = {  150,  30,   90};

static const char  *obstacleStr[] = {"none", "left edge", "right edge", "front edge"};

static int lastObstacle = OBST_NONE;

void lookBegin()
{
  irSensorBegin();    // initialize sensors
  softServoWrite(90, 250);        // center the servo
}

// returns true if the given obstacle is detected
boolean lookForObstacle(int obstacle)
{
  switch (obstacle) {
    case  OBST_FRONT_EDGE: return irEdgeDetect(SENSE_IR_LEFT) && irEdgeDetect(SENSE_IR_RIGHT);
    case  OBST_LEFT_EDGE:  return irEdgeDetect(SENSE_IR_LEFT);
    case  OBST_RIGHT_EDGE: return irEdgeDetect(SENSE_IR_RIGHT);
    case  OBST_LEFT_BUMP:  return digitalRead(bumpSwitchPins[SENSE_BUMP_LEFT]);
    case  OBST_RIGHT_BUMP: return digitalRead(bumpSwitchPins[SENSE_BUMP_RIGHT]);
    case  OBST_FRONT:   int d = lookAt(servoAngles[DIR_CENTER]);  return d > 0 &&  d <= MIN_DISTANCE ;
  }
  return false;
}

// returns the distance of objects at the given angle
int lookAt(int angle)
{
  static int oldAngle = -1;
  if ( angle != oldAngle) {
    softServoWrite(angle, servoDelay ); // wait for servo to get into position
    oldAngle = angle;
  }
  int distance = pingGetDistance(pingTrigPin, pingEchoPin);
  Serial.print(" cm = "); Serial.println(distance);
  if ( angle != servoAngles[DIR_CENTER])
  {
    SerialDebug.print("looking at dir ");
    SerialDebug.print(angle), SerialDebug.print(" distance= ");
    SerialDebug.println(distance);
    softServoWrite(servoAngles[DIR_CENTER], servoDelay / 2);
  }
  return distance;
}

// function to check if robot can continue moving in current direction
// returns true if robot is not blocked moving in current direction
// this version only tests for obstacles in front
boolean checkMovement()
{
  boolean isClear = true; // default return value if no obstacles
  if (moveGetDirection() > 0) // moving forward ?
  {
    if (lookForObstacle(OBST_FRONT) == true)
    {
      isClear = false;
    }
  }
  return isClear;
}


// Look for and avoid obstacles using servo to scan
void roam(int speed, int rotateSpeed)
{
  int distance = lookAt(servoAngles[DIR_CENTER]);
  if (distance == 0)
  {
    moveStop();
    SerialDebug.println("No front sensor");
    return;  // no sensor
  }
  else if (distance <= MIN_DISTANCE)
  {
    moveStop();
    //SerialDisplay.print("Scanning:");
    int leftDistance  = lookAt(servoAngles[DIR_LEFT]);
    if (leftDistance > CLEAR_DISTANCE)  {
      //   SerialDisplay.print(" moving left: ");
      moveRotateAngle(rotateSpeed, -90);
    }
    else {
      delay(500);
      int rightDistance = lookAt(servoAngles[DIR_RIGHT]);
      if (rightDistance > CLEAR_DISTANCE) {
        //  SerialDebug.println(" moving right: ");
        moveRotateAngle(rotateSpeed, 90);
      }
      else {
        // SerialDisplay.print(" no clearence : ");
        distance = max( leftDistance, rightDistance);
        if (distance < CLEAR_DISTANCE / 2) {
          soundSiren(1000);
          moveDistance( -20, 20);  // back up 20 cm
          moveRotateAngle(rotateSpeed, -180); // turn around
        }
        else {
          if (leftDistance > rightDistance)
            moveRotateAngle(rotateSpeed, -90);
          else
            moveRotateAngle(rotateSpeed, 90);
        }
      }
    }
  }
  lookAt(servoAngles[DIR_CENTER]);
  if (moveGetDirection() > 0) // if was moving forward
    moveDistance(1000, speed);  // move up to 10 meters or until stopped
}


bool isOverLine()
{
  digitalWrite(irControlPin, HIGH);
  delay(1);
  int total =  analogRead(irSensorPins[SENSE_IR_LEFT]) + analogRead(irSensorPins[SENSE_IR_CENTER]) + analogRead(irSensorPins[SENSE_IR_RIGHT]);
  //Serial.print("is over line= "); Serial.println(total);
  digitalWrite(irControlPin, LOW);
  return (total > 300) ;
}

// the following is based on loop code from myRobotEdge
// robot checks for edge and moves to avoid
void avoidEdge(int speed, int rotateSpeed)
{
  /// code for roaming around and avoiding obstacles
  if ( lookForObstacle(OBST_FRONT_EDGE) == true)
  {
    SerialDebug.print("both sensors detected edge, last obstacle was ");
    SerialDebug.println(obstacleStr[lastObstacle]);
    lastObstacle = OBST_FRONT_EDGE;
    moveDistance(-20, -20);
    moveRotateAngle(rotateSpeed, 120);
    while (lookForObstacle(OBST_FRONT_EDGE) == true )
      moveStop(); // stop motors if still over cliff
  }
  else if (lookForObstacle(OBST_LEFT_EDGE) == true)
  {
    SerialDebug.print("left sensor detected edge, last obstacle was ");
    SerialDebug.println(obstacleStr[lastObstacle]);
    moveDistance( -20, 20);
    // todo swerve(speed-20, speed, MOV_BACK, 500);
    if (lastObstacle == OBST_RIGHT_EDGE)
      moveRotateAngle(rotateSpeed, 180); // turn around
    else
      moveRotateAngle(rotateSpeed, 45); // turn right
    lastObstacle = OBST_LEFT_EDGE;
  }
  else if (lookForObstacle(OBST_RIGHT_EDGE) == true)
  {
    SerialDebug.print("right sensor detected edge, last obstacle was ");
    SerialDebug.println(obstacleStr[lastObstacle]);
    //      timedMove(MOV_BACK, 200);
    moveDistance( -20, 20);
    // todo swerve(speed, speed-20, MOV_BACK, 500);
    if (lastObstacle == OBST_LEFT_EDGE)
      moveRotateAngle(rotateSpeed, 180); // turn around
    else
      moveRotateAngle(rotateSpeed, -45); // turn left
    lastObstacle = OBST_RIGHT_EDGE;
  }
  else
  {
    //moveSetPower(speed);
    moveDistance(1000, speed);
  }
  delay(1);
}


int cutoffIr = 40;
  
void lineFollowFloat(int LineFollowPower)
{

  static int previousError = 2000;
  static unsigned long previousMillis = 0;   // will store last time PID was updated
  static const int pidInterval = 30;         // milliseconds between readings
  

  int PWR = 50;

  int freq = 35; // frequency of updates;
  int maxDelta = PWR; // max correction

  float Kp = 0.050;
  float Kd = 1.6;
  float Ki = 0.0001;

  float curError = 2000;
  float prevError = 2000;

  float proportional = 0;
  float integral = 0;
  float derivative = 0;
  
  int leftSpeed, rightSpeed;

  int correction = 0;
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= pidInterval) {
    previousMillis = currentMillis;
    digitalWrite(irControlPin, HIGH);
    delay(1);
    int leftVal = constrain(analogRead(irSensorPins[SENSE_IR_LEFT]), cutoffIr, 1000);
    int centerVal = constrain(analogRead(irSensorPins[SENSE_IR_CENTER]), cutoffIr, 1000);
    int rightVal = constrain(analogRead(irSensorPins[SENSE_IR_RIGHT]), cutoffIr, 1000);
    digitalWrite(irControlPin, LOW);

    SerialLink.printf("Line Follow: %d, %d, %d, ",leftVal,centerVal,rightVal);    

    int currentError = computeError(leftVal, centerVal, rightVal, previousError);
    int proportional = currentError - 2000;

    if (proportional == 0) {
      integral = 0;
    } else {
      integral += proportional;
    }

    derivative = proportional - (previousError - 2000);
    previousError = currentError;

    correction = (int) floor(Kp * proportional + Ki * integral + Kd * derivative);
    int delta = constrain(correction, -maxDelta, maxDelta) ;
    if (delta > 0) {       //robot is left of the line
      leftSpeed = LineFollowPower + delta ;
      rightSpeed = LineFollowPower - delta;
      //SerialDebug.printf("Line Follow: Left of line, l=%d, r=%d\n", leftSpeed, rightSpeed);
    } else {                // right of the line
      leftSpeed = LineFollowPower + delta;
      rightSpeed = LineFollowPower - delta ;
      //SerialDebug.printf("Line Follow: Right of line, l=%d, r=%d\n", leftSpeed, rightSpeed);
    }
    leftSpeed = constrain(leftSpeed, 0, 100);
    rightSpeed = constrain(rightSpeed, 0, 100);
    moveSetPower(leftSpeed, rightSpeed);
    SerialLink.printf("%d, %d, ", proportional, delta);
    SerialLink.printf("speeds: %d, %d\n", leftSpeed, rightSpeed);
  }
}

  // PID divisors:
int Kp = 9; // was 10;
int Kd = 60;  // was 60 derivative is multiplied by 16 and divided by this factor
int Ki = 1000;
  
void lineFollow(int LineFollowPower)
{
  int maxDelta = 60; // LineFollowPower / 4; // correction power range plus/minus this percent
  static int previousError = 2000;
  static unsigned long previousMillis = 0;   // will store last time PID was updated
  static const int pidInterval = 15;         // milliseconds between readings
  //static const int cutoffIr = 40;            // the lowest valid reading

  int leftSpeed, rightSpeed;
  int integral, derivative, correction;
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= pidInterval) {
    previousMillis = currentMillis;
    digitalWrite(irControlPin, HIGH);
    delay(1);
    int leftVal = constrain(analogRead(irSensorPins[SENSE_IR_LEFT]), cutoffIr, 1000);
    int centerVal = constrain(analogRead(irSensorPins[SENSE_IR_CENTER]), cutoffIr, 1000);
    int rightVal = constrain(analogRead(irSensorPins[SENSE_IR_RIGHT]), cutoffIr, 1000);
    digitalWrite(irControlPin, LOW);

    SerialLink.printf("Line Follow: %3d, %3d, %3d, ",leftVal,centerVal,rightVal);    

    int currentError = computeError(leftVal, centerVal, rightVal, previousError);
    int proportional = currentError - 2000;

    if (proportional == 0) {
      integral = 0;
    } else {
      integral += proportional;
    }

    derivative = proportional - (previousError - 2000);
    previousError = currentError;

    if (Kd != 0)
      correction =  proportional / Kp  + integral / Ki + (derivative * 16) / Kd;
    else
      correction =  proportional / Kp + integral / Ki;

    int delta = constrain(correction, -100, maxDelta) ; // only speed increase is limited
    if (delta > 0) {       //robot is left of the line
      leftSpeed = LineFollowPower + delta ;
      rightSpeed = LineFollowPower - delta;
      //SerialDebug.printf("Line Follow: Left of line, l=%d, r=%d\n", leftSpeed, rightSpeed);
    } else {                // right of the line
      leftSpeed = LineFollowPower + delta;
      rightSpeed = LineFollowPower - delta ;
      //SerialDebug.printf("Line Follow: Right of line, l=%d, r=%d\n", leftSpeed, rightSpeed);
    }
    leftSpeed = constrain(leftSpeed, 0, 100);
    rightSpeed = constrain(rightSpeed, 0, 100);
    moveSetPower(leftSpeed, rightSpeed);
    SerialLink.printf("%3d, %2d, ", proportional, delta);
    SerialLink.printf("speeds: %2d, %2d\n", leftSpeed, rightSpeed);
  }
}

long computeError(long left, long middle, long right, long previous)
{
  if ( (left == cutoffIr) && (right == cutoffIr) && (middle == cutoffIr) ) {
    return previous;
  } else {
    return ( (middle * 2000 + right * 4000) / (left + middle + right) );
  }
}

int lastAngle = 200; // a value that is outside normal range

// writes given angle to servo for given delay in milliseconds
void softServoWrite(int angle, long servoDelay)
{
  if ( angle != lastAngle)
  {
    pinMode(servoPin, OUTPUT);
    int pulsewidth = map(angle, 0, 180, 544, 2400); // width in microseconds
    do {
      digitalWrite(servoPin, HIGH);
      delayMicroseconds(pulsewidth);
      digitalWrite(servoPin, LOW);
      delay(20); // wait for 20 milliseconds
      servoDelay -= 20;
    }
    while (servoDelay >= 0);
    lastAngle = angle;
  }
  for (int i = 0; i < nbrSwitches; i++) {
    if ( bumpSwitchPins[i] == servoPin) {
      pinMode(servoPin, INPUT_PULLUP);  // restore pin mode if shared with a mbimp switch
      printf("restoring servo pin %d as input\n", servoPin);
      break;
    }
  }
}
