/*
 *  distance sensor code   
 */


// Returns the distance in cm
// this will return 0 if no ping sensor is connected or the distance is greater than around 10 feet
int pingGetDistance(int trigPin, int echoPin)
{
  // establish variables for duration of the ping,
  // and the distance result in inches and centimeters:
  long duration, cm;

  pinMode(trigPin, OUTPUT); 
  // The PING))) is triggered by a HIGH pulse of 2 or more microseconds.
  // Give a short LOW pulse beforehand to ensure a clean HIGH pulse:
  digitalWrite(trigPin, LOW);
  delayMicroseconds(5);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  pinMode(echoPin, INPUT); 
  duration = pulseIn(echoPin, HIGH, 10000); // if a pulse does not arrive in 10 ms then the ping sensor is not connected
  if(duration > 10000)
    duration = 10000;
  // convert the time into a distance
  cm = microsecondsToCentimeters(duration);
  //return (cm * 10) / 12 ; // HACK convert cm to inches
  return cm  ; 
}
long microsecondsToCentimeters(long microseconds)
{
  // The speed of sound is 340 m/s or 29 microseconds per centimeter.
  // The ping travels out and back, so to find the distance of the
  // object we take half of the distance travelled.
  return microseconds / 29 / 2;
}
