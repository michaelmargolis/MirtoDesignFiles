
/*
 *  IR reflectance sensor code for edge detection and line following  
 */

//pins are defined in the main sketch tab

int irSensorAmbient[nbrIrSensors]; // sensor value with no reflection
int irSensorEdge[nbrIrSensors];    // value considered detecting an edge
boolean isDetected[nbrIrSensors] = {false,false}; // set true if object detected

const int irEdgeThreshold    = 90; // % level above ambient to trigger edge

void irSensorBegin()
{
  pinMode(irControlPin, OUTPUT);
  irSensorsCalibrate();
  digitalWrite(irControlPin, LOW); // turn off (until needed)
}

// calibrate all sensors
void irSensorsCalibrate(){
   for(int sensor = 0; sensor < nbrIrSensors; sensor++)
      irSensorAmbient[sensor] = 0;
  digitalWrite(irControlPin, HIGH);
  delay(1);       
  for(int sensor = 0; sensor < nbrIrSensors; sensor++){
      int ambientVal = 0;       
      for(int i = 0; i < 8; i++ ) // take eight readings for each sensor
          ambientVal +=  analogRead(irSensorPins[sensor]);
      irSensorAmbient[sensor] = ambientVal / 8;
  }
  digitalWrite(irControlPin, LOW);
  // find the lowest value (highest reflection) and store as ambient
  int ambient = 1023;  // default is value for min reflection
  for(int sensor = 0; sensor < nbrIrSensors; sensor++){
     if( irSensorAmbient[sensor] < ambient )
          ambient = irSensorAmbient[sensor] ;   
  }
 // set all sensors to the ambient value
  for(int sensor = 0; sensor < nbrIrSensors; sensor++){
    irSensorAmbient[sensor] = ambient; 
    irSensorEdge[sensor]    = 500; // (ambient * (long)(100+irEdgeThreshold)) / 100;   
    SerialDebug.printf("Sensor %d ambient = %d\n", sensor, ambient);
    SerialDebug.printf("Sensor %d = %d\n", sensor, analogRead(irSensorPins[sensor]));
  }
}

// todo- rotate robot ?
// calibrate for ambient light 
void irSensorCalibrate(byte sensor)
{
  // TODO - rotate robot
  digitalWrite(irControlPin, HIGH);
  delay(1);
  int ambient = analogRead(irSensorPins[sensor]); // get ambient level
  digitalWrite(irControlPin, LOW); 
  irSensorAmbient[sensor] = ambient; 
  // precalculate the levels for object and edge detection  
  irSensorEdge[sensor]    = 600; // was 500 // (ambient * (long)(100+irEdgeThreshold)) / 100; 
}

char const * locationString[] = {"Left", "Right", "Center"}; // labels for debug

boolean irEdgeDetect(int sensor)
{
  boolean result = false; // default value
  digitalWrite(irControlPin, HIGH);
  delay(1);
  int value = analogRead(irSensorPins[sensor]); // get IR light level
  digitalWrite(irControlPin, LOW); 
  if( value >= irSensorEdge[sensor]) {
       result = true; // edge detected (higher value means less reflection)
       if( isDetected[sensor] == false) { // only print on initial detection
         //Serial.print(locationString[sensor]);         
         SerialDebug.printf("%s edge detected\n", locationString[sensor]);
       }
   }
   isDetected[sensor] = result; 
   return result;
}


