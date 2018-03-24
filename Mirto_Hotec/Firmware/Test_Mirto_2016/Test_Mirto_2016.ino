/*
 * Test sketch for Mirto-2016 PCB with Teensy 3.2
 * This code tests the following Mirto board components:
 *  LCD 
 *  Neo pixel leds (one onboard and up to 8 external)
 *  Speaker
 *  on-board Potentiometer
 *  on-board LED on Pin 13
 *  Servo connector
 *  Serial comms with Raspberry pi
 *  ESP8266 WiFi board
 *  
 * and the core robot functions:
 *  bump switches
 *  ir sensors
 *  motors
 *  encoders
 *  
 *The onboard tactile switch ends each test and moves to the next
 *The status of each test is reported on the LCD.
 
 * This sketch requires the following external libraries (installable from library manager):
 *  Adafruit Neopixel
 *  U8G2
 */
 
#include "RobotDefines.h"  // global defines
#include "Movement.h"
#include "tinyLcd.h"
#include <Adafruit_NeoPixel.h>

Adafruit_NeoPixel strip = Adafruit_NeoPixel(1, neoPixelPin, NEO_RGB + NEO_KHZ400);

#define MIN_POWER 50 // minimum percent power that enables the robot to move
int speed = MIN_POWER+20; // default speed in percent power when moving along a straight line


void setup()
{
  while(!Serial); // uncomment this to block until serial monitor is  opened
  Serial.begin(57600);  // note teensy Serial runs at full USB speed (12 Mbit/sec)
  Serial1.begin(57600);  // speed should be same as ESP8266
  Serial3.begin(57600);  // speed should be same as Pi
  lcd.begin(); // init the i2c lcd
  Serial.println("starting");  // echo to serial monitor
  Serial3.println("starting"); // echo to the Raspberry Pi
  pinMode(switchPin, INPUT);
  pinMode(ledPin, OUTPUT);
  soundBegin() ;

  strip.begin();
  strip.setBrightness(64); // 1/4 brightness
  //rainbow(8); 
    
  irSensorBegin();    // initialize sensors  
  softServoWrite(90, 250);        // center the servo
  moveBegin();  

   
  for(int sw=0; sw < nbrSwitches; sw++) {     
    pinMode( bumpSwitchPins[sw], INPUT_PULLUP); 
  }
  soundOhhh();

  Serial.println("Push button to start tests");
  lcd.printf("Push button to start"); 
  lcd.printf("Ready for tests"); 
  while(debounce() == LOW) 
       ;
  lcd.clear(); 
  while(debounce() != LOW) // wait for switch to be released
       ;
     
  lcd.printf("Cycling LED colours"); 
  Serial.println("Cycling LED colours");
  strip.begin();  // neopixel init
  showNeoPixels(); 
         
  testBumpAndLED();
  showPotValue();
  servoTest();
  showDistance();
  playTones();
  showIrSensors();
  showEncoders(); 
  esp8266Test();
  lcd.clear(); 
  testPiSerial();
}

void loop()
{
    digitalWrite(irControlPin, HIGH);
    delay(1);    
    int leftVal = analogRead(irSensorPins[SENSE_IR_LEFT]);
    int centerVal = analogRead(irSensorPins[SENSE_IR_CENTER]);
    int rightVal = analogRead(irSensorPins[SENSE_IR_RIGHT]);
    digitalWrite(irControlPin, LOW);  
    Serial.print(leftVal); Serial.print(",");
    Serial.print(centerVal); Serial.print(",");
    Serial.println(rightVal);

    if(digitalRead(bumpSwitchPins[0]) == LOW) {
       moveTimed(0, 50, 1000);
       Serial.println("right bump switch pressed");
    }   
   if(digitalRead(bumpSwitchPins[1]) == LOW){
       moveTimed(50, 0, 1000);
       Serial.println("left bump switch pressed");      
    }             
      
    moveShowEncoder();       
    delay(100);
}


// writes given angle to servo for given delay in milliseconds
void softServoWriteX(int angle, long servoDelay)
{
static int lastAngle = -1;   
  if( angle != lastAngle)
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
    while(servoDelay >=0);
    lastAngle = angle;
  } 
}

void showNeoPixels()
{
  int i, c;

  for(c=0; c<256; c++) {
    for(i=0; i<strip.numPixels(); i++) {
      strip.setPixelColor(i, Wheel((i+c) & 255));
    }
    strip.show();
    delay(20);
  } 
  strip.setBrightness(2);
  strip.show();
}
// Input a value 0 to 255 to get a color value.
// The colours are a transition r - g - b - back to r.
uint32_t Wheel(byte WheelPos) {
  if(WheelPos < 85) {
   return strip.Color(WheelPos * 3, 255 - WheelPos * 3, 0);
  } else if(WheelPos < 170) {
   WheelPos -= 85;
   return strip.Color(255 - WheelPos * 3, 0, WheelPos * 3);
  } else {
   WheelPos -= 170;
   return strip.Color(0, WheelPos * 3, 255 - WheelPos * 3);
  }
}

void playTones()
{
    // press switch to exit
  lcd.clear();    
  lcd.printf("Adjust tone with pot",0);
  lcd.text("Press butten to end",1);
  Serial.println("Adjust tone with pot, press butten to end");
  while(debounce() == LOW) 
  {
     int val   = map(analogRead(trimPotPin),0,1024,200,5000);
     tone(speakerPin, val);
     Serial.print("frequency = "); Serial.println(val); 
     sprintf(_buf, "frequency %d    ",val);
     lcd.text(_buf,3);
     //delay(50);
  }
  noTone(speakerPin);
  Serial.println("release switch");
  lcd.text("release switch",0);
  while(debounce() != LOW )
      ; 
  lcd.clear();  
}

void showPotValue()
{
  // press switch to exit
  while(debounce() == LOW)
  {    
      int val   = map(analogRead(trimPotPin),0,1024,0,100);
      Serial.print("pot percent = "); Serial.println(val);
      lcd.clear();      
      lcd.hGraph("Pot Position", 0,val,0,0 );
      delay(50);
  }
  lcd.text("release switch",0);
  Serial.println("release switch");
  while(debounce() != LOW )
      ; 
  lcd.clear();
}

void servoTest()
{
    // press switch to exit
  lcd.clear();    
  lcd.text("Position servo using pot",0);
  lcd.text("Press butten to end",1);
  Serial.println("Position servo using pot, press butten to end");
  static int prevVal;
  while(debounce() == LOW) 
  {
     int val   = map(analogRead(trimPotPin),0,1024,20,160);     
     if( abs(val-prevVal) > 2) {  //  hysteresis to minimise jitter          
        Serial.print("angle = "); Serial.println(val); 
        sprintf(_buf, "angle = %d    ",val);
        lcd.text(_buf,3);
        softServoWrite(val, 50);     
        prevVal = val;
     }
  } 
  lcd.text("release switch",0);
  Serial.println("release switch");
  while(debounce() != LOW )
      ; 
  lcd.clear();  
}

void showEncoders()
{
  // press board switch to exit
  while(debounce() == LOW )
  {
      int power= map(analogRead(trimPotPin),0,1024,0,100);
      moveTimed(power, power, 1000);
      moveShowEncoder();
/*            
      lcd.clear();                  
      int leftVal  = map(constrain(leftPulse,1000,20000), 20000, 1000,0,100);
      int rightVal = map(constrain(rightPulse,1000,20000), 20000,1000,0,100);
      lcd.hGraph("Encoder pulse widths", 0,leftVal,rightVal,0 );  
*/      
      delay(50);
  }
  lcd.text("release switch",0);
  Serial.println("release switch");
  while( debounce() != LOW )
      ; 
  lcd.clear();
  moveStop();

}

void showIrSensors()
{
  // press both bump switches to exit
  Serial.println("Show IR");
  while( debounce() == LOW )
  {
      digitalWrite(irControlPin, HIGH);
      delay(1);    
      int leftVal   = map(analogRead(irSensorPins[SENSE_IR_LEFT]),  1024,0,0,100);
      int centerVal = map(analogRead(irSensorPins[SENSE_IR_CENTER]),1024,0,0,100);
      int rightVal  = map(analogRead(irSensorPins[SENSE_IR_RIGHT]), 1024,0,0,100);
      digitalWrite(irControlPin, LOW);  
      Serial.print(leftVal); Serial.print(","); Serial.print(centerVal); Serial.print(","); Serial.println(rightVal);
      lcd.clear();      
      lcd.hGraph("Reflectance percent", 0,leftVal,centerVal,rightVal );
      delay(50);
  }
  lcd.text("release switch",0);
  Serial.println("release switch");
  while(debounce() != LOW )
      ; 
  lcd.clear();  
}

void showDistance()
{
  Serial.println("Show Distance");
  while( debounce() == LOW )
  {
      int cm = pingGetDistance(pingTrigPin,pingEchoPin);   
      int val   = map(analogRead(cm),0,175,0,100);
      Serial.print("distance = "); Serial.println(val);
      lcd.clear();      
      lcd.hGraph("Distance Cm", 0,val,0,0 );
      delay(50);
  }
  lcd.text("release switch",0);
  Serial.println("release switch");
  while(debounce() != LOW )
      ; 
  lcd.clear(); 
}

void esp8266Test()
{
  lcd.text("Testing ESP8266",0);
  Serial1.flush();
  Serial1.print("print(wifi.sta.getip())\n");
  Serial1.find("\n"); // ignore print echo
  String ip = Serial1.readStringUntil('\t'); 
  if(ip.length() > 5) {
      lcd.text("IP request returned:",1);
     // TODO  lcd.text(ip.c_str(),2);
      Serial.println(ip.c_str());
  }
  else {
    lcd.text("no reply from ESP8266",1);
  }
  
  while( debounce() == LOW )
  {
      if(Serial1.available())
          Serial.print((char)Serial1.read());
      if(Serial.available())
          Serial1.print((char)Serial.read());          
  }
  lcd.text("release switch",0);
  while(debounce() != LOW )
      ; 
  lcd.clear(); 
   
}

void testBumpAndLED()
{
 lcd.clear();
 lcd.printf("Press Bump Switches",0);
 Serial.println("testing Bump Switchs");
  while( debounce() == LOW )
  {
      if( digitalRead(bumpSwitchPins[SENSE_BUMP_LEFT]) == LOW){
        lcd.printf("left bump pressed");
         delay(200);
      }
      else if( digitalRead(bumpSwitchPins[SENSE_BUMP_RIGHT]) == LOW){
        lcd.printf("right bump pressed");
         delay(200);
      } 
  }
 // lcd.text("release switch",0);
 // Serial.println("release switch");
  while(debounce() != LOW )
      ; 
  lcd.clear();  
}

void testPiSerial()
{
 lcd.text("Testing Pi Serial",0);
  Serial3.println("testing Pi Serial port");
  while( debounce() == LOW )
  {
      if(Serial3.available())
          Serial.print((char)Serial3.read());
      if(Serial.available())
          Serial3.print((char)Serial.read());          
  }
  lcd.text("release switch",0);
  Serial.println("release switch");  
  while(debounce() != LOW )
      ; 
  lcd.clear(); 
}

boolean checkAbort()
{
   return debounce()== HIGH;      
}

// debounce returns the state after a debounce period
boolean debounce()
{
  unsigned long startMillis = millis();
  const int debouncePeriod = 10;  // milliseconds to wait until stable  
  boolean state;
  boolean previousState;

  previousState = digitalRead(switchPin);          // store switch state
  while( millis() - startMillis < debouncePeriod )
  {     
      state = digitalRead(switchPin);  // read the pin
      if( state != previousState)
      {   
         startMillis = millis();  // restart if the state changes      
         previousState = state;  // and save the current state
      }
  }    
  return state;
}


