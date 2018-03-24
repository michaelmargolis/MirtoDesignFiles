// Sound.ino



#define NOTE_C3  131
#define NOTE_CS3 139
#define NOTE_D3  147
#define NOTE_DS3 156
#define NOTE_E3  165
#define NOTE_F3  175
#define NOTE_FS3 185
#define NOTE_G3  196
#define NOTE_GS3 208
#define NOTE_A3  220
#define NOTE_AS3 233
#define NOTE_B3  247
#define NOTE_C4  262
#define NOTE_CS4 277
#define NOTE_D4  294
#define NOTE_DS4 311
#define NOTE_E4  330
#define NOTE_F4  349
#define NOTE_FS4 370
#define NOTE_G4  392
#define NOTE_GS4 415
#define NOTE_A4  440
#define NOTE_AS4 466
#define NOTE_B4  494


int lineTones[] = { NOTE_A3, NOTE_C4, NOTE_C4, NOTE_A3};
int edgeTones[] = {  NOTE_C4, NOTE_A3, NOTE_C4, NOTE_A3};

void soundBegin() 
{
  pinMode(speakerPin, OUTPUT);  
}

void beep (int speakerPin, float noteFrequency, long noteDuration)
{
  int x;
  // Convert the frequency to microseconds
  float microsecondsPerWave = 1000000/noteFrequency;
  // Calculate how many milliseconds there are per HIGH/LOW cycles.
  float millisecondsPerCycle = 1000/(microsecondsPerWave * 2);
  // Multiply noteDuration * number or cycles per millisecond
  float loopTime = noteDuration * millisecondsPerCycle;
  // Play the note for the calculated loopTime.
  for (x=0;x<loopTime;x++)
  {
    digitalWrite(speakerPin,HIGH);
    delayMicroseconds(microsecondsPerWave);
    digitalWrite(speakerPin,LOW);
    delayMicroseconds(microsecondsPerWave);
  }
}

void toneWithDelay(uint8_t _pin, unsigned int frequency, unsigned long duration)
{
  unsigned long startMillis = millis();

  long period = 1000000 / frequency;
  while(millis() - startMillis < duration) {
    digitalWrite(_pin, HIGH);
    delayMicroseconds(period/2);
    digitalWrite(_pin, LOW);
    delayMicroseconds(period/2);
  } 
  delay(duration * 1.30);
}
void sound( int *notes, int count)
{
  // to calculate the note duration, take one second 
  // divided by the note type.
  //e.g. quarter note = 1000 / 4, eighth note = 1000/8, etc.
  int noteDuration = 1000/8;
  for (int thisNote = 0; thisNote < count; thisNote++) {
    toneWithDelay(speakerPin, notes[thisNote], noteDuration);
  }
}

void soundOhhh() {
  for (int i=1000; i<2000; i=i*1.02) { 
    beep(speakerPin,i,10); 
  } 
  for (int i=2000; i>1000; i=i*.98) {
    beep(speakerPin,i,10);
  }
}

void soundUhOh() {
  for (int i=1000; i<1244; i=i*1.01) {
    beep(speakerPin,i,30); 
  }
  delay(200);
  for (int i=1244; i>1108; i=i*.99) {
    beep(speakerPin,i,30);    
  }
  beep(speakerPin,1106,200);  
}


const int FREQ_SKIP = 50;
const int MAX_FREQ = 10000; //max frequency of the siren wail
const int MIN_FREQ = 3700; //min frequency of the siren wail

void soundSiren()
{
  const  int DELAY = 2;
  static int freq = MIN_FREQ;
  static int dir = 1;

  beep(speakerPin, freq, DELAY);
  freq = freq + FREQ_SKIP*dir;
  if(freq >MAX_FREQ)
    dir = -1;
  else if(freq < MIN_FREQ)
    dir = 1; 
}


// sounds siren for the given duration on ms
void soundSiren(unsigned long duration)
{
  const  int DELAY = 2;

  unsigned long siren_start_time = millis();
  int freq;
  do
  {
    for (freq=MIN_FREQ; freq<=MAX_FREQ; freq+=FREQ_SKIP)
    {    
      beep(speakerPin, freq, DELAY);
    }

    for (freq=MAX_FREQ; freq>=MIN_FREQ; freq-=FREQ_SKIP)
    {
      beep(speakerPin, freq, DELAY);
    }
  }
  while (millis() - siren_start_time < duration);
}
