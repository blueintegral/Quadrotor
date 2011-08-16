#include "pins_arduino.h"
#include <Wire.h>

float MotorOutput[]={0, 0, 0, 0};
int RECpins[] = {7, 8, 2, 4};
int MotorPins[] = {5, 6, 9, 10};
int RECtime [4];
int ScaledRECin [4];

float RollPitchAngConst=10;
float RollPitchConst=2.2;
float RPDerivitiveConst=.05;
float YawConst=1.1;
float YawAngConst=0;
float ThrottleConst =5.0;

char buf [100];
volatile byte pos;
volatile boolean process_it;

void setup ()
{
  Serial.begin(9600);
  Wire.begin();

  pinMode(RECpins[0], INPUT);
  pinMode(RECpins[1], INPUT);
  pinMode(RECpins[2], INPUT);
  pinMode(RECpins[3], INPUT);
  pinMode(MotorPins[0], OUTPUT);
  pinMode(MotorPins[1], OUTPUT);
  pinMode(MotorPins[2], OUTPUT);
  pinMode(MotorPins[3], OUTPUT);

  pinMode(MISO, OUTPUT);
  
  SPCR |= _BV(SPE);
  pos = 0;
  process_it = false;
  SPCR |= _BV(SPIE);
}

void loop () {
  if (process_it){
  buf [pos] = 0; 
    Serial.print (int( buf[0]));
    Serial.print("   ");
    Serial.print (int( buf[1]));
    Serial.print("   ");
    Serial.print (int( buf[2]));
    Serial.println (" ");
    
  pos = 0;
  process_it = false;
}
}

















ISR (SPI_STC_vect)
{
byte c = SPDR;  // grab byte from SPI Data Register
  
  // add to buffer if room
  if (pos < sizeof buf)
    {
    buf [pos++] = c;
    
    // example: newline means time to process buffer
    if (c == '\n')
      process_it = true;
    }  // end of room available
}  // end of interrupt routine SPI_STC_vect
  
  

