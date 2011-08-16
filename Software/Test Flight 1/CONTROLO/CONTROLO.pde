//for slave core
// Written by Nick Gammon
// February 2011


#include "pins_arduino.h"
#include <Wire.h>

const int Accel_Address (0x53);
const int Gyro_Address (0x68);
const int Num_Bytes_Accel (6);
const int Num_Bytes_Gyro (8);

int X_Offset=-43;
int Y_Offset=14;
int Z_Offset=7;
char str[512];

int Accel[3];
float NormAccel[3];
int Gyro[3];
float RotationalVelocity[3];
float AbsolutePosition[3];
float RelativeVelocity[3];
float AbsoluteVelocity[3];
float Angles[] = {0, 0, 0};
float AccelAngles[] = {0, 0, 0};
int NewTime = 0;
int OldTime = 0;

int RECpins[] = {7, 8, 2, 4};
int MotorPins[] = {5, 6, 9, 10};
int RECtime [4];
int ScaledRECin [4];

float RollPitchAngConst=10;
float RollPitchConst=2.2;
float RPDerivitiveConst=.05;

float YawConst=1.1;
float YawAngConst=0;

float MotorOutput[]={0, 0, 0, 0};

float ThrottleConst =5.0;

int count=0;

char buf [100];
volatile byte pos;
volatile boolean process_it;

void setup (void)
{
  
    Serial.begin(9600);
  Wire.begin();
  initializeAccel();
  initializeGyro();
  pinMode(RECpins[0], INPUT);
  pinMode(RECpins[1], INPUT);
  pinMode(RECpins[2], INPUT);
  pinMode(RECpins[3], INPUT);
  pinMode(MotorPins[0], OUTPUT);
  pinMode(MotorPins[1], OUTPUT);
  pinMode(MotorPins[2], OUTPUT);
  pinMode(MotorPins[3], OUTPUT);
  
  
  Serial.begin (9600);   // debugging

  // have to send on master in, *slave out*
  pinMode(MISO, OUTPUT);
  
  // turn on SPI in slave mode
  SPCR |= _BV(SPE);
  
  // get ready for an interrupt 
  pos = 0;   // buffer empty
  process_it = false;

  // now turn on interrupts
  SPCR |= _BV(SPIE);

}  // end of setup


// SPI interrupt routine
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

// main loop - wait for flag set in interrupt routine
void loop (void)
{
  if (process_it)
    {
    buf [pos] = 0;  
    //Serial.print (int( buf[0]));
    //Serial.print (int( buf[1]));
    //Serial.print (int( buf[2]));
    //Serial.println (" ");
    pos = 0;
    process_it = false;
    
    
    if (pulseIn(2, HIGH, 50000)){
    for (int i=0; i<4; i++) {
      RECtime[i]=pulseIn(RECpins[i], HIGH, 1000000);
      ScaledRECin[i]=(RECtime[i]-1000)/10;
      Serial.print(ScaledRECin[i]);
      Serial.print("  ");
   }
   
        MotorOutput[0] = ThrottleConst*ScaledRECin[0] + (ScaledRECin[2]-50)*RollPitchConst + (ScaledRECin[1]-50)*YawConst + buf[0]*RollPitchAngConst + YawAngConst*buf[3];
        MotorOutput[1] = ThrottleConst*ScaledRECin[0] + (ScaledRECin[3]-50)*RollPitchConst + (ScaledRECin[1]-50)*YawConst + buf[1]*RollPitchAngConst - YawAngConst*buf[3];
        MotorOutput[2] = ThrottleConst*ScaledRECin[0] - (ScaledRECin[2]-50)*RollPitchConst - (ScaledRECin[1]-50)*YawConst - buf[0]*RollPitchAngConst + YawAngConst*buf[3];
        MotorOutput[3] = ThrottleConst*ScaledRECin[0] - (ScaledRECin[3]-50)*RollPitchConst - (ScaledRECin[1]-50)*YawConst - buf[1]*RollPitchAngConst - YawAngConst*buf[3];
        
   
   
 for (int i=0; i<4; i++) {
        MotorOutput[i]=map((MotorOutput[i]-30), 0, 550, 0, 255);
        if (MotorOutput[i]<0){
          MotorOutput[i]=0;
        }
        Serial.print(MotorOutput[i]);
        Serial.print("  ");
      }
      Serial.println("");
      
      for (int i=0; i<4; i++) {
        analogWrite(MotorPins[i], MotorOutput[i]);
      }
  
    }  // end of flag set
    
 
   
   
    
    
 } else {
   Serial.println("Did not get a newline");
 }
    
}  // end of loop


void ConvertGyro(int * Gyro, float * RotationalVelocity){
   for (int i=0; i<3; i++) { //Convert to Degrees/s
    RotationalVelocity[i]=Gyro[i]/14.375; 
  }
  }

void NormalizeAccel(int * Accel, float * NormAccel){
  for (int i=0; i<3; i++) {
     NormAccel[i]=Accel[i];
  }
  float OneG=sqrt(NormAccel[0]*NormAccel[0]+NormAccel[1]*NormAccel[1]+NormAccel[2]*NormAccel[2]);
  for (int i=0; i<3; i++) { 
     NormAccel[i]=NormAccel[i]/OneG*32.174; //ft per second squared
  }
  }

void PrintRaw (int * Accel, int * Gyro) {
  sprintf(str, "ACCEL: %d  %d  %d  GYRO: %d  %d  %d", Accel[0], Accel[1], Accel[2], Gyro[0], Gyro[1], Gyro[2]);  
  Serial.print(str);
  Serial.print(10, BYTE);
}

void WriteToDevice(int Device_Address, byte Register, byte Value){
  Wire.beginTransmission(Device_Address);
  Wire.send(Register);
  Wire.send(Value);
  Wire.endTransmission();
}

void ReadFromDevice(int Device_Address, byte Register, int Quantity, byte buff[]) {
  Wire.beginTransmission(Device_Address);
  Wire.send(Register);
  Wire.endTransmission();
  Wire.beginTransmission(Device_Address);
  Wire.requestFrom(Device_Address, Quantity);
  int i=0;
  while(Wire.available()) {
    buff[i] = Wire.receive();
    i++;
  }
  Wire.endTransmission();
}

void initializeGyro() {
  WriteToDevice(Gyro_Address, 0x3E, 0x00); //Power Managment, Default Values
  WriteToDevice(Gyro_Address, 0x15, 0x09); //Internal Sample Rate, Set to 100hz
  WriteToDevice(Gyro_Address, 0x16, 0x1d); //10hz Low Pass, 2000 deg/sec range
  WriteToDevice(Gyro_Address, 0x17, 0x00); //No Interupts
}

void initializeAccel() {
  WriteToDevice(Accel_Address, 0x2D, 0); //Initialize (0x2D is Power Control Register)
  WriteToDevice(Accel_Address, 0x2D, 16); //Enable Automatic Sleep Mode
  WriteToDevice(Accel_Address, 0x2D, 8); //Puts Device in to Measurement Mode
  WriteToDevice(Accel_Address, 0x31, 0x00); // (0x0X)g Range, default is 2g range
  WriteToDevice(Accel_Address, 0x2C, 0x09); // Set Sample Rate to 25hz
}

void ReadGyro(int * result){
  byte buff[Num_Bytes_Gyro];
  ReadFromDevice(Gyro_Address, 0x1B, Num_Bytes_Gyro, buff);
  result[0]=((buff[2] << 8) | buff[3]) + X_Offset;
  result[1]=((buff[4] << 8) | buff[5]) + Y_Offset;
  result[2]=((buff[6] << 8) | buff[7]) + Z_Offset;
}

void ReadAccel(int * result) {
  byte buff[Num_Bytes_Accel];
  ReadFromDevice(Accel_Address, 0x32, Num_Bytes_Accel, buff);
  result[0] = ((((int)buff[1]) << 8) | buff[0]) - 12;   
  result[1] = ((((int)buff[3])<< 8) | buff[2]) + 3;
  result[2] = ((((int)buff[5]) << 8) | buff[4]) + 15;
}

int StartingOrient(int * Accel, float * Angles){
  Angles[0] = atan2(Accel[1], Accel[2])*180/PI;
  Angles[1] = -atan2(Accel[0], Accel[2])*180/PI;
  Angles[2] = 0;
}

void Integrate(float * Integrand, int NewTime, int OldTime, float * Result){
  
float NewTimes = (int) NewTime;
float OldTimes = (int) OldTime;
float dx = (NewTimes - OldTimes)/1000; //delta x;
Result[0] = Integrand[0]*dx;
Result[1] = Integrand[1]*dx;
Result[2] = Integrand[2]*dx;
}

