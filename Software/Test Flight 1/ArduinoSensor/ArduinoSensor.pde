#include <Wire.h>
#include <SPI.h>
#include "pins_arduino.h"

char buf [100];
char b=0;
volatile byte pos;
volatile boolean process_it;

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
float Angles[3]={0, 0, 0};
float Zeros[3]={0, 0, 0};
float AccelAngles[3];
int NewTime;
int OldTime;
float OneG;


int Start=1;

void setup() {
  pinMode(MISO, OUTPUT);
  
  SPI.begin ();
  SPI.setClockDivider(SPI_CLOCK_DIV8);
  
  Serial.begin(9600);
  Wire.begin();
  initializeAccel();
  initializeGyro();
  ReadAccel(Accel);
  OneG = FindOneG(Accel, NormAccel);
  Serial.println(OneG);
}

void loop() {
  

  OldTime = millis();
  ReadAccel(Accel);
  NormalizeAccel(Accel, NormAccel, OneG);
  ReadGyro(Gyro);
  ConvertGyro(Gyro, RotationalVelocity);
  StartingOrient(Accel, AccelAngles);
  Angles[0]=AccelAngles[0]/4+Angles[0]*3/4;
  Angles[1]=AccelAngles[1]/4+Angles[1]*3/4;
 /* 
  if (Angles[2]>360){
    Angles[2]=Angles[2]-360;
  }
  if (Angles[2]<-360){
    Angles[2]=Angles[2]+360;
  }
  */
 
  
  
  for (int i=0; i<5; i++) {
    NewTime = millis();
    ReadAccel(Accel);
    NormalizeAccel(Accel, NormAccel, OneG);
    ReadGyro(Gyro);
    for (int i=0; i<3; i++){
      if (Gyro[i]<5 && Gyro[i]>-5) {
        Gyro[i]=0;
      }
    }
    ConvertGyro(Gyro, RotationalVelocity);
    PrintResults(Angles, Zeros);
    //KillGravity(Angles, NormAccel);
    //Integrate(NormAccel, NewTime, OldTime, RelativeVelocity);
    Integrate(RotationalVelocity, NewTime, OldTime, Angles);
    //EulerAngles(Angles, RelativeVelocity, AbsoluteVelocity);
    //Integrate(AbsoluteVelocity, NewTime, OldTime, AbsolutePosition);
    //PrintResults(NormAccel, Angles);
    OldTime = NewTime;
    

  char c;

  // enable Slave Select
  digitalWrite(SS, LOW);    // SS is pin 10

  SPI.transfer(byte(Angles[0]));
  SPI.transfer(byte(Angles[1]));
  SPI.transfer(byte(Angles[2]));
  // send test string
  for (const char * p = "a\n" ; c = *p; p++)
    SPI.transfer (c);

  // disable Slave Select
  digitalWrite(SS, HIGH);

    
  }
  

}

void KillGravity(float * Angles, float * NormAccel){
  NormAccel[0]=NormAccel[0]-32.17*cos(Angles[1]/57.3)*cos(Angles[2]/57.3);
  NormAccel[1]=NormAccel[1]-32.17*cos(Angles[2]/57.3)*cos(Angles[0]/57.3);
  NormAccel[2]=NormAccel[2]-32.17*cos(Angles[0]/57.3)*cos(Angles[1]/57.3);
}

void PrintResults(float * AbsolutePosition, float * Angles){
  Serial.print(AbsolutePosition[0]);
  Serial.print(" ");
  Serial.print(AbsolutePosition[1]);
  Serial.print(" ");
  Serial.print(AbsolutePosition[2]);
  Serial.print(" ");
  Serial.print(Angles[0]);
  Serial.print(" ");
  Serial.print(Angles[1]);
  Serial.print(" ");
  Serial.print(Angles[2]);
  Serial.println(" ");
}


void EulerAngles(float * Angles, float * RelativeVelocity, float * AbsoluteVelocity){
  float Cphi=cos(Angles[0]);
  float Ctheta=cos(Angles[1]);
  float Cpsi=cos(Angles[2]);
  float Sphi=sin(Angles[0]);
  float Stheta=sin(Angles[1]);
  float Spsi=sin(Angles[2]);

  AbsoluteVelocity[0]=Ctheta*Cpsi*RelativeVelocity[0] + (Sphi*Stheta*Cpsi - Cphi*Spsi)*RelativeVelocity[1] + (Cphi*Stheta*Cpsi + Sphi*Spsi)*RelativeVelocity[2];
  AbsoluteVelocity[1]=Ctheta*Spsi*RelativeVelocity[0] + (Sphi*Stheta*Spsi + Cphi*Cpsi)*RelativeVelocity[1] + (Cphi*Stheta*Spsi - Sphi*Spsi)*RelativeVelocity[2];
  AbsoluteVelocity[2]=-Stheta*RelativeVelocity[0] + Sphi*Ctheta*RelativeVelocity[1] + Cphi*Ctheta*RelativeVelocity[2];
}


void ConvertGyro(int * Gyro, float * RotationalVelocity){
   for (int i=0; i<3; i++) { //Convert to Degrees/s
    RotationalVelocity[i]=Gyro[i]/14.375; 
  }
  }

void NormalizeAccel(int * Accel, float * NormAccel, float OneG){
  for (int i=0; i<3; i++) {
     NormAccel[i]=Accel[i];
  }
  for (int i=0; i<3; i++) { 
     NormAccel[i]=NormAccel[i]/OneG*32.174; //ft per second squared
  }
}
 
int FindOneG(int * Accel, float * NormAccel){
  for (int i=0; i<3; i++) {
     NormAccel[i]=Accel[i];
  }
  OneG=sqrt(NormAccel[0]*NormAccel[0]+NormAccel[1]*NormAccel[1]+NormAccel[2]*NormAccel[2]);
  return(OneG);
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
float DT = (NewTimes - OldTimes)/1000;
Result[0] = Integrand[0]*DT+Result[0];
Result[1] = Integrand[1]*DT+Result[1];
Result[2] = Integrand[2]*DT+Result[2];
}
