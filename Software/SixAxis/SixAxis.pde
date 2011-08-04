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
int Gyro[3];
float Rot[3];
float Pos[3];
float RelV[3];
float Vel[3];
float RotV[3];
float RotA[3];

void setup() {
  Serial.begin(9600);
  Wire.begin();
  initializeAccel();
  initializeGyro();
}

void loop() {
  int time = millis();
  
  ReadAccel(Accel);
  float NormAccel[3];
  for (int i=0; i<3; i++) { 
     NormAccel[i]=Accel[i];
  }
  float OneG=sqrt(NormAccel[0]*NormAccel[0]+NormAccel[1]*NormAccel[1]+NormAccel[2]*NormAccel[2]);
  for (int i=0; i<3; i++) { 
     NormAccel[i]=NormAccel[i]/OneG*32.174; //ft per second squared
  }
  
  ReadGyro(Gyro);
  for (int i=0; i<3; i++) { //Convert to Degrees/s
    RotA[i]=Gyro[i]/14.375*180/PI; 
  }
  
  
  

  
  PrintRaw(Accel, Gyro);
  StartingOrient(Accel, Rot);
  
  delay(100);
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
  WriteToDevice(Accel_Address, 0x2C, 0x07); // Set Sample Rate to 25hz
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

int StartingOrient(int * Accel, float * Rot){
  Rot[0] = atan2(Accel[1], Accel[2])*180/PI;
  Rot[1] = -atan2(Accel[0], Accel[2])*180/PI;
  Rot[2] = 0;
}

