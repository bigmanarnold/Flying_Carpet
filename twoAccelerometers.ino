#include<Wire.h>

const int MPU_addr1=0x68;  // I2C address of the MPU-6050
const int MPU_addr2=0x69;  // I2C address of the MPU-6050
int16_t AcX1,AcY1,AcZ1,Tmp1,GyX1,GyY1,GyZ1;
int16_t AcX2,AcY2,AcZ2,Tmp2,GyX2,GyY2,GyZ2;
double timefor1=millis();
double timefor2=millis();
int i = 1;
double gyroX1,gyroY1,gyroZ1,gyroX2,gyroY2,gyroZ2;
double angleX1,angleY1,angleZ1,angleX2,angleY2,angleZ2;
double accelerationAngleX2,accelerationAngleY2,accelerationAngleZ2,accelerationAngleX1,accelerationAngleY1,accelerationAngleZ1;
double dt = 0;
double xoffset1,yoffset1,zoffset1,xoffset2,yoffset2,zoffset2;
double xangularoffset1,yangularoffset1,zangularoffset1,xangularoffset2,yangularoffset2,zangularoffset2;


void setup(){
  Wire.begin();
  
  Wire.beginTransmission(MPU_addr1);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(false);

  //calibrates first MPU-6050
  for(int loopcounter = 0; loopcounter < 1000; loopcounter++){
    Wire.beginTransmission(MPU_addr1);
    Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_addr1,14,true);  // request a total of 14 registers
    AcX1=Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)    
    AcY1=Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
    AcZ1=Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
    Tmp1=Wire.read()<<8|Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
    GyX1=Wire.read()<<8|Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
    GyY1=Wire.read()<<8|Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
    GyZ1=Wire.read()<<8|Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)

    xoffset1 = xoffset1 + AcX1;
    yoffset1 = yoffset1 + AcY1;
    zoffset1 = zoffset1 + AcZ1;
    xangularoffset1 = xangularoffset1 +GyX1;
    yangularoffset1 = yangularoffset1 +GyY1;
    zangularoffset1 = zangularoffset1 +GyZ1;
  }
  xoffset1 = xoffset1/1000;
  yoffset1 = yoffset1/1000;
  zoffset1 = 16384-(zoffset1/1000); //use the 16384 as that is the equivalent of 1g
  xangularoffset1=xangularoffset1/1000;
  yangularoffset1=yangularoffset1/1000;
  zangularoffset1=zangularoffset1/1000;

  Wire.beginTransmission(MPU_addr2);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(false);

  //calibrates first MPU-6050
  for(int loopcounter = 0; loopcounter < 1000; loopcounter++){
    Wire.beginTransmission(MPU_addr1);
    Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_addr1,14,true);  // request a total of 14 registers
    AcX2=Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)    
    AcY2=Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
    AcZ2=Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
    Tmp1=Wire.read()<<8|Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
    GyX1=Wire.read()<<8|Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
    GyY1=Wire.read()<<8|Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
    GyZ1=Wire.read()<<8|Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)

    xoffset2 = xoffset2 + AcX2;
    yoffset2 = yoffset2 + AcY2;
    zoffset2 = zoffset2 + AcZ2;
    xangularoffset2 = xangularoffset2 +GyX2;
    yangularoffset2 = yangularoffset2 +GyY2;
    zangularoffset2 = zangularoffset2 +GyZ2;
  }
  xoffset2 = xoffset2/1000;
  yoffset2 = yoffset2/1000;
  zoffset2 = 16384-(zoffset2/1000); //use the 16384 as that is the equivalent of 1g
  xangularoffset2=xangularoffset2/1000;
  yangularoffset2=yangularoffset2/1000;
  zangularoffset2=zangularoffset2/1000;

  Serial.begin(9600);
  Serial.print("Offsets for first MPU = "); 
  Serial.print(xoffset1); Serial.print(", ");
  Serial.print(yoffset1); Serial.print(", ");
  Serial.print(zoffset1); Serial.print(", ");
  Serial.print(xangularoffset1); Serial.print(", ");
  Serial.print(yangularoffset1); Serial.print(", ");
  Serial.print(zangularoffset1); Serial.print(" ");
  Serial.println("Setup Complete...");
}



//-----------------------------------------------------------------------------------------------------------------------------------------------------------


void loop(){

  //read data for the first MPU-6050
  Wire.beginTransmission(MPU_addr1);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr1,14,true);  // request a total of 14 registers
  AcX1=Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)    
  AcY1=Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  AcZ1=Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  Tmp1=Wire.read()<<8|Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  GyX1=Wire.read()<<8|Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  GyY1=Wire.read()<<8|Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  GyZ1=Wire.read()<<8|Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)

  //Calibrates the acceleration
  AcX1 = AcX1-xoffset1;
  AcY1 = AcY1-yoffset1;
  AcZ1 = AcZ1-zoffset1;

  //scale and calibrate the gyroscope according to data sheet
  GyX1 = (GyX1-xangularoffset1)/131;
  GyY1 = (GyY1-yangularoffset1)/131;
  GyZ1 = (GyZ1-zangularoffset1)/131;

  //find the angle of acceleration
  accelerationAngleX1 = (180/3.141592) * atan2(AcX1 , sqrt(pow(AcY1,2) + pow(AcZ1,2))); 
  accelerationAngleY1 = (180/3.141592) * atan2(AcY1 , sqrt(pow(AcX1,2) + pow(AcZ1,2)));
  accelerationAngleZ1 = (180/3.141592) * atan2(sqrt(pow(AcY1,2) + pow(AcX1,2)) , AcZ1);

  if (i == 1){
    //initiallizes the gyro angle
    gyroX1 = accelerationAngleX1;
    gyroY1 = accelerationAngleY1;
    gyroZ1 = accelerationAngleZ1;
    timefor1 = millis();
  }
  else{
    //integrate the gyroscopic change
    dt = abs((timefor1-millis()))/1000;
    timefor1 = millis();
    gyroX1 = gyroX1 + (dt*GyX1);
    gyroY1 = gyroY1 + (dt*GyY1);
    gyroZ1 = gyroZ1 + (dt*GyZ1);
  }

  //filters the data
  angleX1 = (.96 * accelerationAngleX1) + (.04*gyroX1);
  angleY1 = (.96 * accelerationAngleY1) + (.04*gyroY1);
  angleZ1 = (.96 * accelerationAngleZ1) + (.04*gyroZ1);

  //print out the angle in each direction
  Serial.print(angleX1); Serial.print(", ");
  Serial.print(angleY1); Serial.print(", ");
  Serial.print(angleZ1); Serial.print(",     ,");


  

  //read data for the second MPU-6050
  Wire.beginTransmission(MPU_addr2);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr2,14,true);  // request a total of 14 registers
  AcX2=Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)    
  AcY2=Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  AcZ2=Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  Tmp2=Wire.read()<<8|Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  GyX2=Wire.read()<<8|Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  GyY2=Wire.read()<<8|Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  GyZ2=Wire.read()<<8|Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)

  //Calibrates the acceleration
  AcX2 = AcX2-xoffset2;
  AcY2 = AcY2-yoffset2;
  AcZ2 = AcZ2-zoffset2;

  //scale and calibrate the gyroscope according to data sheet
  GyX2 = (GyX2-xangularoffset2)/131;
  GyY2 = (GyY2-yangularoffset2)/131;
  GyZ2 = (GyZ2-zangularoffset2)/131;

  //find the angle of acceleration
  accelerationAngleX2 = (180/3.141592) * atan2(AcX2 , sqrt(pow(AcY2,2) + pow(AcZ2,2))); 
  accelerationAngleY2 = (180/3.141592) * atan2(AcY2 , sqrt(pow(AcX2,2) + pow(AcZ2,2)));
  accelerationAngleZ2 = (180/3.141592) * atan2(sqrt(pow(AcY2,2) + pow(AcX2,2)) , AcZ2);

  if (i == 1){
    //initiallizes the gyro angle
    gyroX2 = accelerationAngleX2;
    gyroY2 = accelerationAngleY2;
    gyroZ2 = accelerationAngleZ2;
    timefor2=millis();
  }
  else{
    //integrate the gyroscopic change
    dt = abs((timefor2-millis())/1000);
    timefor2 = millis();
    gyroX2 = gyroX2 + (dt*GyX2);
    gyroY2 = gyroY2 + (dt*GyY2);
    gyroZ2 = gyroZ2 + (dt*GyZ2);
  }

  //filters the data
  angleX2 = (.96 * accelerationAngleX2) + (.04*gyroX2);
  angleY2 = (.96 * accelerationAngleY2) + (.04*gyroY2);
  angleZ2 = (.96 * accelerationAngleZ2) + (.04*gyroZ2);

  //print out the angle in each direction
  Serial.print(angleX2); Serial.print(", ");
  Serial.print(angleY2); Serial.print(", ");
  Serial.print(angleZ2); Serial.print(",               ");
  Serial.println(dt);


  i++;
}
