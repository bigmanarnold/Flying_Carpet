#include<Wire.h>
const int MPU_addr=0x68;  // I2C address of the MPU-6050
int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;
double timefor=millis();
int i = 1;
double gyroX,gyroY,gyroZ;
double angleX,angleY,angleZ;
double accelerationAngleX,accelerationAngleY,accelerationAngleZ;
double dt = 0;
double xoffset,yoffset,zoffset;
double xangularoffset,yangularoffset,zangularoffset;


//-----------------------------------------------------------------------------------------------------------------------------------------------------------



void setup(){
  Serial.begin(9600);
  Serial.println("Beginning setup");
  Wire.begin();
 
  
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(false);

  //calibrates the MPU-6050
  for(int loopcounter = 0; loopcounter < 1000; loopcounter++){
    Wire.beginTransmission(MPU_addr);
    Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_addr,14,true);  // request a total of 14 registers
    AcX=Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)    
    AcY=Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
    AcZ=Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
    Tmp=Wire.read()<<8|Wire.read();  // 0x14 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
    GyX=Wire.read()<<8|Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
    GyY=Wire.read()<<8|Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
    GyZ=Wire.read()<<8|Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)

    xoffset = xoffset + AcX;
    yoffset = yoffset + AcY;
    zoffset = zoffset + AcZ;
    xangularoffset = xangularoffset +GyX;
    yangularoffset = yangularoffset +GyY;
    zangularoffset = zangularoffset +GyZ;
  }
  xoffset = xoffset/1000;
  yoffset = yoffset/1000;
  zoffset = 16384-(zoffset/1000); //use the 16384 as that is the equivalent of 1g
  xangularoffset=xangularoffset/1000;
  yangularoffset=yangularoffset/1000;
  zangularoffset=zangularoffset/1000;

  Serial.println("Setup Complete");
}



//-----------------------------------------------------------------------------------------------------------------------------------------------------------


void loop(){

  //read data for the first MPU-6050
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr,14,true);  // request a total of 14 registers
  AcX=Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)    
  AcY=Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  AcZ=Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  Tmp=Wire.read()<<8|Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  GyX=Wire.read()<<8|Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  GyY=Wire.read()<<8|Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  GyZ=Wire.read()<<8|Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)

  //Adjusts for Calibration
  AcX = AcX-xoffset;
  AcY = AcY-yoffset;
  AcZ = AcZ-zoffset;

  //scale and adjust for calibration
  GyX = (GyX-xangularoffset)/131;
  GyY = (GyY-yangularoffset)/131;
  GyZ = (GyZ-zangularoffset)/131;

  //find the angle of acceleration
  accelerationAngleX = (180/3.141592) * atan2(AcX , sqrt(pow(AcY,2) + pow(AcZ,2))); 
  accelerationAngleY = (180/3.141592) * atan2(AcY , sqrt(pow(AcX,2) + pow(AcZ,2)));
  accelerationAngleZ = (180/3.141592) * atan2(sqrt(pow(AcY,2) + pow(AcX,2)) , AcZ);

  if (i == 1){
    //initiallizes the gyro angle
    gyroX = accelerationAngleX;
    gyroY = accelerationAngleY;
    gyroZ = accelerationAngleZ;
    timefor = millis();
  }
  else{
    //integrate the gyroscopic change
    dt = abs((timefor-millis()))/1000;
    timefor = millis();
    gyroX = gyroX + (dt*GyX);
    gyroY = gyroY + (dt*GyY);
    gyroZ = gyroZ + (dt*GyZ);
  }

  //filters the data
  angleX = (.96 * accelerationAngleX) + (.04*gyroX);
  angleY = (.96 * accelerationAngleY) + (.04*gyroY);
  angleZ = (.96 * accelerationAngleZ) + (.04*gyroZ);

  //print out the angle in each direction
  Serial.print(angleX); Serial.print(", ");
  Serial.print(angleY); Serial.print(", ");
  Serial.println(angleZ);


  i++;
}
