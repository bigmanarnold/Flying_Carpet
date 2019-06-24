#include "QuaternionControl.h"
#include "Madgwick.h"
#include "I2Cdev.h"
#include "MPU6050.h"
#include <Servo.h>
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif
MPU6050 accelgyro;
#define OUTPUT_READABLE_ACCELGYRO


// IMU
int16_t ax, ay, az;
int16_t gx, gy, gz;
float AccelOffset[3];
float GyroOffset[3];

// RCVR
int rollpin = 23;
int pitchpin = 25;
int thrpin = 27;
int yawpin = 29;
int ch5pin = 31;
volatile int thr_value = 0;
volatile int thr_time = 0;
volatile int roll_value = 0;
volatile int roll_time = 0;
volatile int pitch_value = 0;
volatile int pitch_time = 0;
volatile int yaw_value = 0;
volatile int yaw_time = 0;
volatile int ch5_value = 0;
volatile int ch5_time = 0;
/*
  float qd[4]= {0.9641, 0.0843, 0.0729, 0.2410};
  float q[4] = {0.9460, 0.2044, 0.0016, 0.2518};
*/
float R, P, Y;
float aXYZ[3];
float qd[4] = {1, 0, 0, 0};
float  q[4];
float w[3] = {0};
int pwm[4] = {0}, cot[4] = {0}, KILL = 0;
float out[4], Desired[4];       // Desired = {U0, U3, roll, pitch)
float timenow, timeprev = 0, dt;
Control control;
Madgwick AHRS;
Servo m1, m2, m3, m4;
void StopMotors();
void fetchIMU(float*, float*);
void RPY(float*);
void DesiredQuat(float, float);
void InputMapping(int, int, int, int, float*);



void setup() {
  Serial.begin(9600);
  m1.attach(3);
  m2.attach(4);
  m3.attach(5);
  m4.attach(6);

  m1.writeMicroseconds(1500);
  m2.writeMicroseconds(1500);
  m3.writeMicroseconds(1500);
  m4.writeMicroseconds(1500);
  attachInterrupt(thrpin, risingThrottle, RISING);
  attachInterrupt(rollpin, risingRoll, RISING);
  attachInterrupt(pitchpin, risingPitch, RISING);
  attachInterrupt(yawpin, risingYaw, RISING);
  attachInterrupt(ch5pin, risingCh5, RISING);

  // join I2C bus (I2Cdev library doesn't do this automatically)
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin();
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    Fastwire::setup(400, true);
  #endif

  // initialize devices
  accelgyro.initialize();
  offsetIMUdata(AccelOffset, GyroOffset);

  // verify connection
  Serial.println("Testing device connections...");
  Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
}

int t0 = millis();

void loop() {
  float n = 0; int cnt = 1;
  int tstart = micros();

  if (KILL == 1) {
    StopMotors();
    Serial.println("KILL SWITCH ON!");
  }
  else {
    // compute estimate of state
    fetchIMU(aXYZ, w);
    timenow = ((float)millis()) / 1000.0f; dt = timenow - timeprev; timeprev = timenow;
    AHRS.AHRSupdate(dt, aXYZ, w, q, out);
    // get desired values
    InputMapping(thr_value, roll_value, pitch_value, yaw_value, Desired);
    DesiredQuat(Desired[2], Desired[3]);

    // compute control
    control.attitude_control(Desired[0], Desired[1], qd, q, w, pwm, out);

    // pass values to motors
    m1.writeMicroseconds(pwm[0]);
    m2.writeMicroseconds(pwm[1]);
    m3.writeMicroseconds(pwm[2]);
    m4.writeMicroseconds(pwm[3]);


    // output data
    /*
      for (int n = 0; n<4; n++) cot[n] = map(pwm[n], 1000, 2000, 0, 100);

      Serial.print(thr_value); Serial.print("\t");
      Serial.print(cot[0]); Serial.print("\t");
      Serial.print(cot[1]); Serial.print("\t");
      Serial.print(cot[2]); Serial.print("\t");
      Serial.print(cot[3]); Serial.println("\t");
    */
  }

  int tend = micros();
  Serial.print(aXYZ[0]); Serial.print("\t");
  Serial.print(aXYZ[1]); Serial.print("\t");
  Serial.print(aXYZ[2]); Serial.print("\t");
  Serial.print(w[0]); Serial.print("\t");
  Serial.print(w[1]); Serial.print("\t");
  Serial.print(w[2]); Serial.print("\t");
  /*
    Serial.println();
    Serial.print("dt:\t");Serial.print(tstart-tend);
    Serial.print("\t\tn:\t");
  */
}


void offsetIMUdata(float AccelOffset[3], float GyroOffset[3]) {
  for (int i = 0; i < 5000; i++) {
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    AccelOffset[0] += ax;
    AccelOffset[1] += ay;
    AccelOffset[2] += az;
    GyroOffset[0] += gx;
    GyroOffset[1] += gy;
    GyroOffset[2] += gz;
  }
  for (int i = 0; i < 6; i++) {
    AccelOffset[i] = AccelOffset[i] / 5000;

    // using 2g sensitivity
    AccelOffset[0] = -AccelOffset[0];
    AccelOffset[1] = -AccelOffset[1];
    AccelOffset[2] = 16384 - AccelOffset[2];
  }
  for (int i = 0; i < 6; i++) {
    GyroOffset[i] = GyroOffset[i] / 5000;
    GyroOffset[0] = -GyroOffset[0];
    GyroOffset[1] = -GyroOffset[1];
    GyroOffset[2] = -GyroOffset[2];
  }
}

void fetchIMU(float aXYZ[3], float w[3]) {
  static float LPp = 0;
  static float LPq = 0;
  static float LPr = 0;
  static float LPx = 0;
  static float LPy = 0;
  static float LPz = 1;

  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  LPx = 0.9 * LPx + 0.1 * (float)(-ax - AccelOffset[0]) / 16834.0;
  LPy = 0.9 * LPy + 0.1 * (float)(ay + AccelOffset[1]) / 16834.0;
  LPz = 0.9 * LPz + 0.1 * (float)(az + AccelOffset[2]) / 16834.0;
  LPp = 0.9 * LPp + 0.1 * (float)( gx - GyroOffset[0]) * 1.3323e-04;
  LPq = 0.9 * LPq + 0.1 * (float)(-gy - GyroOffset[1]) * 1.3323e-04;
  LPr = 0.9 * LPr + 0.1 * (float)(-gz - GyroOffset[2]) * 1.3323e-04;
  aXYZ[0] = LPx;
  aXYZ[1] = LPy;
  aXYZ[2] = LPz;

  w[0] = LPp;
  w[1] = LPq;
  w[2] = LPr;
}

void RPY(float q[4]) {
  R = atan2(2 * (q[0] * q[1] + q[2] * q[3]), q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]) * 180.0f / 3.14159f;
  P = asin(2 * (q[0] * q[2] - q[1] * q[3])) * 180.0f / 3.14159f;
  Y = atan2(2 * (q[0] * q[3] + q[1] * q[2]), 1 - 2 * (q[2] * q[2] + q[3] * q[3])) * 180.0f / 3.14159f;
  Serial.print(R); Serial.print(",");
  Serial.print(P); Serial.print(",");
  Serial.println(Y);
}

void DesiredQuat(float roll, float pitch) {
  // finds desired quaternion for roll and pitch only
  qd[0] = cos(roll / 2) * cos(pitch / 2);
  qd[1] = cos(pitch / 2) * sin(roll / 2);
  qd[2] = cos(roll / 2) * sin(pitch / 2);
  qd[3] = -sin(pitch / 2) * sin(roll / 2);
}

void InputMapping(int TPWM, int rollPWM, int pitchPWM, int yawPWM, float Desired[4]) {
  static float yaw_m = 4.902e-5, yaw_b = -.0730;
  static float rp_m = 2 * 3.14159 / 5000,  rp_b = -3.14159 / 5 - 2 * 3.14159 * 990 / 5000;
  static float LPthrottle = 990;
  
  // Total Thrust
  LPthrottle = .5 * LPthrottle + .5 * (float)TPWM;
  Desired[0] = 20 * (LPthrottle - 1050) / 950;
  if ( 0 > Desired[0]) Desired[0] = 0;
  // Yaw Moment
  if ((1505 > yawPWM) && (yawPWM > 1480)) {
    Desired[1] = 0;
  } else {
    Desired[1] = yaw_m * (float)yawPWM + yaw_b;
  }
  // Roll
  if ((1505 > rollPWM) && (rollPWM > 1480)) {
    Desired[2] = 0;
  } else {
    Desired[2] = rp_m * (float)rollPWM + rp_b;
  }
  // Pitch
  if ((1505 > pitchPWM) && (pitchPWM > 1480)) {
    Desired[3] = 0;
  } else {
    Desired[3] = rp_m * (float)pitchPWM + rp_b;
  }
}



// interrupt/RCVR code! //////////////////////////////////////////////////////////////////////////////
void risingThrottle() {
  attachInterrupt(thrpin, fallingThrottle, FALLING);
  thr_time = micros();
}
void fallingThrottle() {
  attachInterrupt(thrpin, risingThrottle, RISING);
  thr_value = micros() - thr_time;
  // Serial.println(thr_value);
}
void risingRoll() {
  attachInterrupt(rollpin, fallingRoll, FALLING);
  roll_time = micros();
}
void fallingRoll() {
  attachInterrupt(rollpin, risingRoll, RISING);
  roll_value = micros() - roll_time;
  // Serial.println(roll_value);
}
void risingPitch() {
  attachInterrupt(pitchpin, fallingPitch, FALLING);
  pitch_time = micros();
}
void fallingPitch() {
  attachInterrupt(pitchpin, risingPitch, RISING);
  pitch_value = micros() - pitch_time;
  // Serial.println(pitch_value);
}
void risingYaw() {
  attachInterrupt(yawpin, fallingYaw, FALLING);
  yaw_time = micros();
}
void fallingYaw() {
  attachInterrupt(yawpin, risingYaw, RISING);
  yaw_value = micros() - yaw_time;
  // Serial.println(yaw_value);
}
void risingCh5() {
  attachInterrupt(ch5pin, fallingCh5, FALLING);
  ch5_time = micros();
}
// kill switch is on SWC if TX (off = 990, on = 1900)
void fallingCh5() {
  attachInterrupt(ch5pin, risingCh5, RISING);
  ch5_value = micros() - ch5_time;
  if (ch5_value < 1500) {
    KILL = 1;
  }
  else {
    KILL = 0;
  }
  // Serial.println(ch5_value);
}
void StopMotors() {
  m1.writeMicroseconds(990);
  m2.writeMicroseconds(990);
  m3.writeMicroseconds(990);
  m4.writeMicroseconds(990);
}
