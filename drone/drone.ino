/************************************************************
 *  MINI QUADCOPTER
 *  ESP32 + MPU6050 (DMP + Kalman)
 *  MODE: ANGLE (SETPOINT = 0)
 ************************************************************/

#include <Wire.h>
#include <ESP32Servo.h>

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <SimpleKalmanFilter.h>

/* ================= CONFIG ================= */
// LOOP
#define LOOP_TIME_US   3000    // ~333Hz = 3 ms

// ESC
#define MIN_THROTTLE   1050
#define MAX_THROTTLE   1400
#define IDLE_THROTTLE  1250

// ESC pins
#define M1_PIN 18
#define M2_PIN 19
#define M3_PIN 23
#define M4_PIN 5

// PID (SAFE DESK TEST)
#define P_ANGLE  1.8f
#define I_ANGLE  0.0f
#define D_ANGLE  0.2f

#define PID_LIMIT 40
/* ================= GLOBAL ================= */

// MPU6050 + DMP
MPU6050 mpu;
bool dmpReady = false;
uint8_t fifoBuffer[64];
uint16_t packetSize;

Quaternion q;
VectorFloat gravity;
float ypr[3];

// Kalman filter
SimpleKalmanFilter kalmanRoll(2, 2, 0.01);
SimpleKalmanFilter kalmanPitch(2, 2, 0.01);

// Angles
float angleRoll = 0.0f;
float anglePitch = 0.0f;

// PID
float pidRoll = 0, pidPitch = 0;
float iRoll = 0, iPitch = 0;
float lastRollErr = 0, lastPitchErr = 0;

// Throttle
float throttle = IDLE_THROTTLE;

// Time
uint32_t lastLoopTime = 0;
uint32_t lastPIDTime  = 0;

// Motors
Servo esc1, esc2, esc3, esc4;

/* ================= IMU INIT ================= */

void imuInit() {
  Wire.begin();
  Wire.setClock(400000);
  mpu.initialize(); // Bật PWR_MGMT_1: 6B, GYRO_CONFIG: 1B, ACCEL_CONFIG: 1C, Bộ lọc thông thấp (DLPF): 1A

  if (mpu.dmpInitialize() == 0) {
    mpu.setDMPEnabled(true);
    dmpReady = true;
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    Serial.println("MPU6050 DMP init FAILED");
    while (1);
  }

  // Ranges
  mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_2000);
  mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_8);

  // Calibration (RẤT QUAN TRỌNG)
  mpu.CalibrateAccel(6);
  mpu.CalibrateGyro(6);

  Serial.println("MPU6050 DMP READY");
}

/* ================= IMU UPDATE ================= */
void imuUpdate() {
  if (!dmpReady) return;
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {

    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

    float rawRoll  = ypr[2] * 180.0f / M_PI;
    float rawPitch = ypr[1] * 180.0f / M_PI;

    angleRoll  = kalmanRoll.updateEstimate(rawRoll);
    anglePitch = kalmanPitch.updateEstimate(rawPitch);

    // Deadzone nhỏ
    if (abs(angleRoll)  < 0.5f) angleRoll  = 0;
    if (abs(anglePitch) < 0.5f) anglePitch = 0;
  }
}
/* ================= PID UPDATE ================= */

void pidUpdate() {
  float dt = (micros() - lastPIDTime) * 1e-6f;
  lastPIDTime = micros();
  if (dt <= 0 || dt > 0.05f) return;

  // Setpoint = 0
  float errRoll  = -angleRoll;
  float errPitch = -anglePitch;

  // Integral
  iRoll  += errRoll  * dt;
  iPitch += errPitch * dt;

  iRoll  = constrain(iRoll,  -100, 100);
  iPitch = constrain(iPitch, -100, 100);

  // Derivative
  float dRoll  = (errRoll  - lastRollErr)  / dt;
  float dPitch = (errPitch - lastPitchErr) / dt;

  pidRoll  = P_ANGLE * errRoll  + I_ANGLE * iRoll  + D_ANGLE * dRoll;
  pidPitch = P_ANGLE * errPitch + I_ANGLE * iPitch + D_ANGLE * dPitch;

  pidRoll  = constrain(pidRoll,  -PID_LIMIT, PID_LIMIT);
  pidPitch = constrain(pidPitch, -PID_LIMIT, PID_LIMIT);

  lastRollErr  = errRoll;
  lastPitchErr = errPitch;
}

/* ================= MOTOR MIX ================= */
void motorWrite() {
   int MOTOR_3_4_OFFSET = 12.21;

  int m3 = throttle - pidRoll + pidPitch + MOTOR_3_4_OFFSET;
  int m4 = throttle - pidRoll - pidPitch + MOTOR_3_4_OFFSET;
  int m1 = throttle + pidRoll + pidPitch;
  int m2  = throttle + pidRoll - pidPitch;

  m1 = constrain(m1, MIN_THROTTLE, MAX_THROTTLE);
  m2 = constrain(m2, MIN_THROTTLE, MAX_THROTTLE);
  m3 = constrain(m3, MIN_THROTTLE, MAX_THROTTLE);
  m4 = constrain(m4, MIN_THROTTLE, MAX_THROTTLE);

  esc1.writeMicroseconds(m1);
  esc2.writeMicroseconds(m2);
  esc3.writeMicroseconds(m3);
  esc4.writeMicroseconds(m4);
}

void printSerialPlotter() {
  Serial.print(anglePitch);  // Output hệ thống
  Serial.print(" ");
  Serial.println(pidPitch);  // Output PID
}

/* ================= SETUP ================= */

void setup() {
  Serial.begin(115200);
  delay(2000);

  imuInit();

  esc1.setPeriodHertz(50);
  esc2.setPeriodHertz(50);
  esc3.setPeriodHertz(50);
  esc4.setPeriodHertz(50);

  esc1.attach(M1_PIN, 1000, 2000);
  esc2.attach(M2_PIN, 1000, 2000);
  esc3.attach(M3_PIN, 1000, 2000);
  esc4.attach(M4_PIN, 1000, 2000);

  // ARM ESC – CHUẨN
  esc1.writeMicroseconds(1000);
  esc2.writeMicroseconds(1000);
  esc3.writeMicroseconds(1000);
  esc4.writeMicroseconds(1000);
  delay(3000);

  // Tăng dần lên IDLE
  for (int t = 1000; t <= IDLE_THROTTLE; t += 5) {
    esc1.writeMicroseconds(t);
    esc2.writeMicroseconds(t);
    esc3.writeMicroseconds(t);
    esc4.writeMicroseconds(t);
    delay(20);
  }

  throttle = IDLE_THROTTLE;

  lastPIDTime  = micros();
  lastLoopTime = micros();
  Serial.println("READY");
}
/* ================= LOOP ================= */
void loop() {
  if (micros() - lastLoopTime < LOOP_TIME_US) return;
  lastLoopTime = micros();

  imuUpdate();
  pidUpdate();
  motorWrite();


}









