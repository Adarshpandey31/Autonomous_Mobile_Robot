#include <Wire.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_Sensor.h>
#include <utility/imumaths.h>
#include <CytronMotorDriver.h>
#include <math.h>

// ====== Motor objects (PWM_DIR mode) ======
CytronMD motor1(PWM_DIR, 12, 38);  // M1 - Front Left
CytronMD motor2(PWM_DIR, 10, 36);  // M2 - Front Right
CytronMD motor3(PWM_DIR, 13, 24);  // M3 - Rear Right
CytronMD motor4(PWM_DIR, 11, 22);  // M4 - Rear Left

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

// ====== Robot geometry (tune these) ======
const float WHEEL_RADIUS = 0.05f;  // meters
const float Lx = 0.12f;            // meters (half length from center to wheel along x)
const float Ly = 0.12f;            // meters (half width from center to wheel along y)
const float DIAG = (Lx + Ly);      // used in rotation term
const float INV_SQRT2 = 0.70710678118654752440f;

// Maximum wheel linear speed (m/s) that corresponds to full motor command
const float MAX_WHEEL_LINEAR_SPEED = 1.0f;
const int PWM_MAX = 255;

// Quick sign toggles if wiring/polarity differs (change to -1 or 1)
const int SIGN_M1 = 1;
const int SIGN_M2 = -1;
const int SIGN_M3 = 1;
const int SIGN_M4 = -1;

// If your chosen convention for axes is different, flip these
const int SIGN_VX = 1;  // forward positive
const int SIGN_VY = 1;  // right positive
const int SIGN_WZ = 1;  // CCW positive

// Precompute 1/sqrt(2)

// ====== Runtime variables ======
float vx = 0.0f;  // m/s
float vy = 0.0f;  // m/s
float wz = 0.0f;  // rad/s

void setup() {
  Serial.begin(115200);

  if (!bno.begin()) {
    Serial.println("IMU not detected");
    while (1)
      ;
  }
  delay(1000);
  bno.setExtCrystalUse(true);
  Serial.println("Initialised");

  allStop();
}

void loop() {
  if (Serial.available()) {
    String line = Serial.readStringUntil('\n');
    line.trim();
    if (line.length() > 0) parseIncomingData(line);
  }
  applyMotorSpeed(vx, vy, wz);

  transmitIMUData();

  delay(10);
}

void parseIncomingData(String &s) {
  s.replace(',', ' ');
  s.trim();
  int i1 = s.indexOf(' ');
  if (i1 < 0) return;
  int i2 = s.indexOf(' ', i1 + 1);
  if (i2 < 0) return;

  vx = s.substring(0, i1).toFloat();
  vy = s.substring(i1 + 1, i2).toFloat();
  wz = s.substring(i2 + 1).toFloat();
}

void applyMotorSpeed(float vx, float vy, float wz) {
  // Apply sign conventions
  vx *= SIGN_VX;
  vy *= SIGN_VY;
  wz *= SIGN_WZ;

  // Compute wheel linear speeds
  float v1 = ((vx - vy) * INV_SQRT2 + wz * DIAG);  // Front Left
  float v2 = ((vx + vy) * INV_SQRT2 - wz * DIAG);  // Front Right
  float v3 = ((vx + vy) * INV_SQRT2 + wz * DIAG);  // Rear Right
  float v4 = ((vx - vy) * INV_SQRT2 - wz * DIAG);  // Rear Left

  // Convert to PWM
  int pwm1 = linearSpeedToPwm(v1) * SIGN_M1;
  int pwm2 = linearSpeedToPwm(v2) * SIGN_M2;
  int pwm3 = linearSpeedToPwm(v3) * SIGN_M3;
  int pwm4 = linearSpeedToPwm(v4) * SIGN_M4;

  motor1.setSpeed(pwm1);
  motor2.setSpeed(pwm2);
  motor3.setSpeed(pwm3);
  motor4.setSpeed(pwm4);
}


void allStop() {
  motor1.setSpeed(0);
  motor2.setSpeed(0);
  motor3.setSpeed(0);
  motor4.setSpeed(0);
}

int linearSpeedToPwm(float v_mps) {
  if (v_mps > MAX_WHEEL_LINEAR_SPEED) v_mps = MAX_WHEEL_LINEAR_SPEED;
  if (v_mps < -MAX_WHEEL_LINEAR_SPEED) v_mps = -MAX_WHEEL_LINEAR_SPEED;
  return (int)(v_mps / MAX_WHEEL_LINEAR_SPEED * PWM_MAX);
}

void transmitIMUData() {
  imu::Quaternion q = bno.getQuat();
  imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  imu::Vector<3> acc = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);

  unsigned long t = millis();

  Serial.print("t,"); Serial.print(t);
  Serial.print(",quat,"); Serial.print(q.w(), 6); Serial.print(',');
  Serial.print(q.x(), 6); Serial.print(','); Serial.print(q.y(), 6); Serial.print(',');
  Serial.print(q.z(), 6);
  Serial.print(",gyro,"); Serial.print(gyro.x(), 4); Serial.print(',');
  Serial.print(gyro.y(), 4); Serial.print(','); Serial.print(gyro.z(), 4);
  Serial.print(",acc,"); Serial.print(acc.x(), 4); Serial.print(',');
  Serial.print(acc.y(), 4); Serial.print(','); Serial.println(acc.z(), 4);
}

