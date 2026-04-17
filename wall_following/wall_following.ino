#include <Wire.h>
#include "Adafruit_VL53L0X.h"

// =====================================================
// =============== USER CONFIGURATION ===================
// =====================================================

// ---------- XSHUT pins for the 3 ToFs ----------
#define XSHUT_LEFT   10
#define XSHUT_FRONT  19
#define XSHUT_RIGHT  18

// ---------- Motor driver pins (L298N style) ----------
// Left motor
#define L_IN1  4
#define L_IN2  5
#define L_PWM  6

// Right motor
#define R_IN1  7
#define R_IN2  8
#define R_PWM  9

// ---------- PWM settings for ESP32 ----------
const int PWM_FREQ = 20000;
const int PWM_RES  = 8;   // 8-bit: 0~255
const int L_CH     = 0;
const int R_CH     = 1;

// ---------- ToF I2C addresses ----------
#define ADDR_LEFT   0x30
#define ADDR_FRONT  0x31
#define ADDR_RIGHT  0x32

// ---------- Wall following parameters ----------
float desiredWallDist = 180.0;   // mm, desired side distance from wall
float frontStopDist   = 160.0;   // mm, obstacle too close ahead
float wallLostDist    = 600.0;   // mm, if side sensor bigger than this, wall may be gone

// PD gains
float Kp = 0.7;
float Kd = 1.2;

// Speed settings
int baseSpeed       = 140;   // normal forward speed, 0~255
int minSpeed        = 80;
int maxSpeed        = 220;
int turnSpeed       = 140;   // turning speed when obstacle ahead
int sharpTurnOffset = 90;    // stronger turning amount for obstacle avoidance

// Direction switching hysteresis
// If right is clearly smaller -> follow right wall
// If left is clearly smaller  -> follow left wall
float switchMargin = 40.0;   // mm

// Filter
float alpha = 0.35;          // exponential moving average factor

// Loop timing
unsigned long loopPeriodMs = 30;

// =====================================================
// ================== GLOBAL OBJECTS ====================
// =====================================================

Adafruit_VL53L0X loxLeft  = Adafruit_VL53L0X();
Adafruit_VL53L0X loxFront = Adafruit_VL53L0X();
Adafruit_VL53L0X loxRight = Adafruit_VL53L0X();

VL53L0X_RangingMeasurementData_t measureLeft;
VL53L0X_RangingMeasurementData_t measureFront;
VL53L0X_RangingMeasurementData_t measureRight;

// Filtered distance values
float dLeftFilt  = 300.0;
float dFrontFilt = 300.0;
float dRightFilt = 300.0;

// true  -> clockwise wall following (follow right wall)
// false -> counterclockwise wall following (follow left wall)
bool followRightWall = true;

float prevError = 0.0;
unsigned long prevTimeMs = 0;

// =====================================================
// ================= HELPER FUNCTIONS ===================
// =====================================================

void stopMotors() {
  digitalWrite(L_IN1, LOW);
  digitalWrite(L_IN2, LOW);
  digitalWrite(R_IN1, LOW);
  digitalWrite(R_IN2, LOW);
  ledcWrite(L_CH, 0);
  ledcWrite(R_CH, 0);
}

void setLeftMotor(int speedCmd) {
  speedCmd = constrain(speedCmd, -255, 255);

  if (speedCmd > 0) {
    digitalWrite(L_IN1, HIGH);
    digitalWrite(L_IN2, LOW);
    ledcWrite(L_CH, speedCmd);
  } else if (speedCmd < 0) {
    digitalWrite(L_IN1, LOW);
    digitalWrite(L_IN2, HIGH);
    ledcWrite(L_CH, -speedCmd);
  } else {
    digitalWrite(L_IN1, LOW);
    digitalWrite(L_IN2, LOW);
    ledcWrite(L_CH, 0);
  }
}

void setRightMotor(int speedCmd) {
  speedCmd = constrain(speedCmd, -255, 255);

  if (speedCmd > 0) {
    digitalWrite(R_IN1, HIGH);
    digitalWrite(R_IN2, LOW);
    ledcWrite(R_CH, speedCmd);
  } else if (speedCmd < 0) {
    digitalWrite(R_IN1, LOW);
    digitalWrite(R_IN2, HIGH);
    ledcWrite(R_CH, -speedCmd);
  } else {
    digitalWrite(R_IN1, LOW);
    digitalWrite(R_IN2, LOW);
    ledcWrite(R_CH, 0);
  }
}

void setDrive(int leftCmd, int rightCmd) {
  leftCmd  = constrain(leftCmd,  -255, 255);
  rightCmd = constrain(rightCmd, -255, 255);
  setLeftMotor(leftCmd);
  setRightMotor(rightCmd);
}

float ema(float oldVal, float newVal, float a) {
  return a * newVal + (1.0 - a) * oldVal;
}

float readSingleRangeMM(Adafruit_VL53L0X &sensor, VL53L0X_RangingMeasurementData_t &m) {
  sensor.rangingTest(&m, false);

  if (m.RangeStatus == 4) {
    // out of range
    return 1200.0;
  }

  if (m.RangeMilliMeter <= 0) {
    return 1200.0;
  }

  return (float)m.RangeMilliMeter;
}

bool initSensorWithAddress(Adafruit_VL53L0X &sensor, uint8_t addr) {
  if (!sensor.begin(addr, false, &Wire)) {
    return false;
  }
  return true;
}

bool initThreeToFs() {
  // Hold all sensors in shutdown first
  pinMode(XSHUT_LEFT, OUTPUT);
  pinMode(XSHUT_FRONT, OUTPUT);
  pinMode(XSHUT_RIGHT, OUTPUT);

  digitalWrite(XSHUT_LEFT, LOW);
  digitalWrite(XSHUT_FRONT, LOW);
  digitalWrite(XSHUT_RIGHT, LOW);
  delay(50);

  // LEFT
  digitalWrite(XSHUT_LEFT, HIGH);
  delay(50);
  if (!initSensorWithAddress(loxLeft, ADDR_LEFT)) {
    Serial.println("Failed to boot LEFT VL53L0X");
    return false;
  }
  Serial.println("LEFT VL53L0X started");

  // FRONT
  digitalWrite(XSHUT_FRONT, HIGH);
  delay(50);
  if (!initSensorWithAddress(loxFront, ADDR_FRONT)) {
    Serial.println("Failed to boot FRONT VL53L0X");
    return false;
  }
  Serial.println("FRONT VL53L0X started");

  // RIGHT
  digitalWrite(XSHUT_RIGHT, HIGH);
  delay(50);
  if (!initSensorWithAddress(loxRight, ADDR_RIGHT)) {
    Serial.println("Failed to boot RIGHT VL53L0X");
    return false;
  }
  Serial.println("RIGHT VL53L0X started");

  return true;
}

void updateFollowDirection(float leftMM, float rightMM) {
  // Hysteresis to avoid rapid flipping
  if (rightMM + switchMargin < leftMM) {
    followRightWall = true;   // clockwise
  } else if (leftMM + switchMargin < rightMM) {
    followRightWall = false;  // counterclockwise
  }
}

void printDebug(float leftMM, float frontMM, float rightMM, float error, float control) {
  Serial.print("L: ");
  Serial.print(leftMM);
  Serial.print("  F: ");
  Serial.print(frontMM);
  Serial.print("  R: ");
  Serial.print(rightMM);
  Serial.print("  follow: ");
  Serial.print(followRightWall ? "RIGHT/CW" : "LEFT/CCW");
  Serial.print("  err: ");
  Serial.print(error);
  Serial.print("  ctrl: ");
  Serial.println(control);
}

// =====================================================
// ====================== SETUP =========================
// =====================================================

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n3-ToF Wall Following Start");

  Wire.begin();
  Wire.setClock(400000);

  // Motor pins
  pinMode(L_IN1, OUTPUT);
  pinMode(L_IN2, OUTPUT);
  pinMode(R_IN1, OUTPUT);
  pinMode(R_IN2, OUTPUT);

  ledcSetup(L_CH, PWM_FREQ, PWM_RES);
  ledcSetup(R_CH, PWM_FREQ, PWM_RES);
  ledcAttachPin(L_PWM, L_CH);
  ledcAttachPin(R_PWM, R_CH);

  stopMotors();

  if (!initThreeToFs()) {
    Serial.println("Sensor init failed. Motors stopped.");
    while (1) {
      stopMotors();
      delay(100);
    }
  }

  // Initial read
  delay(100);
  float dL = readSingleRangeMM(loxLeft, measureLeft);
  float dF = readSingleRangeMM(loxFront, measureFront);
  float dR = readSingleRangeMM(loxRight, measureRight);

  dLeftFilt  = dL;
  dFrontFilt = dF;
  dRightFilt = dR;

  updateFollowDirection(dLeftFilt, dRightFilt);

  prevTimeMs = millis();
}

// =====================================================
// ======================= LOOP =========================
// =====================================================

void loop() {
  static unsigned long lastLoop = 0;
  unsigned long now = millis();

  if (now - lastLoop < loopPeriodMs) {
    return;
  }
  lastLoop = now;

  // ---------------- Read sensors ----------------
  float dL = readSingleRangeMM(loxLeft, measureLeft);
  float dF = readSingleRangeMM(loxFront, measureFront);
  float dR = readSingleRangeMM(loxRight, measureRight);

  dLeftFilt  = ema(dLeftFilt,  dL, alpha);
  dFrontFilt = ema(dFrontFilt, dF, alpha);
  dRightFilt = ema(dRightFilt, dR, alpha);

  // ---------------- Decide direction ----------------
  updateFollowDirection(dLeftFilt, dRightFilt);

  // ---------------- Timing for derivative ----------------
  float dt = (now - prevTimeMs) / 1000.0;
  if (dt <= 0.0) dt = 0.001;
  prevTimeMs = now;

  float error = 0.0;
  float deriv = 0.0;
  float control = 0.0;

  // =====================================================
  // FRONT OBSTACLE CASE
  // =====================================================
  if (dFrontFilt < frontStopDist) {
    // If following right wall (clockwise), turn left
    // If following left wall (counterclockwise), turn right
    if (followRightWall) {
      setDrive(baseSpeed - sharpTurnOffset, baseSpeed + sharpTurnOffset);
    } else {
      setDrive(baseSpeed + sharpTurnOffset, baseSpeed - sharpTurnOffset);
    }

    printDebug(dLeftFilt, dFrontFilt, dRightFilt, error, control);
    return;
  }

  // =====================================================
  // WALL FOLLOW CONTROL
  // =====================================================
  if (followRightWall) {
    // clockwise: follow right wall
    // positive error => too far from right wall? let's define carefully:
    // error = desired - measured
    // if measured too small => positive -> steer left
    error = desiredWallDist - dRightFilt;

    // if wall is lost, gently curve right to search again
    if (dRightFilt > wallLostDist) {
      control = -50;  // negative means turn right in our mapping below
    } else {
      deriv = (error - prevError) / dt;
      control = Kp * error + Kd * deriv;
    }

  } else {
    // counterclockwise: follow left wall
    // for symmetry:
    // if too close to left wall, want to steer right
    error = dLeftFilt - desiredWallDist;

    // if wall is lost, gently curve left to search again
    if (dLeftFilt > wallLostDist) {
      control = 50;   // positive means turn left in our mapping below
    } else {
      deriv = (error - prevError) / dt;
      control = Kp * error + Kd * deriv;
    }
  }

  prevError = error;

  // =====================================================
  // MAP CONTROL TO MOTOR COMMANDS
  // positive control  -> turn left
  // negative control  -> turn right
  // =====================================================
  control = constrain(control, -120, 120);

  int leftCmd  = baseSpeed - (int)control;
  int rightCmd = baseSpeed + (int)control;

  leftCmd  = constrain(leftCmd,  minSpeed, maxSpeed);
  rightCmd = constrain(rightCmd, minSpeed, maxSpeed);

  setDrive(leftCmd, rightCmd);

  printDebug(dLeftFilt, dFrontFilt, dRightFilt, error, control);
}