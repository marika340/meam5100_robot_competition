// =====================================================================
// COMBINED: Webpage PID Control + Autonomous Wall Following
// 
// State machine:
//   WEBPAGE_CONTROL  -> Manual drive via web UI with PID
//   TRANSITION       -> Brief stop + reset when wall detected
//   WALL_FOLLOWING   -> Autonomous 3-ToF wall following
//
// Transition trigger: front ToF reads < wallEngageDist mm
// You can also force WEBPAGE_CONTROL from web UI via /mode=0
// =====================================================================

#include <WiFi.h>
#include <WiFiUdp.h>
#include "html510.h"
#include "PIDandTOF_web.h"
#include <Encoder.h>
#include <Wire.h>
#include "Adafruit_VL53L0X.h"
#include "Adafruit_VL53L1X.h"

// =====================================================================
// WIFI CONFIG
// =====================================================================
HTML510Server h(80);
const char* ssid     = "iPhone";
const char* password = "anhduong";
IPAddress myIP   (192,168,1,105);
IPAddress gateway(192,168,1,1);
IPAddress subnet (255,255,255,0);

// =====================================================================
// MOTOR PINS  (webpage-control side)
// =====================================================================
Encoder myEnc[] = { {35,36}, {34,33} };
int Hpin[]      = {1, 2};
int Hpin_dir1[] = {42, 40};
int Hpin_dir2[] = {41, 39};
int enc_pin_a[] = {34, 36};
int enc_pin_b[] = {33, 35};
int pwm_channels[] = {0, 1};

// PWM / encoder constants
int   encoder_slot    = 12;
int   frequency       = 500;
int   resolution_bit  = 14;
int   resolution      = ((1 << resolution_bit) - 1);
int   gear_ratio      = 34;
float count_per_revolution = (12.0 * 4 * gear_ratio);

// PID state
float Kp = 1.4, Ki = 1.0, Kd = 0.0;
float rpm_desired = 85.0;
float rpm_new     = 0.0;
float motor_speed[]     = {0, 0};
int   motor_direction[] = {0, 0};
float integral[]        = {0, 0};
float previous_error[]  = {0, 0};
long  previous_count[]  = {0, 0};
long  prev_manual_count[] = {0, 0};
unsigned long last_pid_time  = 0;
unsigned long last_time      = 0;

// Manual→PID transition vars
float current_manual_rpm[2] = {0, 0};
bool  auto_enable    = false;
bool  pid_enable     = false;
unsigned long manual_start_time = 0;
float pid_transition = 0.90;

// =====================================================================
// TOF SENSOR PINS  (wall following side)
// =====================================================================
#define XSHUT_LEFT   19
#define XSHUT_FRONT  10
#define XSHUT_RIGHT  18

#define ADDR_LEFT    0x30
#define ADDR_FRONT   0x31
#define ADDR_RIGHT   0x32

Adafruit_VL53L0X loxLeft  = Adafruit_VL53L0X(); //RANGE UP TO 1000 MM
Adafruit_VL53L1X loxFront = Adafruit_VL53L1X(); //RANGE UP TO 4000 MM
Adafruit_VL53L0X loxRight = Adafruit_VL53L0X(); //RANGE UP TO 1000 MM

VL53L0X_RangingMeasurementData_t measureLeft;
//VL53L1X_RangingMeasurementData_t measureFront; DO NOT MEASURE LIKE VL53L0X
VL53L0X_RangingMeasurementData_t measureRight;

float dLeftFilt  = 300.0;
float dFrontFilt = 300.0;
float dRightFilt = 300.0;
float alpha      = 0.35;

// Wall-follow parameters
float desiredWallDist = 180.0;  // mm
float frontStopDist   = 160.0;  // mm — hard-stop during wall following
float wallLostDist    = 600.0;  // mm
float wallEngageDist  = 250.0;  // mm — front distance that triggers auto mode
float switchMargin    = 40.0;

float wf_Kp = 0.7, wf_Kd = 1.2;
int   baseSpeed       = 140;
int   minSpeed        = 80;
int   maxSpeed        = 220;
int   sharpTurnOffset = 90;

bool  followRightWall = true;
float wf_prevError    = 0.0;
unsigned long wf_prevTimeMs = 0;
unsigned long loopPeriodMs  = 30;

// =====================================================================
// OPERATING MODE
// =====================================================================
enum CarMode { WEBPAGE_CONTROL, TRANSITION, WALL_FOLLOWING, CENTERING, PRESSING_BUTTON };
CarMode carMode = WEBPAGE_CONTROL;

// =====================================================================
// LOW-LEVEL MOTOR HELPERS
// =====================================================================

// Used by webpage-control PID
void ledcAnalogWrite(uint8_t pin, uint32_t value, uint32_t value_max) {
  uint32_t duty = map(value, 0, value_max, 0, resolution);
  ledcWrite(pin, duty);
}

void motor(int i) {
  uint32_t pwm_value = (uint32_t)constrain(motor_speed[i], 0, resolution);
  if (motor_direction[i] == -1) {
    digitalWrite(Hpin_dir1[i], HIGH);
    digitalWrite(Hpin_dir2[i], LOW);
    ledcWrite(Hpin[i], pwm_value);
  } else if (motor_direction[i] == 1) {
    digitalWrite(Hpin_dir1[i], LOW);
    digitalWrite(Hpin_dir2[i], HIGH);
    ledcWrite(Hpin[i], pwm_value);
  } else {
    digitalWrite(Hpin_dir1[i], LOW);
    digitalWrite(Hpin_dir2[i], LOW);
    ledcWrite(Hpin[i], 0);
  }
}

void stopAllMotors() {
  for (int i = 0; i < 2; i++) {
    motor_direction[i] = 0;
    motor_speed[i]     = 0;
    motor(i);
  }
}

// Used by wall-following (raw PWM, separate from PID state)
void setLeftMotorRaw(int speedCmd) {
  speedCmd = constrain(speedCmd, -255, 255);
  if (speedCmd > 0) {
    digitalWrite(Hpin_dir1[0], LOW);
    digitalWrite(Hpin_dir2[0], HIGH);
    ledcWrite(Hpin[0], map(speedCmd, 0, 255, 0, resolution));
  } else if (speedCmd < 0) {
    digitalWrite(Hpin_dir1[0], HIGH);
    digitalWrite(Hpin_dir2[0], LOW);
    ledcWrite(Hpin[0], map(-speedCmd, 0, 255, 0, resolution));
  } else {
    digitalWrite(Hpin_dir1[0], LOW);
    digitalWrite(Hpin_dir2[0], LOW);
    ledcWrite(Hpin[0], 0);
  }
}

void setRightMotorRaw(int speedCmd) {
  speedCmd = constrain(speedCmd, -255, 255);
  if (speedCmd > 0) {
    digitalWrite(Hpin_dir1[1], LOW);
    digitalWrite(Hpin_dir2[1], HIGH);
    ledcWrite(Hpin[1], map(speedCmd, 0, 255, 0, resolution));
  } else if (speedCmd < 0) {
    digitalWrite(Hpin_dir1[1], HIGH);
    digitalWrite(Hpin_dir2[1], LOW);
    ledcWrite(Hpin[1], map(-speedCmd, 0, 255, 0, resolution));
  } else {
    digitalWrite(Hpin_dir1[1], LOW);
    digitalWrite(Hpin_dir2[1], LOW);
    ledcWrite(Hpin[1], 0);
  }
}

void setDriveRaw(int leftCmd, int rightCmd) {
  setLeftMotorRaw(constrain(leftCmd,  -255, 255));
  setRightMotorRaw(constrain(rightCmd, -255, 255));
}

// =====================================================================
// TOF HELPERS
// =====================================================================
float ema(float oldVal, float newVal, float a) {
  return a * newVal + (1.0 - a) * oldVal;
}

float readSingleRangeMM(Adafruit_VL53L0X &sensor, VL53L0X_RangingMeasurementData_t &m) {
  sensor.rangingTest(&m, false);
  if (m.RangeStatus == 4 || m.RangeMilliMeter <= 0) return 1200.0;
  return (float)m.RangeMilliMeter;
}

bool initSensorWithAddress(Adafruit_VL53L0X &sensor, uint8_t addr) {
  return sensor.begin(addr, false, &Wire);
}

bool initThreeToFs() {
  pinMode(XSHUT_LEFT,  OUTPUT);
  pinMode(XSHUT_FRONT, OUTPUT);
  pinMode(XSHUT_RIGHT, OUTPUT);
  digitalWrite(XSHUT_LEFT,  LOW);
  digitalWrite(XSHUT_FRONT, LOW);
  digitalWrite(XSHUT_RIGHT, LOW);
  delay(50);

  digitalWrite(XSHUT_LEFT, HIGH); delay(50);
  if (!initSensorWithAddress(loxLeft, ADDR_LEFT)) {
    Serial.println("Failed: LEFT VL53L0X"); return false;
  }
  Serial.println("LEFT VL53L0X OK");

  digitalWrite(XSHUT_FRONT, HIGH); delay(50);
  if (!loxFront.begin(ADDR_FRONT)) { //DO NOT NEED INIT SENSOR HELPER FOR VL53L1X
    Serial.println("Failed: FRONT VL53L0X"); return false;
  }
  loxFront.startRanging(); //START MEASURE HERE
  Serial.println("FRONT VL53L1X OK");

  digitalWrite(XSHUT_RIGHT, HIGH); delay(50);
  if (!initSensorWithAddress(loxRight, ADDR_RIGHT)) {
    Serial.println("Failed: RIGHT VL53L0X"); return false;
  }
  Serial.println("RIGHT VL53L0X OK");

  return true;
}

void readAndFilterToFs() {
  //HANDLE L0X SENSOR
  dLeftFilt  = ema(dLeftFilt,  readSingleRangeMM(loxLeft,  measureLeft),  alpha);
  //dFrontFilt = ema(dFrontFilt, readSingleRangeMM(loxFront, measureFront), alpha);
  dRightFilt = ema(dRightFilt, readSingleRangeMM(loxRight, measureRight), alpha);

  //HANDLE L1X SENSOR
  if (loxFront.dataReady()) { //USE FUNCTION FROM THE LIBRARY
    float front_read = (float)loxFront.distance();
    if (front_read > 0){
      dFrontFilt = ema(dFrontFilt, front_read, alpha);
    }
    loxFront.clearInterrupt();
  }
}

void updateFollowDirection(float leftMM, float rightMM) {
  if      (rightMM + switchMargin < leftMM) followRightWall = true;
  else if (leftMM  + switchMargin < rightMM) followRightWall = false;
}

// =====================================================================
// WEBPAGE-CONTROL PID LOGIC  (unchanged from original)
// =====================================================================

void PIDcontrol() {
  unsigned long current = millis();
  float dt = (current - last_pid_time) / 1000.0;
  float current_rpms[2] = {0, 0};

  if (dt >= 0.100) {
    for (int i = 0; i < 2; i++) {
      long current_count = myEnc[i].read();
      float current_rpm  = abs(((current_count - previous_count[i]) /
                               (float)count_per_revolution) * (60.0 / dt));
      current_rpms[i] = current_rpm;

      if (motor_direction[i] == 0) {
        motor_speed[i] = 0; integral[i] = 0; motor(i); continue;
      }

      float individual_target = abs(rpm_desired);
      float error  = individual_target - current_rpm;
      float output = Kp * error;
      integral[i]  = constrain(integral[i] + (error * dt), -50, 50);
      float output_Ki  = Ki * integral[i];
      float derivative = (error - previous_error[i]) / dt;
      float output_Kd  = Kd * derivative;
      float control_output = output + output_Ki + output_Kd;
      motor_speed[i] = constrain(control_output * (resolution / 130.0), 0, resolution);

      Serial.printf("M%d | Target:%.1f | Curr:%.1f | Err:%.1f | Out:%.1f | Dir:%d | PWM:%.0f\n",
                    i, individual_target, current_rpm, error, control_output,
                    motor_direction[i], motor_speed[i]);

      previous_count[i] = current_count;
      previous_error[i] = error;
    }

    // Motor sync
    if (motor_direction[0] != 0 && motor_direction[1] != 0) {
      float sync_error = current_rpms[0] - current_rpms[1];
      float sync_gain  = 0.005;
      motor_speed[0] = constrain(motor_speed[0] - sync_gain * (resolution / 130.0) * sync_error, 0, resolution);
      motor_speed[1] = constrain(motor_speed[1] + sync_gain * (resolution / 130.0) * sync_error, 0, resolution);
      float integral_diff = integral[0] - integral[1];
      integral[0] -= 0.1 * integral_diff;
      integral[1] += 0.1 * integral_diff;
    }

    motor(0); motor(1);
    last_pid_time = current;
  }
}

void manualMode() {
  if (millis() - last_time >= 100) {
    long now = millis();
    float dt_manual = (now - last_time) / 1000.0;
    for (int i = 0; i < 2; i++) {
      long current_manual_count = myEnc[i].read();
      float count = current_manual_count - prev_manual_count[i];
      current_manual_rpm[i] = abs((count / count_per_revolution) * (60.0 / dt_manual));
      prev_manual_count[i]  = current_manual_count;
      Serial.printf("MOTOR %d | Manual RPM: %.1f | Dir: %d  ", i, current_manual_rpm[i], motor_direction[i]);
    }
    Serial.println();
    last_time = now;
  }
}

void autoMode() {
  if (abs(rpm_desired) < 1.0) {
    pid_enable = false; auto_enable = false;
    for (int i = 0; i < 2; i++) {
      motor_speed[i] = 0; integral[i] = 0; previous_error[i] = 0; motor(i);
    }
    return;
  }

  if (pid_enable) {
    PIDcontrol();
  } else {
    manualMode();
    if (auto_enable && abs(rpm_desired) > 0) {
      bool m0ok    = abs(current_manual_rpm[0]) >= abs(rpm_desired * pid_transition);
      bool m1ok    = abs(current_manual_rpm[1]) >= abs(rpm_desired * pid_transition);
      bool timeout = (millis() - manual_start_time > 1000);

      if ((m0ok && m1ok) || timeout) {
        Serial.println(">>> PID engaged.");
        for (int i = 0; i < 2; i++) {
          float current_error = abs(rpm_desired) - current_manual_rpm[i];
          integral[i] = (Ki > 0)
            ? (map(constrain(abs(rpm_new), 0, 130), 0, 130, 0, resolution) - (Kp * current_error)) / Ki
            : 0;
          previous_error[i]  = current_error;
          previous_count[i]  = myEnc[i].read();
        }
        pid_enable  = true;
        auto_enable = false;
      }
    }
  }
}

// =====================================================================
// WALL-FOLLOWING LOGIC
// =====================================================================
void handleSharpTurn() {
  sharpTurnOffset = (int)constrain(h.getVal(), 0, 255);
  Serial.printf("sharpTurnOffset: %d\n", sharpTurnOffset);
  h.sendhtml(body);
}

//ADD A HANDLE CORNER FUNCTION
void handleCorner(bool LeftTurn) {
  setDriveRaw(0,0); delay(100); //TURN OFF MOTORS BEFORE ACTION - GOOD PRACTICE!
  if (LeftTurn) {
    while (dFrontFilt < frontStopDist) { // TURN UNTIL CLEAR OBSTACLE
    setDriveRaw(-120,120); //PIVOT TO THE LEFT
    readAndFilterToFs();
    delay(500);
    }
    } else {
    setDriveRaw(baseSpeed,baseSpeed); //OR GO AT THE CURRENT SPEED
    delay(300);
    setDriveRaw(120,-120); //PIVOT TO THE RIGHT
    delay(500);
  }
  wf_prevError = 0.0; //RESET PID ERROR
}
//HANDLE WALL FOLLOWING
void runWallFollowing() {
  static unsigned long lastLoop = 0;
  unsigned long now = millis();
  if (now - lastLoop < loopPeriodMs) return;
  lastLoop = now;

  readAndFilterToFs();
  // updateFollowDirection(dLeftFilt, dRightFilt);

  float dt = (now - wf_prevTimeMs) / 1000.0;
  if (dt <= 0.0) dt = 0.001;
  wf_prevTimeMs = now;

  float error = 0.0, deriv = 0.0, control = 0.0;
  //HANDLE CORNERS
  // Hard obstacle ahead — turn away
  if (dFrontFilt < frontStopDist) {
    if (followRightWall)
      setDriveRaw(baseSpeed - sharpTurnOffset, baseSpeed + sharpTurnOffset);
    else
      setDriveRaw(baseSpeed + sharpTurnOffset, baseSpeed - sharpTurnOffset);

    // Serial.printf("[WF] Obstacle! L:%.0f F:%.0f R:%.0f\n", dLeftFilt, dFrontFilt, dRightFilt);
    Serial.printf("[WF] Obstacle! F:%.0f R:%.0f\n", dFrontFilt, dRightFilt);
    return;
  }

  if (followRightWall) {
    error = desiredWallDist - dRightFilt;
    if (dRightFilt > wallLostDist) {
      control = -50; // curve right to search
    } else {
      deriv   = (error - wf_prevError) / dt;
      control = wf_Kp * error + wf_Kd * deriv;
    }
  } else {
    //Serial.printf("followRightWall = false");
    error = dLeftFilt - desiredWallDist;
    if (dLeftFilt > wallLostDist) {
      control = 50; // curve left to search
    } else {
      deriv   = (error - wf_prevError) / dt;
      control = wf_Kp * error + wf_Kd * deriv;
    }
  }

  wf_prevError = error;
  control = constrain(control, -120, 120);

  int leftCmd  = constrain(baseSpeed - (int)control, minSpeed, maxSpeed);
  int rightCmd = constrain(baseSpeed + (int)control, minSpeed, maxSpeed);
  setDriveRaw(leftCmd, rightCmd);

  // Serial.printf("[WF] L:%.0f F:%.0f R:%.0f follow:%s err:%.1f ctrl:%.1f\n",
  //               dLeftFilt, dFrontFilt, dRightFilt,
  //               followRightWall ? "R" : "L", error, control);

  Serial.printf("[WF] F:%.0f R:%.0f follow:%s err:%.1f ctrl:%.1f\n",
                dFrontFilt, dRightFilt,
                followRightWall ? "R" : "L", error, control);
}
//ADD PUSHING BUTTON ACTION -> DETECT BUTTON 
void pressButton() { //NO PID FOR THIS BECAUSE DISTANCE CLOSE ENOUGH
  stopAllMotors();
  //DRIVE FORWARD TO PUSH
  setDriveRaw(80,80);
  delay(500); //DRIVE HALF A SECOND TO REACH BUTTON
  //HOLD POSITION
  setDriveRaw(50,50);
  delay(8000); //HOLD FOR 8 SECONDS
  //GO BACK
  setDriveRaw(-80,-80);
  delay(500);
  stopAllMotors();
}
//CENTER THE CAR AS GETTING NEAR TO THE NEXUS ON THE OTHER SIDE TO PRESS BUTTON
//THIS IS TO CAPTURE OPPONENT NEXUS AND LOWER LEXUS, BY TAKING ADVANTAGE OF SYMMETRICAL FIELD
void runCenteringMode() {
  static unsigned long lastCenteringTime = 0;
  unsigned long now = millis();
  float dt = (now - lastCenteringTime) / 1000.0;
  //RUN PD WHEN NEED
  if (dt < 0.01) return; 
  lastCenteringTime = now;

  readAndFilterToFs();

  //TRY TO DETECT BUTTON ON THE RAMP BY SENSING THE BUTTON THICKNESS
  //WIDTH OF THE RAMP IS ABOUT 19 INCHES (~482 MM)
  //CENTER AT RAMP = 9.5 INCHES (~241 MM)
  // Logic: When we are near the button, we expect a sudden drop in distance 
  // on one side to 50mm (thickness of the button) while the other remains 
  // near the ramp distance (241mm).
  bool buttonRight = (dRightFilt < 70.0 && dLeftFilt > 200.0);
  bool buttonLeft  = (dLeftFilt < 70.0 && dRightFilt > 200.0);

  if (buttonRight || buttonLeft) {
    stopAllMotors();
    delay(200);
    // Pivot 90 degrees to face the button
    if (buttonRight) {
      setDriveRaw(120, -120); // Pivot right
    } else {
      setDriveRaw(-120, 120); // Pivot left
    }
    delay(500); // Adjust this delay FOR THE TURN
    
    stopAllMotors();
    pressButton();
    carMode = WEBPAGE_CONTROL;
    return;
  }

  //CALCULATE CENTERING ERROR
  float totalWidth = dLeftFilt + dRightFilt;
  float targetDist = totalWidth/2.0; 
  //CALCULATE THE TARGET DISTANCE BY USING READINGS FROM BOTH TOF LEFT AND RIGHT
  float centerError = (targetDist - dLeftFilt) - (dRightFilt - targetDist);
  if (abs(centerError) < 10.0) centerError = 0.0; //IF IT IS CENTERED ENOUGH, STOP TUNING
  //ADD PID CONTROL FOR CENTERING
  
  // FIX THE PD LOOP
  float deriv = (centerError - wf_prevError) / dt;
  float control = (wf_Kp * centerError) + (wf_Kd * deriv);
  wf_prevError = centerError;
  //COMPENSATE FOR CORRECTION
  //ADD CONSTRAINT TO PREVENT MOTOR BURN OUT
  control = constrain(control, -100, 100);
  int leftCmd  = baseSpeed + (int)control;
  int rightCmd = baseSpeed - (int)control;
  setDriveRaw(leftCmd, rightCmd);
  //DETECT BUTTON
  if (dFrontFilt < 100) { // Adjust '300MM' based on button distance - OUR LOWEST RANGE IS 30 MM
    stopAllMotors();
    pressButton();
  }
}
// =====================================================================
// WEB HANDLERS
// =====================================================================

void handleRoot()  { h.sendhtml(body); }

void handleDir() {
  String dir = h.getText();
  motor_direction[0] = 0; motor_direction[1] = 0;
  motor(0); motor(1);
  delay(10);

  if      (dir == "F") { motor_direction[0] =  1; motor_direction[1] =  1; }
  else if (dir == "B") { motor_direction[0] = -1; motor_direction[1] = -1; }
  else if (dir == "L") { motor_direction[0] = -1; motor_direction[1] =  1; }
  else if (dir == "R") { motor_direction[0] =  1; motor_direction[1] = -1; }
  else                 { motor_direction[0] =  0; motor_direction[1] =  0; }

  for (int i = 0; i < 2; i++) { integral[i] = 0; previous_error[i] = 0; }
  motor(0); motor(1);
  h.sendhtml(body);
}

void handleSpeed() {
  rpm_new     = h.getVal();
  rpm_desired = constrain(rpm_new, -130, 130);
  manual_start_time = millis();
  pid_enable  = false;
  auto_enable = true;
  for (int i = 0; i < 2; i++) {
    integral[i] = 0; previous_error[i] = 0;
    motor_speed[i] = map(constrain(abs(rpm_new), 0, 130), 0, 130, 0, resolution);
    motor(i);
  }
  Serial.printf("Manual warmup started. Target RPM: %.1f\n", rpm_desired);
  h.sendhtml(body);
}

void handleKp() {
  Kp = h.getVal();
  for (int i = 0; i < 2; i++) { integral[i] = 0; previous_error[i] = 0; }
  Serial.printf("Kp: %.2f\n", Kp);
  h.sendhtml(body);
}

void handleKi() {
  Ki = h.getVal();
  for (int i = 0; i < 2; i++) { integral[i] = 0; previous_error[i] = 0; }
  Serial.printf("Ki: %.2f\n", Ki);
  h.sendhtml(body);
}

void handleKd() {
  Kd = h.getVal();
  for (int i = 0; i < 2; i++) { integral[i] = 0; previous_error[i] = 0; }
  Serial.printf("Kd: %.2f\n", Kd);
  h.sendhtml(body);
}

void handleAuto() {
  auto_enable = (h.getVal() == 1);
  if (!auto_enable) { pid_enable = false; Serial.println("Auto disabled."); }
  else              { Serial.println("Auto enabled."); }
  h.sendhtml(body);
}

// /mode=0 -> force back to WEBPAGE_CONTROL
// /mode=1 -> force into WALL_FOLLOWING
// /mode=2 -> centering
void handleMode() {
  int mode = h.getVal();
  if (mode == 0) {
    carMode = WEBPAGE_CONTROL;
    stopAllMotors();
    pid_enable = false; auto_enable = true; //WANT AUTO MODE WITH MANUAL WARM UP TO PID 
    rpm_desired = 0.0;
    for (int i = 0; i < 2; i++) {
      myEnc[i].write(0);
      prev_manual_count[i] = 0; previous_count[i] = 0;
      integral[i] = 0; previous_error[i] = 0;
    }
    manual_start_time = millis();
    Serial.println("Mode: WEBPAGE_CONTROL (forced)");
  } else if (mode == 1) {
    carMode = WALL_FOLLOWING;
    stopAllMotors();
    wf_prevError  = 0.0;
    wf_prevTimeMs = millis();
    Serial.println("Mode: WALL_FOLLOWING (forced)");
  } else if (mode == 2) {
  carMode = CENTERING;
  stopAllMotors();
  Serial.println("Mode: CENTERING (forced)");
  }
  h.sendhtml(body);
}

void handleWfKp() {
  wf_Kp = h.getVal();
  wf_prevError = 0.0;
  Serial.printf("wf_Kp: %.2f\n", wf_Kp);
  h.sendhtml(body);
}

void handleWfKd() {
  wf_Kd = h.getVal();
  wf_prevError = 0.0;
  Serial.printf("wf_Kd: %.2f\n", wf_Kd);
  h.sendhtml(body);
}

// =====================================================================
// SETUP
// =====================================================================

void setup() {
  Serial.begin(115200);

  // Motor + encoder pin setup
  for (int i = 0; i < 2; i++) {
    pinMode(Hpin_dir1[i], OUTPUT);
    pinMode(Hpin_dir2[i], OUTPUT);
    pinMode(Hpin[i],      OUTPUT);
    pinMode(enc_pin_a[i], INPUT_PULLUP);
    pinMode(enc_pin_b[i], INPUT_PULLUP);
    ledcAttachChannel(Hpin[i], frequency, resolution_bit, pwm_channels[i]);
  }

  // I2C for ToF sensors
  Wire.begin();
  Wire.setClock(400000);

  if (!initThreeToFs()) {
    Serial.println("ToF sensor init failed — will run webpage-only mode.");
    // Continue without wall following available
  } else {
    // Initial filtered reads
    delay(100);
    dLeftFilt  = readSingleRangeMM(loxLeft,  measureLeft);
    //dFrontFilt = readSingleRangeMM(loxFront, measureFront); //HAVE TO USE A DIFFERENT FUNCTION FOR L1X
    dRightFilt = readSingleRangeMM(loxRight, measureRight);
    // updateFollowDirection(dLeftFilt, dRightFilt);
  }
  wf_prevTimeMs = millis();

  // WiFi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) { delay(500); Serial.print("."); }
  Serial.println("\nWiFi connected");
  Serial.println(WiFi.localIP());

  // Web handlers
  h.begin();
  h.attachHandler("/motor_speed=", handleSpeed);
  h.attachHandler("/Kp=",         handleKp);
  h.attachHandler("/Ki=",         handleKi);
  h.attachHandler("/Kd=",         handleKd);
  h.attachHandler("/dir=",        handleDir);
  h.attachHandler("/Auto=",       handleAuto);
  h.attachHandler("/mode=",       handleMode); // 0=webpage, 1=wall_follow
  h.attachHandler("/wf_Kp=",     handleWfKp);
  h.attachHandler("/wf_Kd=",     handleWfKd);
  h.attachHandler("/sharpTurn=", handleSharpTurn);
  h.attachHandler("/",            handleRoot);

  last_pid_time = millis();
  last_time     = millis();

  Serial.println("Starting in WEBPAGE_CONTROL mode.");
  Serial.printf("Front ToF < %.0f mm triggers auto wall-following.\n", wallEngageDist);
}

// =====================================================================
// LOOP — STATE MACHINE
// =====================================================================

void loop() {
  h.serve(); // always serve web requests regardless of mode

  // Always read ToF sensors so the front distance is fresh
  readAndFilterToFs();

  switch (carMode) {

    // ------------------------------------------------------------------
    case WEBPAGE_CONTROL:
      // Check if we're close enough to a wall to hand off
      if (dFrontFilt < wallEngageDist) {
        Serial.printf(">>> Wall detected at %.0f mm — switching to WALL_FOLLOWING\n", dFrontFilt);
        carMode = TRANSITION;
      } else {
        autoMode(); // normal PID / manual loop
      }
      break;

    // ------------------------------------------------------------------
    case TRANSITION:
      // Brief stop, reset wall-follow state, then enter wall-following
      stopAllMotors();
      pid_enable  = false;
      auto_enable = false;
      rpm_desired = 0;
      wf_prevError  = 0.0;
      wf_prevTimeMs = millis();
      // updateFollowDirection(dLeftFilt, dRightFilt);
      if (millis() - transitionStartTime > 300) { //USE THIS IS BETTER THAN USING DELAYS
        transitionStartTime = 0; // Reset for next time
        carMode = WALL_FOLLOWING;
      }
      Serial.println(">>> WALL_FOLLOWING engaged.");
      break;

    // ------------------------------------------------------------------
    case WALL_FOLLOWING:
      runWallFollowing();
      // Optional: uncomment to allow manual override from web UI
      // (user would call /mode=0 to return to webpage control)
      break;
    // ------------------------------------------------------------------
    case CENTERING:
      runCenteringMode();  //CENTER TO CAPTURE LOWER TOWER AND NEXUS
      break;
    // ------------------------------------------------------------------
    case PRESSING_BUTTON:
      pressButton();
      carMode = WEBPAGE_CONTROL; // Return to manual after task
      break;
  }
}
