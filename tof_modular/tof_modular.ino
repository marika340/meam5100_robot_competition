// =====================================================================
// tof_modular — modular OOP refactor of tof_webpage_and_wall_following
//
// File layout (all in this folder so Arduino IDE auto-compiles them):
//   Mode.h                - abstract base class for car operating modes
//   Motor.h/cpp           - one DC motor + encoder
//   PID.h/cpp             - pure PID controller (no motor coupling)
//   Drivetrain.h/cpp      - 2 motors + 2 PIDs + sync (low-level service)
//   ToFArray.h/cpp        - 3 ToF sensors + filtering
//   ManualDrive.h/cpp     - closed-loop RPM Mode driven by web UI
//   WallFollow.h/cpp      - autonomous wall-following Mode
//   WebController.h/cpp   - WiFi + HTTP UI
//
// Centering / press-button kept as small free functions for now;
// promote to Mode subclasses when ready.
// =====================================================================

#include <Arduino.h>
// #include <Wire.h>
#include "Motor.h"
#include "PID.h"
#include "Drivetrain.h"
#include "ToFArray.h"
#include "ManualDrive.h"
#include "WallFollow.h"
#include "WebController.h"
#include "vive.h"

// =====================================================================
// PIN / HARDWARE CONFIG
// =====================================================================

// Motors: index 0 = LEFT, 1 = RIGHT
//   PWM pin, dir1, dir2, encA, encB, ledc channel, freq, resolution, counts per revolution
Motor leftMotor (1,  42, 41, 35, 36, 0,  500, 14, 12.0f * 4 * 34);
Motor rightMotor(2,  40, 39, 34, 33, 1,  500, 14, 12.0f * 4 * 34);

Drivetrain drivetrain(leftMotor, rightMotor); 

// ToF: XSHUT pins + I2C addresses
#define XSHUT_LEFT   18
#define XSHUT_FRONT  17
#define XSHUT_RIGHT  10
#define ADDR_LEFT    0x30
#define ADDR_FRONT   0x29
#define ADDR_RIGHT   0x32
#define viveLeft     20
#define viveRight    19
ToFArray tofs(XSHUT_LEFT, XSHUT_FRONT, XSHUT_RIGHT,
              ADDR_LEFT,  ADDR_FRONT,  ADDR_RIGHT);

// Modes
ManualDrive manualDrive(drivetrain);
WallFollow  wallFollow (drivetrain, tofs);

// Vive
vive leftVive(viveLeft);
vive rightVive(viveRight);

// WiFi
// const char* ssid     = "Junyi's iPhone";
// const char* password = "d6Hc-VSwL-MyCa-P5Hb";
const char* ssid     = "TP-Link_8A8C";
const char* password = "12488674";

// Forward declaration: web -> main mode change callback
void onModeChange(int mode);
WebController web(manualDrive, wallFollow, onModeChange);

// =====================================================================
// SUPERVISOR / MAIN STATE MACHINE
// =====================================================================
enum CarMode { MANUAL_DRIVE, TRANSITION, WALL_FOLLOWING, CENTERING, PRESSING_BUTTON };
CarMode carMode = MANUAL_DRIVE;

// TRANSITION timing + destination
//   pendingMode is what TRANSITION will hand off to when its timer
//   elapses. Lets us reuse TRANSITION for any future drive-mode →
//   drive-mode handoff (e.g. hardcoded path → wall-follow).
unsigned long transitionStartMs = 0;
CarMode pendingMode = MANUAL_DRIVE;

// Pointer to currently-active Mode (nullptr for non-Mode states like
// TRANSITION / CENTERING / PRESSING_BUTTON, which still live as free
// functions).
Mode* currentMode = nullptr;

// =====================================================================
// MODE HELPERS
// =====================================================================
static void enterMode(CarMode next) {
  if (carMode == next) return;
  // Universal onExit + safety stop. onExit calls drivetrain.stop() too,
  // but the redundancy is harmless and keeps non-Mode states consistent.
  if (currentMode) {
    currentMode->onExit();
    currentMode = nullptr;
  }
  drivetrain.stop();

  carMode = next;
  switch (carMode) {
    case MANUAL_DRIVE:
      currentMode = &manualDrive;
      currentMode->onEnter();
      break;
    case TRANSITION:
      transitionStartMs = millis();
      Serial.println("Mode: TRANSITION");
      break;
    case WALL_FOLLOWING:
      currentMode = &wallFollow;
      currentMode->onEnter();
      break;
    case CENTERING:
      Serial.println("Mode: CENTERING");
      break;
    case PRESSING_BUTTON:
      Serial.println("Mode: PRESSING_BUTTON");
      break;
  }
}

// Helper: brief stop, then hand off to dest mode.
static void enterTransitionTo(CarMode dest) {
  pendingMode = dest;
  enterMode(TRANSITION);
}

// Web /mode= callback
void onModeChange(int mode) {
  switch (mode) {
    case 0: enterMode(MANUAL_DRIVE);             break;
    case 1: enterTransitionTo(WALL_FOLLOWING);   break;
    case 2: enterTransitionTo(CENTERING);        break;
    default: break;
  }
}

// =====================================================================
// SETUP
// =====================================================================
void setup() {
  Serial.begin(115200);

  leftVive.begin();
  rightVive.begin();

  // Hardware
  drivetrain.begin();

  Wire.begin();
  Wire.setClock(400000);
  if (!tofs.begin()) {
    Serial0.println("ToF init failed — webpage-only mode.");
  } else {
    delay(100);
    tofs.primeFilters();
  }

  // WiFi + handlers
  web.begin(ssid, password);

  // Boot directly into ManualDrive. carMode is already MANUAL_DRIVE so
  // enterMode would short-circuit; activate the Mode explicitly.
  currentMode = &manualDrive;
  manualDrive.onEnter();

  Serial0.println("Starting in MANUAL_DRIVE mode.");
  Serial0.println("Wall-following engages only via web /mode=1.");
}

// =====================================================================
// LOOP
// =====================================================================
void loop() {
  web.serve();           // always serve HTTP
  tofs.update();         // always read distances

  vive::Position leftPos  = leftVive.callibrate();
  vive::Position rightPos = rightVive.callibrate();
  Serial0.printf("Left: X %.1f, Left: Y %.1f\n",  leftPos.x,  leftPos.y);
  Serial0.printf("Right: X %.1f, Right: Y %.1f\n", rightPos.x, rightPos.y);


  switch (carMode) {
    case MANUAL_DRIVE:
      if (currentMode) currentMode->update();
      break;

    case TRANSITION:
      // Brief non-blocking stop, then hand off to whatever mode requested it.
      drivetrain.stop();
      if (millis() - transitionStartMs > 300) {
        enterMode(pendingMode);
      }
      break;

    case WALL_FOLLOWING:
      if (currentMode) currentMode->update();
      break;

    case CENTERING:
      runCentering();
      break;

    case PRESSING_BUTTON:
      runPressButton();
      enterMode(MANUAL_DRIVE);
      break;
  }
}

// =====================================================================
// LEGACY MODE STUBS — keep behaviour until promoted to Mode subclasses
// =====================================================================

// Drives forward briefly, holds, retreats. Used after CENTERING locks on.
static void runPressButton() {
  drivetrain.stop();
  drivetrain.driveDirect( 80,  80); delay(500);   // approach
  drivetrain.driveDirect( 50,  50); delay(8000);  // hold for 8s
  drivetrain.driveDirect(-80, -80); delay(500);   // retreat
  drivetrain.stop();
}

// Quick-and-dirty centering using a local PD on (right - left).
// Promote to a Mode subclass when you're ready.
static void runCentering() {
  static unsigned long lastMs = 0;
  static PID centerPid(0.7f, 0.0f, 1.2f, 1000.0f);
  unsigned long now = millis();
  float dt = (now - lastMs) / 1000.0f;
  if (dt < 0.01f) return;
  lastMs = now;

  // Detect button by sudden one-sided drop (ramp width ~482 mm; button thickness ~50 mm).
  bool buttonRight = (tofs.right() < 70.0f && tofs.left()  > 200.0f);
  bool buttonLeft  = (tofs.left()  < 70.0f && tofs.right() > 200.0f);
  if (buttonRight || buttonLeft) {
    drivetrain.stop(); delay(200);
    if (buttonRight) drivetrain.driveDirect( 120, -120);
    else             drivetrain.driveDirect(-120,  120);
    delay(500);
    drivetrain.stop();
    runPressButton();
    enterMode(MANUAL_DRIVE);
    return;
  }

  // PD on lateral error: positive => closer to right wall, steer left.
  float err = tofs.right() - tofs.left();
  if (fabsf(err) < 10.0f) err = 0.0f;
  float ctrl = centerPid.compute(0.0f, -err, dt);
  ctrl = constrain(ctrl, -100.0f, 100.0f);

  int baseSpeed = 140;
  drivetrain.driveDirect(baseSpeed + (int)ctrl, baseSpeed - (int)ctrl);

  if (tofs.front() < 100.0f) {
    drivetrain.stop();
    runPressButton();
    enterMode(MANUAL_DRIVE);
  }
}
