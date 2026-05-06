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
//   Centering.h/cpp       - PD wall-centering Mode (open-ended OR auto-stop)
//   PressTower.h/cpp      - non-blocking approach/hold/retreat Mode
//   LowTower.h/cpp        - composite Mode: straight -> rotate x2 -> center -> press
//   WebController.h/cpp   - WiFi + HTTP UI

#include <Arduino.h>
#include <Wire.h>
#include "Motor.h"
#include "PID.h"
#include "Drivetrain.h"
#include "ToFArray.h"
#include "ManualDrive.h"
#include "WallFollow.h"
#include "Centering.h"
#include "PressTower.h"
#include "LowTower.h"
#include "AttackNexus.h"
#include "AttackTopTower.h"
#include "WebController.h"
#include "RobotPosition.h"
#include "TopHat.h"
#include "Attacker.h"

// PIN / HARDWARE CONFIG

// Motors: index 0 = LEFT, 1 = RIGHT
//   PWM pin, dir1, dir2, encA, encB, ledc channel, freq, resolution, counts per revolution
// JGA25-370 12V 400RPM: 11 PPR/channel * 4 quadrature edges * 21:1 gearbox = 924 counts/output rev
Motor leftMotor (1,  42, 41, 35, 36, 0,  500, 14, 11.0f * 4 * 21);
Motor rightMotor(2,  40, 39, 34, 33, 1,  500, 14, 11.0f * 4 * 21);


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
Centering   centering  (drivetrain, tofs);
// PressTower(drivetrain, approachMs, holdMs, retreatMs)
PressTower  towerPress    (drivetrain,  500, 8000,  500);   // long hold for low tower
PressTower  nexusPress    (drivetrain,  500, 1500,  500);   // shorter hold for nexus
PressTower  topTowerPress (drivetrain,  500, 8500,  500);   // top tower: 8.5 s hold
// LowTower composes drivetrain + Centering + PressTower (tower variant).
//   straightInches=9*12, frontStopMm=100, rotateDir=0 (CW)
LowTower    lowTower   (drivetrain, centering, towerPress);
// AttackNexus composes drivetrain + Centering + PressTower (nexus variant).
//   straightInches=9*12 (108"), frontStopMm=100, pressCount=4
AttackNexus attackNexus(drivetrain, centering, nexusPress);

// Robot Position
RobotPosition robotPos(viveLeft, viveRight);

// AttackTopTower composes drivetrain + WallFollow + RobotPosition + ToFArray
// + PressTower (top-tower variant, 8.5 s hold). Uses Vive coordinates to
// detect when the robot reaches the bridge trigger location, then turns
// 90 CCW, drives forward until front ToF < 100 mm, and presses.
AttackTopTower attackTopTower(drivetrain, wallFollow, robotPos, tofs, topTowerPress);


// WiFi
// const char* ssid     = "Junyi's iPhone";
// const char* password = "d6Hc-VSwL-MyCa-P5Hb";
const char* ssid     = "TP-Link_8A8C";
const char* password = "12488674";

// Forward declaration: web -> main mode change callback
void onModeChange(int mode);
WebController web(manualDrive, wallFollow, onModeChange);

// Top Hat Packer Updater
int packetCounter = 0;
uint8_t health = 0;

// Attack Arm Hardware
Attacker arm(15, 2, 1000); // Pin 15, Channel 2, 1000ms window

// SUPERVISOR / MAIN STATE MACHINE
//
// Most car modes are now real Mode subclasses (ManualDrive, WallFollow,
// Centering, PressTower, LowTower). The supervisor owns a `currentMode`
// pointer and uniformly calls its lifecycle methods. The only states
// that are NOT Modes:
//   - TRANSITION    : brief inter-mode stop with a millis() timer
//   - STRAIGHT_MOVE : raw drivetrain.straightMove ticked from the loop
//                     (could become a Mode later; small enough to leave)
enum CarMode { MANUAL_DRIVE, TRANSITION, WALL_FOLLOWING, CENTERING, PRESSING_NEXUS, PRESSING_TOWER, STRAIGHT_MOVE, LOW_TOWER, ATTACK_NEXUS, ATTACK_TOP_TOWER };
CarMode carMode = MANUAL_DRIVE;

// TRANSITION timing + destination
unsigned long transitionStartMs = 0;
CarMode pendingMode = MANUAL_DRIVE;

// Pointer to currently-active Mode (nullptr for non-Mode states like
// TRANSITION and STRAIGHT_MOVE).
Mode* currentMode = nullptr;

// MODE HELPERS
static void enterMode(CarMode next) {
  if (carMode == next) return;
  // Universal onExit + safety stop. onExit() calls drivetrain.stop() too,
  // but the redundancy is harmless and keeps non-Mode states consistent.
  if (currentMode) {
    currentMode->onExit();
    currentMode = nullptr;
  }
  drivetrain.stop();

  carMode = next;
  switch (carMode) {
    case MANUAL_DRIVE:    currentMode = &manualDrive; break;
    case WALL_FOLLOWING:  currentMode = &wallFollow;  break;
    case CENTERING:       currentMode = &centering;   break;
    case PRESSING_TOWER:  currentMode = &towerPress;  break;
    case PRESSING_NEXUS:  currentMode = &nexusPress;  break;
    case LOW_TOWER:       currentMode = &lowTower;    break;
    case ATTACK_NEXUS:    currentMode = &attackNexus; break;
    case ATTACK_TOP_TOWER:currentMode = &attackTopTower; break;
    case TRANSITION:
      transitionStartMs = millis();
      Serial.println("Mode: TRANSITION");
      break;
    case STRAIGHT_MOVE:
      // Caller (enterStraightMove) is responsible for kicking off the move.
      Serial.println("Mode: STRAIGHT_MOVE");
      break;
  }
  if (currentMode) {
    Serial.printf("Mode: %s\n", currentMode->name());
    currentMode->onEnter();
  }
}

// Helper: brief stop, then hand off to dest mode.
static void enterTransitionTo(CarMode dest) {
  pendingMode = dest;
  enterMode(TRANSITION);
}

// Force-takeover into a non-blocking dead-reckoning straight move.
// Exits any current Mode, stops the drivetrain, then kicks off the move.
// Loop will tick updateStraightMove() each iteration and return to
// MANUAL_DRIVE when the move completes.
static void enterStraightMove(int desiredDist) {
  if (currentMode) {
    currentMode->onExit();
    currentMode = nullptr;
  }
  drivetrain.stop();
  carMode = STRAIGHT_MOVE;
  Serial.printf("Mode: STRAIGHT_MOVE %d in\n", desiredDist);
  drivetrain.straightMove(desiredDist);
}

// Web /mode= callback
void onModeChange(int mode) {
  switch (mode) {
    case 0: enterMode(MANUAL_DRIVE);             break;
    case 1: enterTransitionTo(WALL_FOLLOWING);   break;
    case 2: enterTransitionTo(CENTERING);        break;
    case 3: enterTransitionTo(LOW_TOWER);        break;  // HTML "Low Tower" button
    case 4: enterTransitionTo(ATTACK_NEXUS);     break;  // HTML "Attack Nexus" button
    case 5: enterTransitionTo(ATTACK_TOP_TOWER); break;  // HTML "Top Tower" button
    default: break;
  }
}

// Web /straight= callback — force-takeover into STRAIGHT_MOVE.
void onStraightMove(int inches) {
  enterStraightMove(inches);
}


// SETUP
void setup() {
  Serial.begin(115200);

  robotPos.begin();

  // Hardware
  drivetrain.begin();
  arm.begin();

  Wire1.begin(SDA_pin, SCL_pin, 40000); //tophat pins

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
  web.setStraightMoveCallback(onStraightMove);

  // Boot directly into ManualDrive. carMode is already MANUAL_DRIVE so
  // enterMode would short-circuit; activate the Mode explicitly.
  currentMode = &manualDrive;
  manualDrive.onEnter();

  Serial0.println("Starting in MANUAL_DRIVE mode.");
  Serial0.println("Wall-following engages only via web /mode=1.");
}

// LOOP
void loop() {
  web.serve();           // always serve HTTP
  tofs.update();         // always read distances
  robotPos.callibrate();
  arm.update();
  TopHat();

  // Gate the position telemetry — printf-ing every loop iteration
  // saturates the UART and slows the main loop noticeably.
  static unsigned long lastPosPrintMs = 0;
  unsigned long now = millis();
  if (now - lastPosPrintMs >= 100) {
    lastPosPrintMs = now;
    Serial0.printf("L: %.1f,%.1f  R: %.1f,%.1f  M: %.1f,%.1f\n",
                   robotPos.getRobotPosition(LEFT ).x, robotPos.getRobotPosition(LEFT ).y,
                   robotPos.getRobotPosition(RIGHT).x, robotPos.getRobotPosition(RIGHT).y,
                   robotPos.getRobotPosition(MID  ).x, robotPos.getRobotPosition(MID  ).y);
  }

  // Supervisor: TRANSITION and STRAIGHT_MOVE are the only states that
  // aren't real Modes; everything else is uniformly driven through the
  // currentMode pointer. Modes that auto-complete (PressTower, LowTower,
  // Centering with a front-stop threshold) flag isDone() and the
  // supervisor returns to MANUAL_DRIVE on the next tick.
  switch (carMode) {
    case TRANSITION:
      drivetrain.stop();
      if (millis() - transitionStartMs > 300) enterMode(pendingMode);
      break;

    case STRAIGHT_MOVE:
      if (drivetrain.updateStraightMove(millis())) {
        Serial.println("STRAIGHT_MOVE complete -> MANUAL_DRIVE");
        enterMode(MANUAL_DRIVE);
      }
      break;

    default:
      if (currentMode) {
        currentMode->update();
        if (currentMode->isDone()) {
          Serial.printf("%s complete -> MANUAL_DRIVE\n", currentMode->name());
          enterMode(MANUAL_DRIVE);
        }
      }
      break;
  }
}