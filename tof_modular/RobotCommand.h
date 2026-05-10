#ifndef ROBOT_COMMAND_H
#define ROBOT_COMMAND_H

#include <stdint.h>

// =====================================================================
// RobotCommand — typed binary packet sent over ESP-NOW from the bridge
// ESP (laptop tether) to the robot ESP. Mirrors the command surface of
// the existing HTTP UI in PIDandTOF_web.h, so EspNowController can route
// straight into ManualDrive / WallFollow / onModeChange the same way
// WebController does.
//
// Keep this header byte-for-byte identical in:
//   tof_modular/RobotCommand.h
//   bridge_espnow/RobotCommand.h
// =====================================================================

enum CmdType : uint8_t {
  CMD_NOOP     = 0x00,
  CMD_DIR      = 0x01,  // p1 = 'F'/'B'/'L'/'R'/'S' (ASCII char in low byte)
  CMD_SPEED    = 0x02,  // p1 = target RPM (signed int)
  CMD_MODE     = 0x03,  // p1 = onModeChange code (0..5, 10)
  CMD_GAIN     = 0x04,  // p1 = which knob (see GainId below)
                        // p2 = value * 100 (so 1.50 -> 150; sharpTurn raw)
  CMD_STRAIGHT = 0x05,  // p1 = inches
  CMD_ATTACK   = 0x06,  // p1 ignored — toggles Attacker arm
  CMD_PING     = 0xFF,
};

enum GainId : uint8_t {
  GAIN_KP        = 0,   // ManualDrive Kp
  GAIN_KI        = 1,   // ManualDrive Ki
  GAIN_KD        = 2,   // ManualDrive Kd
  GAIN_WF_KP     = 3,   // WallFollow Kp
  GAIN_WF_KD     = 4,   // WallFollow Kd
  GAIN_SHARPTURN = 5,   // WallFollow sharpTurn offset (raw int, p2 not /100)
};

typedef struct __attribute__((packed)) {
  uint8_t  cmd;
  int16_t  p1;
  int16_t  p2;
  uint32_t seq;
} RobotCommand;   // 9 bytes — fits trivially in V1 (250B) or V2 (1470B)

#endif // ROBOT_COMMAND_H
