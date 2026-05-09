#ifndef ESPNOW_RECEIVER_H
#define ESPNOW_RECEIVER_H

// ESPNowReceiver.h
//
// Receives ESP-NOW packets from ESP1 and dispatches them to the
// existing mode/drive system in tof_modular via simple callbacks.
//
// Usage (tof_modular.ino):
//   1. Construct one ESPNowReceiver instance (global scope).
//   2. Register callbacks with setDriveCallback / setModeCallback / setStopCallback.
//   3. Call begin() once in setup(), AFTER web.begin() (needs WiFi up first).
//   4. Call update() every loop() iteration — dispatches any pending command.
//
// Thread safety:
//   The ESP-NOW receive callback fires at interrupt level. A volatile
//   flag + volatile buffer protect the ISR<->loop handoff; update() does
//   a safe memcpy before invoking any user callback.

#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>

// ── Shared command struct ─────────────────────────────────────────────
// !! Must be byte-for-byte identical to the struct in esp1_serial_bridge.ino !!
struct RobotCommand {
  uint8_t cmdType;   // 0=DRIVE  1=MODE  2=STOP
  float   rpm;       // target RPM            (DRIVE)
  int8_t  leftDir;   // 1=forward  -1=reverse (DRIVE)
  int8_t  rightDir;
  int8_t  mode;      // mode index            (MODE)
};

// ── Callback typedefs ─────────────────────────────────────────────────
using EspNowDriveCallback = void (*)(float rpm, int leftDir, int rightDir);
using EspNowModeCallback  = void (*)(int mode);
using EspNowStopCallback  = void (*)();

// ── ESPNowReceiver ────────────────────────────────────────────────────
class ESPNowReceiver {
public:
  ESPNowReceiver() = default;

  // Register callbacks before calling begin().
  void setDriveCallback(EspNowDriveCallback cb) { _onDrive = cb; }
  void setModeCallback (EspNowModeCallback  cb) { _onMode  = cb; }
  void setStopCallback (EspNowStopCallback  cb) { _onStop  = cb; }

  // Initialise ESP-NOW. Call after WiFi is connected (i.e. after web.begin()).
  // Returns true on success.
  bool begin();

  // Call every loop() iteration to dispatch any pending command.
  void update();

private:
  EspNowDriveCallback _onDrive = nullptr;
  EspNowModeCallback  _onMode  = nullptr;
  EspNowStopCallback  _onStop  = nullptr;

  // ISR-safe receive buffer.
  // _hasCommand is set true in the ISR and cleared in update().
  static volatile bool         _hasCommand;
  static volatile RobotCommand _pendingCmd;

  // ESP-NOW receive callback (runs at interrupt level — keep minimal).
  static void onReceive(const esp_now_recv_info_t* info,
                        const uint8_t* data, int len);
};

#endif // ESPNOW_RECEIVER_H
