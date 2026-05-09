#include "ESPNowReceiver.h"

// ── Static member definitions ─────────────────────────────────────────
volatile bool         ESPNowReceiver::_hasCommand = false;
volatile RobotCommand ESPNowReceiver::_pendingCmd  = {};

// ── begin() ───────────────────────────────────────────────────────────
bool ESPNowReceiver::begin() {
  // ESP-NOW requires WiFi to already be initialised. If the router
  // wasn't reachable during web.begin(), fall back to a fixed channel —
  // make sure ESP1 uses the same fallback (channel 1).
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("[ESP-NOW] WiFi not connected — fixing channel to 1.");
    Serial.println("[ESP-NOW] Ensure ESP1 also uses channel 1!");
    esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);
  } else {
    Serial.printf("[ESP-NOW] WiFi up on channel %d\n", WiFi.channel());
  }

  if (esp_now_init() != ESP_OK) {
    Serial.println("[ESP-NOW] Init failed — ESP-NOW receive disabled.");
    return false;
  }

  esp_now_register_recv_cb(onReceive);
  Serial.println("[ESP-NOW] Receiver ready.");
  return true;
}

// ── onReceive() — ISR context, keep minimal ───────────────────────────
void ESPNowReceiver::onReceive(const esp_now_recv_info_t* info,
                               const uint8_t* data, int len) {
  if (len != sizeof(RobotCommand)) return;  // wrong packet size — ignore
  memcpy((void*)&_pendingCmd, data, sizeof(RobotCommand));
  _hasCommand = true;
}

// ── update() — called every loop() ───────────────────────────────────
void ESPNowReceiver::update() {
  if (!_hasCommand) return;
  _hasCommand = false;

  // Safe copy out of volatile storage before touching any callbacks
  RobotCommand cmd;
  memcpy(&cmd, (const void*)&_pendingCmd, sizeof(cmd));

  switch (cmd.cmdType) {
    case 0:  // DRIVE
      if (_onDrive) _onDrive(cmd.rpm, (int)cmd.leftDir, (int)cmd.rightDir);
      break;

    case 1:  // MODE
      if (_onMode) _onMode((int)cmd.mode);
      break;

    case 2:  // STOP
      if (_onStop) _onStop();
      break;

    default:
      Serial.printf("[ESP-NOW] Unknown cmdType: %d\n", cmd.cmdType);
      break;
  }
}
