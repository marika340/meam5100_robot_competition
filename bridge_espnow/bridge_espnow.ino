// =====================================================================
// bridge_espnow — laptop-tethered ESP that converts ASCII serial
// commands from the host PC into typed RobotCommand packets and ships
// them to the robot over ESP-NOW. Designed to talk to the robot side
// implemented in tof_modular/EspNowController.{h,cpp}.
//
// Hardware: ESP32-C3 (any ESP32-family also works).
//
// Channel pinning:
//   The robot is in WiFi.softAP() mode on a fixed channel (set by
//   WebController.cpp -> WiFi.softAP(ssid, password, 5)). ESP-NOW
//   requires both ends to be on the same channel, so the bridge does
//   NOT join any AP — it just forces its own radio onto the robot's
//   channel via esp_wifi_set_channel().
//
// FILL IN BEFORE FLASHING (see comments below):
//   1) ROBOT_AP_MAC[6]      — robot's AP MAC, printed by EspNowController::begin()
//   2) ROBOT_CHANNEL        — must match WiFi.softAP(..., CHANNEL) in WebController.cpp
// =====================================================================

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

#include "RobotCommand.h"

// ---------- USER CONFIG ----------------------------------------------
// Paste the AP MAC printed by the robot at boot.
// Look for: [ESPNOW] Robot AP  MAC (paste into bridge ROBOT_MAC): XX:XX:XX:XX:XX:XX
static uint8_t ROBOT_AP_MAC[6] = {
  0x10, 0x51, 0xDB, 0xD5, 0x4B, 0xA5   // <<< 10:51:DB:D5:4B:A5
};

// Must match WiFi.softAP(..., CHANNEL) in tof_modular/WebController.cpp
// (currently 5). The robot also prints its channel at boot:
//   [ESPNOW] Channel  (paste into bridge ROBOT_CHANNEL): 5
static const uint8_t ROBOT_CHANNEL = 5;
// ---------------------------------------------------------------------

static esp_now_peer_info_t peer = {};   // populated in setup()
static uint32_t g_seq = 0;

// ----- Helpers -------------------------------------------------------

static void printMac(const uint8_t mac[6]) {
  for (int i = 0; i < 6; i++) {
    if (i) Serial.print(':');
    if (mac[i] < 0x10) Serial.print('0');
    Serial.print(mac[i], HEX);
  }
}

// Send-status callback — prints OK/FAIL with the seq number we just
// shipped, so a missing OK shows up as a clear gap in the log.
// Signature: (peer MAC, status) — same across all IDF/Arduino Core versions.
static void onSendCb(const uint8_t* /*mac_addr*/, esp_now_send_status_t status) {
  Serial.printf("[BRIDGE seq=%lu] %s\n",
                (unsigned long)g_seq,
                (status == ESP_NOW_SEND_SUCCESS) ? "OK" : "FAIL");
}

// Build + send a packet. Returns the seq number we used.
static uint32_t sendCmd(uint8_t cmd, int16_t p1, int16_t p2 = 0) {
  RobotCommand pkt;
  pkt.cmd = cmd;
  pkt.p1  = p1;
  pkt.p2  = p2;
  pkt.seq = ++g_seq;
  esp_err_t r = esp_now_send(ROBOT_AP_MAC, (const uint8_t*)&pkt, sizeof(pkt));
  if (r != ESP_OK) {
    Serial.printf("[BRIDGE seq=%lu] esp_now_send err=%d\n",
                  (unsigned long)pkt.seq, (int)r);
  }
  return pkt.seq;
}

// ----- Tiny line parser ----------------------------------------------
// Reads CR/LF-terminated lines from Serial, lower-cases the verb, and
// dispatches. Anything we don't recognize prints a help message.
static void handleLine(String line) {
  line.trim();
  if (line.length() == 0) return;

  // Echo so it's clear what the bridge attempted.
  Serial.print("> "); Serial.println(line);

  // Single-letter direction shortcuts: F/B/L/R/S
  if (line.length() == 1) {
    char c = toupper(line[0]);
    if (c == 'F' || c == 'B' || c == 'L' || c == 'R' || c == 'S') {
      sendCmd(CMD_DIR, (int16_t)c);
      return;
    }
  }

  // Tokenize: verb + optional numeric arg(s)
  int sp = line.indexOf(' ');
  String verb = (sp < 0) ? line : line.substring(0, sp);
  String arg  = (sp < 0) ? ""   : line.substring(sp + 1);
  verb.toLowerCase();
  arg.trim();

  if (verb == "rpm") {
    sendCmd(CMD_SPEED, (int16_t)arg.toInt());
  }
  else if (verb == "mode") {
    sendCmd(CMD_MODE, (int16_t)arg.toInt());
  }
  else if (verb == "kp") {
    sendCmd(CMD_GAIN, GAIN_KP, (int16_t)(arg.toFloat() * 100.0f));
  }
  else if (verb == "ki") {
    sendCmd(CMD_GAIN, GAIN_KI, (int16_t)(arg.toFloat() * 100.0f));
  }
  else if (verb == "kd") {
    sendCmd(CMD_GAIN, GAIN_KD, (int16_t)(arg.toFloat() * 100.0f));
  }
  else if (verb == "wfkp") {
    sendCmd(CMD_GAIN, GAIN_WF_KP, (int16_t)(arg.toFloat() * 100.0f));
  }
  else if (verb == "wfkd") {
    sendCmd(CMD_GAIN, GAIN_WF_KD, (int16_t)(arg.toFloat() * 100.0f));
  }
  else if (verb == "sharp") {
    sendCmd(CMD_GAIN, GAIN_SHARPTURN, (int16_t)arg.toInt());
  }
  else if (verb == "straight") {
    sendCmd(CMD_STRAIGHT, (int16_t)arg.toInt());
  }
  else if (verb == "atk" || verb == "attack") {
    sendCmd(CMD_ATTACK, 0);
  }
  else if (verb == "ping") {
    sendCmd(CMD_PING, 0);
  }
  else if (verb == "help" || verb == "?") {
    Serial.println(F(
      "Commands:\n"
      "  F | B | L | R | S      direction (forward/back/left/right/stop)\n"
      "  rpm <int>              target RPM (ManualDrive)\n"
      "  mode <0..5,10>         0=manual 1=wallFollow 2=center 3=lowTower\n"
      "                         4=attackNexus 5=topTower 10=viveNav\n"
      "  kp <f>  ki <f>  kd <f> ManualDrive PID gains\n"
      "  wfkp <f>  wfkd <f>     WallFollow gains\n"
      "  sharp <int>            WallFollow sharpTurn offset\n"
      "  straight <inches>      dead-reckoning straight move\n"
      "  atk                    toggle Attacker arm\n"
      "  ping                   liveness check\n"
      "  help                   this help\n"
    ));
  }
  else {
    Serial.printf("[BRIDGE] unknown command '%s' — type 'help'\n", verb.c_str());
  }
}

// ----- Setup / loop --------------------------------------------------

void setup() {
  Serial.begin(115200);
  delay(200);                       // give USB CDC a moment

  Serial.println();
  Serial.println("=== ESP-NOW Bridge starting ===");

  // STA mode is enough — we never join an AP, we just need the radio
  // up so we can pin the channel and run ESP-NOW.
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();                // make sure we're not auto-joining anything

  Serial.print("Bridge STA MAC (informational): ");
  Serial.println(WiFi.macAddress());

  // Force the bridge onto the robot's softAP channel. ESP-NOW only
  // delivers when sender + receiver are on the same channel.
  esp_err_t cr = esp_wifi_set_channel(ROBOT_CHANNEL, WIFI_SECOND_CHAN_NONE);
  if (cr != ESP_OK) {
    Serial.printf("esp_wifi_set_channel(%u) err=%d\n", ROBOT_CHANNEL, (int)cr);
  } else {
    Serial.printf("Bridge channel pinned to %u\n", ROBOT_CHANNEL);
  }

  if (esp_now_init() != ESP_OK) {
    Serial.println("esp_now_init FAILED — restarting");
    delay(1000);
    ESP.restart();
  }

  esp_now_register_send_cb(onSendCb);

  // Configure the robot as our peer.
  memcpy(peer.peer_addr, ROBOT_AP_MAC, 6);
  peer.channel = ROBOT_CHANNEL;     // 0 == "current channel" also works after esp_wifi_set_channel
  peer.encrypt = false;
  peer.ifidx   = WIFI_IF_STA;       // bridge is in STA mode

  if (esp_now_add_peer(&peer) != ESP_OK) {
    Serial.println("esp_now_add_peer FAILED");
  } else {
    Serial.print("Peer added: ");
    printMac(ROBOT_AP_MAC);
    Serial.println();
  }

  // Sanity check: warn if the user forgot to paste the MAC.
  bool zeroMac = true;
  for (int i = 0; i < 6; i++) if (ROBOT_AP_MAC[i] != 0) { zeroMac = false; break; }
  if (zeroMac) {
    Serial.println("\n***********************************************************");
    Serial.println("* ROBOT_AP_MAC is all zeros — paste the robot's AP MAC into");
    Serial.println("* bridge_espnow.ino and reflash before sending commands.");
    Serial.println("***********************************************************\n");
  }

  Serial.println("\nReady. Type 'help' for commands.");
}

void loop() {
  // Read one line at a time from the serial monitor.
  static String buf;
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      handleLine(buf);
      buf = "";
    } else {
      buf += c;
      if (buf.length() > 120) buf = "";   // sanity cap
    }
  }
}
