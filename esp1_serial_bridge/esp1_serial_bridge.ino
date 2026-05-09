// esp1_serial_bridge.ino
//
// ESP1 — Serial-to-ESP-NOW bridge
//
// Sits between your PC and the robot:
//   PC  -->  USB/Serial  -->  ESP1  -->  ESP-NOW (2.4 GHz)  -->  ESP2 (robot)
//
// SETUP STEPS:
//   1. Flash the "get_mac" snippet (see below) to ESP2 and copy its MAC.
//   2. Paste that MAC into ESP2_MAC[] below.
//   3. Flash this sketch to ESP1.
//
// SERIAL COMMAND FORMAT (send from PC, newline-terminated):
//   RPM:<val>,L:<dir>,R:<dir>   Manual drive  e.g. "RPM:60,L:1,R:1"
//   MODE:<n>                    Change mode   e.g. "MODE:1"
//   STOP                        Emergency stop
//
// Mode numbers match the existing WebController:
//   0 = MANUAL_DRIVE   1 = WALL_FOLLOWING   2 = CENTERING
//   3 = LOW_TOWER      4 = ATTACK_NEXUS     5 = ATTACK_TOP_TOWER
//   10 = VIVE_NAV
//
// Direction values for L/R: 1 = forward, -1 = reverse
//
// ── Get ESP2 MAC (flash this to ESP2 first, read Serial, then remove) ──
// #include <WiFi.h>
// void setup() { Serial.begin(115200); WiFi.mode(WIFI_STA);
//                Serial.println(WiFi.macAddress()); }
// void loop() {}
// ───────────────────────────────────────────────────────────────────────

#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>

// ── Configuration ─────────────────────────────────────────────────────

// Replace with ESP2's actual MAC address
uint8_t ESP2_MAC[] = {0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF};

// Must match the network ESP2 connects to so both land on the same WiFi channel.
// If the router is unreachable, both ESPs fall back to channel 1 automatically.
const char* SSID     = "where the hell";
const char* PASSWORD = "#allnighter";

// ── Shared command struct (must be identical to RobotCommand in ESPNowReceiver.h) ──
struct RobotCommand {
  uint8_t cmdType;   // 0=DRIVE  1=MODE  2=STOP
  float   rpm;       // target RPM            (DRIVE only)
  int8_t  leftDir;   // 1=forward  -1=reverse (DRIVE only)
  int8_t  rightDir;
  int8_t  mode;      // mode index            (MODE only)
};

// ── Globals ───────────────────────────────────────────────────────────
esp_now_peer_info_t peerInfo;

// ── ESP-NOW send callback ─────────────────────────────────────────────
void onSent(const uint8_t* mac, esp_now_send_status_t status) {
  Serial.println(status == ESP_NOW_SEND_SUCCESS
                 ? "[ESP-NOW] Sent OK"
                 : "[ESP-NOW] Send FAILED — check MAC & channel");
}

// ── Initialise ESP-NOW and register ESP2 as a peer ───────────────────
bool initESPNow() {
  if (esp_now_init() != ESP_OK) {
    Serial.println("[ESP-NOW] Init failed");
    return false;
  }
  esp_now_register_send_cb(onSent);

  memset(&peerInfo, 0, sizeof(peerInfo));
  memcpy(peerInfo.peer_addr, ESP2_MAC, 6);
  peerInfo.channel = 0;      // 0 = inherit current WiFi channel
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("[ESP-NOW] Failed to add peer — check MAC address");
    return false;
  }

  Serial.println("[ESP-NOW] Peer registered, ready to send");
  return true;
}

// ── Send a filled-in RobotCommand struct ─────────────────────────────
void sendCommand(RobotCommand& cmd) {
  esp_err_t result = esp_now_send(ESP2_MAC, (uint8_t*)&cmd, sizeof(cmd));
  if (result != ESP_OK) {
    Serial.printf("[ESP-NOW] esp_now_send error: 0x%X\n", result);
  }
}

// ── Parse one line of serial input and dispatch ───────────────────────
void processLine(String& line) {
  line.trim();
  if (line.length() == 0) return;

  RobotCommand cmd;
  memset(&cmd, 0, sizeof(cmd));

  // ── STOP ────────────────────────────────────────────────────────────
  if (line.equalsIgnoreCase("STOP")) {
    cmd.cmdType = 2;
    sendCommand(cmd);
    Serial.println("[TX] STOP");

  // ── MODE:<n> ────────────────────────────────────────────────────────
  } else if (line.startsWith("MODE:")) {
    cmd.cmdType = 1;
    cmd.mode    = (int8_t)line.substring(5).toInt();
    sendCommand(cmd);
    Serial.printf("[TX] MODE %d\n", cmd.mode);

  // ── RPM:<val>,L:<dir>,R:<dir> ────────────────────────────────────────
  } else if (line.startsWith("RPM:")) {
    // Expected format: RPM:<val>,L:<1|-1>,R:<1|-1>
    // Example: RPM:60,L:1,R:1  (forward at 60 RPM)
    //          RPM:60,L:-1,R:1 (spin left)
    int commaAfterRpm = line.indexOf(',');
    int lIdx          = line.indexOf("L:");
    int commaAfterL   = line.indexOf(',', lIdx);
    int rIdx          = line.indexOf("R:");

    if (commaAfterRpm < 0 || lIdx < 0 || rIdx < 0) {
      Serial.println("[ERR] Bad format. Expected: RPM:<val>,L:<1|-1>,R:<1|-1>");
      return;
    }

    cmd.cmdType  = 0;
    cmd.rpm      = line.substring(4, commaAfterRpm).toFloat();
    cmd.leftDir  = (int8_t)line.substring(lIdx + 2, commaAfterL).toInt();
    cmd.rightDir = (int8_t)line.substring(rIdx + 2).toInt();

    sendCommand(cmd);
    Serial.printf("[TX] DRIVE  rpm=%.1f  L=%+d  R=%+d\n",
                  cmd.rpm, cmd.leftDir, cmd.rightDir);

  } else {
    Serial.println("[ERR] Unknown command.");
    Serial.println("      Valid: STOP | MODE:<n> | RPM:<val>,L:<1|-1>,R:<1|-1>");
  }
}

// ── Setup ─────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n============================");
  Serial.println("  ESP1 Serial Bridge v1.0");
  Serial.println("============================");

  // WiFi station mode — required before ESP-NOW init.
  // Connecting to the same network as ESP2 guarantees matching channels.
  WiFi.mode(WIFI_STA);
  WiFi.begin(SSID, PASSWORD);

  Serial.printf("[WiFi] Connecting to \"%s\"", SSID);
  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < 8000) {
    delay(500);
    Serial.print('.');
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.printf("\n[WiFi] Connected — channel %d\n", WiFi.channel());
  } else {
    Serial.println("\n[WiFi] Router not found — using fixed channel 1.");
    Serial.println("       ESP2 will also fall back to channel 1 automatically.");
    esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);
  }

  if (!initESPNow()) {
    Serial.println("[FATAL] ESP-NOW init failed. Halting.");
    while (true) delay(1000);
  }

  Serial.println("\nReady. Commands:");
  Serial.println("  STOP");
  Serial.println("  MODE:<0-10>");
  Serial.println("  RPM:<val>,L:<1|-1>,R:<1|-1>");
  Serial.println("  Example: RPM:60,L:1,R:1   (forward at 60 RPM)");
}

// ── Loop ──────────────────────────────────────────────────────────────
void loop() {
  if (Serial.available()) {
    String line = Serial.readStringUntil('\n');
    processLine(line);
  }
}
