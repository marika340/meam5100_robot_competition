#include "EspNowController.h"

#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include "Attacker.h"

// Mutex guarding ring-buffer head/tail. Producer = WiFi task (recv cb),
// consumer = main loop (poll). Same pattern as the lecture's debounce
// example (interrupt-debounc-lecture.ino).
static portMUX_TYPE s_qMux = portMUX_INITIALIZER_UNLOCKED;

EspNowController* EspNowController::s_self = nullptr;

EspNowController::EspNowController(ManualDrive& md, WallFollow& wf, ModeCallback onMode)
  : _md(md), _wf(wf), _onMode(onMode)
{
  s_self = this;
}

bool EspNowController::begin() {
  // WebController has already brought up softAP; the radio is parked on
  // its channel and the AP MAC is live. Print everything the bridge
  // needs in one place so it's easy to copy into bridge_espnow.ino.
  Serial.println();
  Serial.println("[ESPNOW] ============================================");
  Serial.print  ("[ESPNOW] Robot AP  MAC (paste into bridge ROBOT_MAC): ");
  Serial.println(WiFi.softAPmacAddress());
  Serial.print  ("[ESPNOW] Robot STA MAC (informational only):           ");
  Serial.println(WiFi.macAddress());
  Serial.print  ("[ESPNOW] Channel  (paste into bridge ROBOT_CHANNEL):   ");
  Serial.println(WiFi.channel());
  Serial.println("[ESPNOW] ============================================");

  if (esp_now_init() != ESP_OK) {
    Serial.println("[ESPNOW] esp_now_init FAILED");
    return false;
  }

  if (esp_now_register_recv_cb(EspNowController::onRecvTrampoline) != ESP_OK) {
    Serial.println("[ESPNOW] register_recv_cb FAILED");
    return false;
  }

  Serial.println("[ESPNOW] Initialized — waiting for bridge packets");
  return true;
}

// ----- ISR-context recv callback ------------------------------------
// Runs in the WiFi task. Keep it short (lecture's IRAM_ATTR / portMUX
// guidance applies). Copy the bytes into the ring buffer and bail.
// Signature per lecture slide 30 (Arduino Core 3.x / IDF 5.x).
void EspNowController::onRecvTrampoline(const esp_now_recv_info_t* /*info*/,
                                        const uint8_t* data, int data_len) {
  if (!s_self) return;
  s_self->enqueueFromCb(data, data_len);
}

void EspNowController::enqueueFromCb(const uint8_t* data, int len) {
  if (len != (int)sizeof(RobotCommand)) {
    // Silently drop malformed packets here — too noisy to Serial.print
    // from inside the WiFi task. poll() reports drop counts periodically.
    portENTER_CRITICAL_ISR(&s_qMux);
    _drops++;
    portEXIT_CRITICAL_ISR(&s_qMux);
    return;
  }

  portENTER_CRITICAL_ISR(&s_qMux);
  size_t next = (_head + 1) % QSZ;
  if (next == _tail) {
    _drops++;                       // queue full
  } else {
    memcpy(&_q[_head], data, sizeof(RobotCommand));
    _head = next;
    _rxCount++;
  }
  portEXIT_CRITICAL_ISR(&s_qMux);
}

// ----- Main-loop drain ----------------------------------------------
void EspNowController::poll() {
  // Periodic drop-counter report so silent failures show up in serial.
  static uint32_t lastReportedDrops = 0;
  static uint32_t lastReportMs = 0;
  uint32_t now = millis();
  if (now - lastReportMs > 5000) {
    lastReportMs = now;
    uint32_t d;
    portENTER_CRITICAL(&s_qMux);
    d = _drops;
    portEXIT_CRITICAL(&s_qMux);
    if (d != lastReportedDrops) {
      Serial.printf("[ESPNOW] drops=%lu (cumulative)\n", (unsigned long)d);
      lastReportedDrops = d;
    }
  }

  // Drain everything queued since last poll.
  while (true) {
    RobotCommand pkt;
    bool got = false;
    portENTER_CRITICAL(&s_qMux);
    if (_tail != _head) {
      memcpy(&pkt, &_q[_tail], sizeof(RobotCommand));
      _tail = (_tail + 1) % QSZ;
      got = true;
    }
    portEXIT_CRITICAL(&s_qMux);
    if (!got) break;

    dispatch(pkt);
  }
}

// ----- Command dispatch ---------------------------------------------
// Mirrors WebController's HTTP handlers one-for-one. Logs are tagged
// [ESPNOW seq=N] so they're easy to distinguish from web logs.
void EspNowController::dispatch(const RobotCommand& pkt) {
  const unsigned long seq = (unsigned long)pkt.seq;

  switch (pkt.cmd) {
    case CMD_DIR: {
      char d = (char)(pkt.p1 & 0xFF);
      int L = 0, R = 0;
      if      (d == 'F') { L =  1; R =  1; }
      else if (d == 'B') { L = -1; R = -1; }
      else if (d == 'L') { L = -1; R =  1; }
      else if (d == 'R') { L =  1; R = -1; }
      else if (d == 'S') { L =  0; R =  0; }
      _md.setDirection(L, R);
      Serial.printf("[ESPNOW seq=%lu] DIR=%c (L=%d R=%d)\n", seq, d, L, R);
      break;
    }

    case CMD_SPEED:
      _md.setTargetRPM((float)pkt.p1);
      Serial.printf("[ESPNOW seq=%lu] RPM=%d\n", seq, (int)pkt.p1);
      break;

    case CMD_MODE:
      Serial.printf("[ESPNOW seq=%lu] MODE=%d\n", seq, (int)pkt.p1);
      if (_onMode) _onMode((int)pkt.p1);
      break;

    case CMD_GAIN: {
      float v = pkt.p2 / 100.0f;
      switch (pkt.p1) {
        case GAIN_KP:
          _md.setKp(v);
          Serial.printf("[ESPNOW seq=%lu] Kp=%.2f\n", seq, v); break;
        case GAIN_KI:
          _md.setKi(v);
          Serial.printf("[ESPNOW seq=%lu] Ki=%.2f\n", seq, v); break;
        case GAIN_KD:
          _md.setKd(v);
          Serial.printf("[ESPNOW seq=%lu] Kd=%.2f\n", seq, v); break;
        case GAIN_WF_KP:
          _wf.setKp(v);
          Serial.printf("[ESPNOW seq=%lu] wf_Kp=%.2f\n", seq, v); break;
        case GAIN_WF_KD:
          _wf.setKd(v);
          Serial.printf("[ESPNOW seq=%lu] wf_Kd=%.2f\n", seq, v); break;
        case GAIN_SHARPTURN:
          // sharpTurn is an int — bridge sends raw (so p2 is the int directly).
          _wf.setSharpTurnOffset((int)pkt.p2);
          Serial.printf("[ESPNOW seq=%lu] sharpTurn=%d\n", seq, (int)pkt.p2); break;
        default:
          Serial.printf("[ESPNOW seq=%lu] unknown GAIN id=%d\n", seq, (int)pkt.p1); break;
      }
      break;
    }

    case CMD_STRAIGHT:
      Serial.printf("[ESPNOW seq=%lu] STRAIGHT=%d in\n", seq, (int)pkt.p1);
      if (_onStraight) _onStraight((int)pkt.p1);
      break;

    case CMD_ATTACK: {
      extern Attacker arm;     // declared in tof_modular.ino (matches WebController::hAttack)
      arm.toggle();
      Serial.printf("[ESPNOW seq=%lu] ATTACK toggle\n", seq);
      break;
    }

    case CMD_PING:
      Serial.printf("[ESPNOW seq=%lu] PING\n", seq);
      break;

    default:
      Serial.printf("[ESPNOW seq=%lu] unknown cmd 0x%02X\n", seq, pkt.cmd);
      break;
  }
}
