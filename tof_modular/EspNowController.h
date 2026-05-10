#ifndef ESP_NOW_CONTROLLER_H
#define ESP_NOW_CONTROLLER_H

#include <Arduino.h>
#include <esp_now.h>
#include "ManualDrive.h"
#include "WallFollow.h"
#include "RobotCommand.h"

// =====================================================================
// EspNowController — sibling of WebController, but for ESP-NOW packets
// from a USB-tethered "bridge" ESP. Routes incoming RobotCommand packets
// into the same ManualDrive / WallFollow / onModeChange entry points
// the web UI uses, so the HTML interface and ESP-NOW input are
// interchangeable from the robot's perspective.
//
// Coexists with WebController on the same radio: the robot is in softAP
// mode on a fixed channel, and ESP-NOW rides that same channel — no
// router, no association, no interference between the two paths.
//
// Caller responsibilities:
//   - construct ONE EspNowController, passing the same refs and mode
//     callback used by WebController
//   - call begin() AFTER WebController::begin() (so the AP/channel is
//     already up; begin() reads WiFi.channel() to log it for the bridge)
//   - call poll() every loop iteration to drain queued packets
//
// The receive callback runs in the WiFi task: it ONLY copies the packet
// into a small ring buffer guarded by portMUX. All actual handling
// (calling ManualDrive / WallFollow / mode change) happens in poll() in
// the main loop context.
// =====================================================================
class EspNowController {
public:
  using ModeCallback         = void (*)(int newMode);
  using StraightMoveCallback = void (*)(int inches);

  EspNowController(ManualDrive& md, WallFollow& wf, ModeCallback onMode);

  // Initializes ESP-NOW and registers the receive callback. Prints the
  // robot's AP MAC + channel — copy these into the bridge sketch.
  // Returns false on init failure.
  bool begin();

  // Optional /straight= analogue.
  void setStraightMoveCallback(StraightMoveCallback cb) { _onStraight = cb; }

  // Drain queued packets and dispatch. Call every loop().
  void poll();

private:
  static constexpr size_t QSZ = 16;   // ring buffer depth (power of 2 not required)

  ManualDrive& _md;
  WallFollow&  _wf;
  ModeCallback _onMode;
  StraightMoveCallback _onStraight = nullptr;

  // SP-SC ring buffer: producer is the WiFi-task recv callback,
  // consumer is the main loop. portMUX guards head/tail updates.
  RobotCommand _q[QSZ];
  volatile size_t   _head = 0;        // next slot to write (cb side)
  volatile size_t   _tail = 0;        // next slot to read  (poll side)
  volatile uint32_t _drops = 0;       // dropped packets when queue is full
  volatile uint32_t _rxCount = 0;     // total packets received

  // Singleton hookup so the static recv trampoline finds the live instance
  static EspNowController* s_self;

  // Static trampoline registered with esp_now_register_recv_cb.
  // Signature matches Arduino Core 3.x / IDF 5.x (lecture slide 30).
  static void onRecvTrampoline(const esp_now_recv_info_t* info,
                               const uint8_t* data, int data_len);

  void enqueueFromCb(const uint8_t* data, int len);
  void dispatch(const RobotCommand& pkt);
};

#endif // ESP_NOW_CONTROLLER_H
