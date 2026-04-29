#ifndef WEB_CONTROLLER_H
#define WEB_CONTROLLER_H

#include <Arduino.h>
#include <WiFi.h>
#include "html510.h"
#include "Drivetrain.h"
#include "WallFollow.h"

// =====================================================================
// WebController: WiFi bring-up + HTTP UI for the car.
//
// The HTML510Server takes plain `void(*)()` handlers with no context,
// so this class uses a static singleton pointer (`s_self`) and static
// trampoline methods that route into a single live instance. Only
// instantiate one WebController.
//
// Wires the web UI to:
//   - Drivetrain (manual drive, PID gains, auto enable)
//   - WallFollow (wf gains, sharp-turn offset)
//   - main.ino  (mode change callback for /mode=)
//
// Caller responsibilities:
//   - construct ONE WebController, passing all refs and mode callback
//   - call begin(ssid, password, ip, gw, sn) once in setup()
//   - call serve() every loop iteration
// =====================================================================
class WebController {
public:
  using ModeCallback = void (*)(int newMode);

  WebController(Drivetrain& dt, WallFollow& wf, ModeCallback onMode);

  // Connect WiFi (blocking) and register all handlers.
  void begin(const char* ssid, const char* password);

  void serve();

private:
  Drivetrain&  _dt;
  WallFollow&  _wf;
  ModeCallback _onMode;
  HTML510Server _h;

  // Singleton hookup so static handlers can find the live instance.
  static WebController* s_self;

  // Handler trampolines (registered with HTML510Server)
  static void hRoot();
  static void hDir();
  static void hSpeed();
  static void hKp();
  static void hKi();
  static void hKd();
  static void hMode();
  static void hWfKp();
  static void hWfKd();
  static void hSharpTurn();

  // Mutable PID gains the web UI updates (so /Kp= without /Ki= or /Kd=
  // still preserves the others). Mirrored into the Drivetrain on change.
  float _kp = 1.4f;
  float _ki = 1.0f;
  float _kd = 0.0f;
};

#endif // WEB_CONTROLLER_H
