#include "WebController.h"
#include "PIDandTOF_web.h"   // provides `body` HTML

WebController* WebController::s_self = nullptr;

WebController::WebController(Drivetrain& dt, WallFollow& wf, ModeCallback onMode)
  : _dt(dt), _wf(wf), _onMode(onMode), _h(80)
{
  s_self = this;
}

void WebController::begin(const char* ssid, const char* password) {
  // NOTE: original sketch declared static IP variables but never called
  // WiFi.config(), so we match: DHCP. To enable static IP, uncomment:
  //   WiFi.config(ip, gateway, subnet);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) { delay(500); Serial.print("."); }
  Serial.println("\nWiFi connected");
  Serial.print("IP: "); Serial.println(WiFi.localIP());

  // Push initial gains into the drivetrain so first PID tick is sane.
  _dt.setPIDGains(_kp, _ki, _kd);

  _h.begin();
  _h.attachHandler("/motor_speed=", hSpeed);
  _h.attachHandler("/Kp=",          hKp);
  _h.attachHandler("/Ki=",          hKi);
  _h.attachHandler("/Kd=",          hKd);
  _h.attachHandler("/dir=",         hDir);
  _h.attachHandler("/mode=",        hMode);
  _h.attachHandler("/wf_Kp=",       hWfKp);
  _h.attachHandler("/wf_Kd=",       hWfKd);
  _h.attachHandler("/sharpTurn=",   hSharpTurn);
  _h.attachHandler("/",             hRoot);
}

void WebController::serve() { _h.serve(); }

// =====================================================================
// Static trampolines — each fetches the live instance and forwards.
// =====================================================================
void WebController::hRoot() {
  if (!s_self) return;
  s_self->_h.sendhtml(body);
}

void WebController::hDir() {
  if (!s_self) return;
  String d = s_self->_h.getText();
  int L = 0, R = 0;
  if      (d == "F") { L =  1; R =  1; }
  else if (d == "B") { L = -1; R = -1; }
  else if (d == "L") { L = -1; R =  1; }
  else if (d == "R") { L =  1; R = -1; }
  s_self->_dt.setDirection(L, R);
  s_self->_h.sendhtml(body);
}

void WebController::hSpeed() {
  if (!s_self) return;
  float rpm = s_self->_h.getVal();
  s_self->_dt.setTargetRPM(rpm);
  Serial.printf("Manual warmup. Target RPM: %.1f\n", rpm);
  s_self->_h.sendhtml(body);
}

void WebController::hKp() {
  if (!s_self) return;
  s_self->_kp = s_self->_h.getVal();
  s_self->_dt.setPIDGains(s_self->_kp, s_self->_ki, s_self->_kd);
  Serial.printf("Kp: %.2f\n", s_self->_kp);
  s_self->_h.sendhtml(body);
}

void WebController::hKi() {
  if (!s_self) return;
  s_self->_ki = s_self->_h.getVal();
  s_self->_dt.setPIDGains(s_self->_kp, s_self->_ki, s_self->_kd);
  Serial.printf("Ki: %.2f\n", s_self->_ki);
  s_self->_h.sendhtml(body);
}

void WebController::hKd() {
  if (!s_self) return;
  s_self->_kd = s_self->_h.getVal();
  s_self->_dt.setPIDGains(s_self->_kp, s_self->_ki, s_self->_kd);
  Serial.printf("Kd: %.2f\n", s_self->_kd);
  s_self->_h.sendhtml(body);
}

void WebController::hMode() {
  if (!s_self) return;
  int mode = s_self->_h.getVal();
  if (s_self->_onMode) s_self->_onMode(mode);
  s_self->_h.sendhtml(body);
}

void WebController::hWfKp() {
  if (!s_self) return;
  float v = s_self->_h.getVal();
  s_self->_wf.setKp(v);
  Serial.printf("wf_Kp: %.2f\n", v);
  s_self->_h.sendhtml(body);
}

void WebController::hWfKd() {
  if (!s_self) return;
  float v = s_self->_h.getVal();
  s_self->_wf.setKd(v);
  Serial.printf("wf_Kd: %.2f\n", v);
  s_self->_h.sendhtml(body);
}

void WebController::hSharpTurn() {
  if (!s_self) return;
  int v = (int)s_self->_h.getVal();
  s_self->_wf.setSharpTurnOffset(v);
  Serial.printf("sharpTurnOffset: %d\n", v);
  s_self->_h.sendhtml(body);
}
