#include "WebController.h"
#include "PIDandTOF_web.h"   // provides `body` HTML
#include "TopHat.h"
#include "Attacker.h"
#include "ViveNavigation.h"

WebController* WebController::s_self = nullptr;

WebController::WebController(ManualDrive& md, WallFollow& wf, ModeCallback onMode)
  : _md(md), _wf(wf), _onMode(onMode), _h(80)
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
  _h.attachHandler("/attack",       hAttack);
  _h.attachHandler("/straight=",    hStraight);
  _h.attachHandler("/goto_vive?x=", hViveNav);
}

void WebController::serve() { _h.serve(); }

// Static trampolines — each fetches the live instance and forwards.
void WebController::hRoot() {
  if (!s_self) return;
  s_self->_h.sendhtml(body);
}

void WebController::hDir() {
  if (!s_self) return;
  incrementPacketCount();
  String d = s_self->_h.getText();
  int L = 0, R = 0;
  if      (d == "F") { L =  1; R =  1; }
  else if (d == "B") { L = -1; R = -1; }
  else if (d == "L") { L = -1; R =  1; }
  else if (d == "R") { L =  1; R = -1; }
  s_self->_md.setDirection(L, R);
  s_self->_h.sendhtml(body);
}

void WebController::hSpeed() {
  if (!s_self) return;
  incrementPacketCount();
  float rpm = s_self->_h.getVal();
  s_self->_md.setTargetRPM(rpm);
  Serial.printf("Manual target RPM: %.1f\n", rpm);
  s_self->_h.sendhtml(body);
}

void WebController::hKp() {
  if (!s_self) return;
  incrementPacketCount();
  float v = s_self->_h.getVal();
  s_self->_md.setKp(v);
  Serial.printf("Kp: %.2f\n", v);
  s_self->_h.sendhtml(body);
}

void WebController::hKi() {
  if (!s_self) return;
  incrementPacketCount();
  float v = s_self->_h.getVal();
  s_self->_md.setKi(v);
  Serial.printf("Ki: %.2f\n", v);
  s_self->_h.sendhtml(body);
}

void WebController::hKd() {
  if (!s_self) return;
  incrementPacketCount();
  float v = s_self->_h.getVal();
  s_self->_md.setKd(v);
  Serial.printf("Kd: %.2f\n", v);
  s_self->_h.sendhtml(body);
}

void WebController::hMode() {
  if (!s_self) return;
  incrementPacketCount();
  int mode = s_self->_h.getVal();
  if (s_self->_onMode) s_self->_onMode(mode);
  s_self->_h.sendhtml(body);
}

void WebController::hWfKp() {
  if (!s_self) return;
  incrementPacketCount();
  float v = s_self->_h.getVal();
  s_self->_wf.setKp(v);
  Serial.printf("wf_Kp: %.2f\n", v);
  s_self->_h.sendhtml(body);
}

void WebController::hWfKd() {
  if (!s_self) return;
  incrementPacketCount();
  float v = s_self->_h.getVal();
  s_self->_wf.setKd(v);
  Serial.printf("wf_Kd: %.2f\n", v);
  s_self->_h.sendhtml(body);
}

void WebController::hSharpTurn() {
  if (!s_self) return;
  incrementPacketCount();
  int v = (int)s_self->_h.getVal();
  s_self->_wf.setSharpTurnOffset(v);
  Serial.printf("sharpTurnOffset: %d\n", v);
  s_self->_h.sendhtml(body);
}

void WebController::hAttack() {
    if (!s_self) return;
    incrementPacketCount();
    extern Attacker arm; // Access the global arm object
    arm.toggle();
    s_self->_h.sendhtml(body);
}

void WebController::hStraight() {
  if (!s_self) return;
  incrementPacketCount();
  int inches = (int)s_self->_h.getVal();
  Serial.printf("Web /straight= %d\n", inches);
  if (s_self->_onStraight) s_self->_onStraight(inches);
  s_self->_h.sendhtml(body);
}
void WebController::hViveNav() {
    if (!s_self) return;
    incrementPacketCount();

    // 1. Get the X value normally (it's at the very start of our handler path)
    int xVive = s_self->_h.getVal(); 

    // 2. Grab the entire remaining part of the URL string
    // This should grab something like "&y=3000"
    String remainder = s_self->_h.getText(); 

    // 3. Manually find the '=' sign and convert what follows it to an integer
    int yVive = 0;
    int equalIndex = remainder.indexOf('=');
    if (equalIndex != -1) {
        // substring gets everything after the '='
        yVive = remainder.substring(equalIndex + 1).toInt();
    }

    // Debugging: This will show you exactly what we grabbed
    
    extern ViveNavigation viveNav; 
    viveNav.setTargetVive(xVive, yVive);
    
    if (s_self->_onMode) s_self->_onMode(10);
    s_self->_h.sendhtml(body);
}