#include "WebController.h"
#include "PIDandTOF_web.h"   // provides `body` HTML
#include "TopHat.h"
#include "Attacker.h"
#include "ViveNavigation.h"
#include "AttackTopTower.h"
#include "RobotPosition.h"
#include "NavigationTools.h"

WebController* WebController::s_self = nullptr;

WebController::WebController(ManualDrive& md, WallFollow& wf, ModeCallback onMode)
  : _md(md), _wf(wf), _onMode(onMode), _h(80)
{
  s_self = this;
}

void WebController::begin(const char* ssid, const char* password) {
  // WiFi.begin(ssid, password);
  // while (WiFi.status() != WL_CONNECTED) { delay(500); Serial.print("."); }
  // Serial.println("\nWiFi connected");
  // Serial.print("IP: "); Serial.println(WiFi.localIP());
  WiFi.softAP(ssid, password, 11);
  Serial.println("\nAP mode started");
  Serial.print("AP IP: "); Serial.println(WiFi.softAPIP());

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
  _h.attachHandler("/calibrate=",   hViveCalibrate);
  _h.attachHandler("/coeff",          hCoeff);
  _h.attachHandler("/state",        hState);
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

// /state -> tiny JSON blob the front-end polls for telemetry.
// Pulls AttackTopTower bridge-gating counters and the Vive MID position
// from the globals declared in tof_modular.ino.
void WebController::hState() {
  if (!s_self) return;
  extern AttackTopTower attackTopTower;
  extern RobotPosition  robotPos;

  RobotPosition::Position mid = robotPos.getRobotPosition(MID);

  char buf[160];
  // Keep keys short; the front-end parses by name.
  snprintf(buf, sizeof(buf),
           "{\"entryHits\":%u,\"entryFlag\":%s,\"trigHits\":%u,\"exitHits\":%u,"
           "\"x\":%.1f,\"y\":%.1f}",
          //  (unsigned)attackTopTower.entryHits(),
          //  attackTopTower.entryFlag() ? "true" : "false",
          //  (unsigned)attackTopTower.trigHits(),
          //  (unsigned)attackTopTower.exitHits(),
           mid.x, mid.y);
  s_self->_h.sendplain(String(buf));
}

// /calibrate=Xv2,Yv2,Xv3,Yv3,Xv4,Yv4,Xv5,Yv5,Xv7,Yv7,Xv8,Yv8
// Parses 12 comma-separated Vive coordinates (6 points × X,Y), runs
// multiple linear regression (equivalent to Google Sheets LINEST), and
// updates NavigationTools calibration coefficients live on the robot.
// Returns a JSON object with ok:true/false and the new coefficients.
void WebController::hViveCalibrate() {
    if (!s_self) return;
    incrementPacketCount();

    // Known real coordinates (inches from center of low tower) for
    // calibration points, ordered: 2, 3, 4, 5, 7, 8.
    static const float KNOWN_X[6] = { -24.0f, -5.75f,  5.75f, 24.0f,  0.0f,   0.0f   };
    static const float KNOWN_Y[6] = {   0.0f,  0.0f,   0.0f,  0.0f,  17.75f, -15.75f };

    // Read CSV payload: everything after "/calibrate=" up to the space
    // before "HTTP/1.1".  getText() stops at any character <= ' '.
    String data = s_self->_h.getText();

    // Parse 12 comma-separated floats into vx[0..5], vy[0..5]
    float vx[6], vy[6];
    int   idx   = 0;   // total token counter (0-11)
    int   start = 0;   // start of current token in `data`

    for (int i = 0; i <= (int)data.length() && idx < 12; i++) {
        if (i == (int)data.length() || data[i] == ',') {
            float val = data.substring(start, i).toFloat();
            if (idx % 2 == 0) vx[idx / 2] = val;
            else               vy[idx / 2] = val;
            idx++;
            start = i + 1;
        }
    }

    if (idx < 12) {
        // Didn't receive all 12 values — bad request
        s_self->_h.sendplain("{\"ok\":false,\"err\":\"parse\"}");
        Serial.printf("hViveCalibrate: parse error, only got %d tokens\n", idx);
        return;
    }

    Serial.println("hViveCalibrate: received Vive points:");
    for (int i = 0; i < 6; i++) {
        Serial.printf("  pt%d: Vive(%.1f, %.1f) -> Real(%.2f, %.2f)\n",
                      i, vx[i], vy[i], KNOWN_X[i], KNOWN_Y[i]);
    }

    // Run least-squares regression; updates NavigationTools::mx_xr etc. directly
    bool ok = NavigationTools::runViveRegression(vx, vy, 6, KNOWN_X, KNOWN_Y);

    if (!ok) {
        s_self->_h.sendplain("{\"ok\":false,\"err\":\"singular\"}");
        Serial.println("hViveCalibrate: regression failed (singular matrix)");
        return;
    }

    Serial.printf("hViveCalibrate: updated coefficients:\n");
    Serial.printf("  Xreal = %.8f*Xv + %.8f*Yv + %.6f\n",
                  NavigationTools::mx_xr, NavigationTools::my_xr, NavigationTools::c_xr);
    Serial.printf("  Yreal = %.8f*Xv + %.8f*Yv + %.6f\n",
                  NavigationTools::mx_yr, NavigationTools::my_yr, NavigationTools::c_yr);

    // Return JSON with the new coefficients so the web UI can display them
    char buf[300];
    snprintf(buf, sizeof(buf),
             "{\"ok\":true,"
             "\"mx_xr\":%.8f,\"my_xr\":%.8f,\"c_xr\":%.6f,"
             "\"mx_yr\":%.8f,\"my_yr\":%.8f,\"c_yr\":%.6f}",
             NavigationTools::mx_xr, NavigationTools::my_xr, NavigationTools::c_xr,
             NavigationTools::mx_yr, NavigationTools::my_yr, NavigationTools::c_yr);
    s_self->_h.sendplain(String(buf));
}

// /coeff -> JSON of the six current NavigationTools calibration coefficients.
// Called by the page on load so the JS coefficient store stays in sync with
// whatever is live on the robot (even if calibration ran in a prior session).
void WebController::hCoeff() {
    if (!s_self) return;
    char buf[256];
    snprintf(buf, sizeof(buf),
             "{\"mx_xr\":%.9f,\"my_xr\":%.9f,\"c_xr\":%.6f,"
             "\"mx_yr\":%.9f,\"my_yr\":%.9f,\"c_yr\":%.6f}",
             NavigationTools::mx_xr, NavigationTools::my_xr, NavigationTools::c_xr,
             NavigationTools::mx_yr, NavigationTools::my_yr, NavigationTools::c_yr);
    s_self->_h.sendplain(String(buf));
}
