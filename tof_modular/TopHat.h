#ifndef TOPHAT_H
#define TOPHAT_H

#include <Arduino.h>
#include <Wire.h>

// =====================================================================
// TopHat: Requests health from tophat and sends number of WiFi packets used 
//at 2 Hz. Stops all motors from running if health = 0
// =====================================================================
static int SDA_pin = 11;
static int SCL_pin = 14;
extern int packetCounter;
static int packetsInLastWindow = 0;
static unsigned long lastPacketWindow = 0;
static const int windowMs = 500; //500ms = 2Hz
extern uint8_t health;

inline void TopHat() {
  if (millis() - lastPacketWindow >= windowMs) {
    
    Wire1.beginTransmission(0x28); //send packet count to Top Hat
    Wire1.write(packetCounter);
    Wire1.endTransmission();
    
    int b = Wire1.requestFrom(0x28, (uint8_t)1); //request health from Top Hat
    Serial.println(b);
    if (b == 1) {
      health = Wire1.read();
      Serial.println(health);
    }
    lastPacketWindow = millis();
    packetsInLastWindow = packetCounter; 
    packetCounter = 0;
  }
}

inline void incrementPacketCount() {
  packetCounter++;
}

#endif
