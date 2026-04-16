#include <Wire.h>
#include "Adafruit_VL53L0X.h"

#define SDA_PIN   8
#define SCL_PIN   9

#define XSHUT_1   10
#define XSHUT_2   18

#define LOX1_ADDR 0x30
#define LOX2_ADDR 0x31

Adafruit_VL53L0X lox1;
Adafruit_VL53L0X lox2;

bool i2cFound(uint8_t addr) {
  Wire.beginTransmission(addr);
  return (Wire.endTransmission() == 0);
}

void printI2CScan() {
  Serial.println("I2C scan:");
  int found = 0;
  for (uint8_t addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      Serial.print("  found 0x");
      if (addr < 16) Serial.print("0");
      Serial.println(addr, HEX);
      found++;
    }
  }
  if (!found) Serial.println("  none");
}

void setup() {
  Serial.begin(115200);
  delay(1500);

  Serial.println("\nDual VL53L0X test");

  Wire.begin(SDA_PIN, SCL_PIN);
  delay(50);

  pinMode(XSHUT_1, OUTPUT);
  pinMode(XSHUT_2, OUTPUT);

  // Hold both sensors in shutdown
  digitalWrite(XSHUT_1, LOW);
  digitalWrite(XSHUT_2, LOW);
  delay(50);

  printI2CScan();

  // ---- Sensor 1 ----
  Serial.println("Enabling ToF1...");
  digitalWrite(XSHUT_1, HIGH);
  delay(100);

  if (!i2cFound(0x29)) {
    Serial.println("ToF1 did not appear at 0x29");
    while (1);
  }
  Serial.println("ToF1 seen at 0x29");

  if (!lox1.begin(LOX1_ADDR, false, &Wire)) {
    Serial.println("Failed to boot VL53L0X #1");
    while (1);
  }
  Serial.print("VL53L0X #1 started at 0x");
  Serial.println(LOX1_ADDR, HEX);
  printI2CScan();

  // ---- Sensor 2 ----
  Serial.println("Enabling ToF2...");
  digitalWrite(XSHUT_2, HIGH);
  delay(100);

  // At this moment sensor 1 should be at 0x30, sensor 2 should appear at 0x29
  printI2CScan();

  if (!i2cFound(0x29)) {
    Serial.println("ToF2 did not appear at 0x29");
    while (1);
  }
  Serial.println("ToF2 seen at 0x29");

  if (!lox2.begin(LOX2_ADDR, false, &Wire)) {
    Serial.println("Failed to boot VL53L0X #2");
    while (1);
  }
  Serial.print("VL53L0X #2 started at 0x");
  Serial.println(LOX2_ADDR, HEX);

  printI2CScan();
  Serial.println("Both VL53L0X sensors initialized.");
}

void loop() {
  VL53L0X_RangingMeasurementData_t m1, m2;

  lox1.rangingTest(&m1, false);
  lox2.rangingTest(&m2, false);

  Serial.print("ToF1: ");
  if (m1.RangeStatus != 4) {
    Serial.print(m1.RangeMilliMeter);
    Serial.print(" mm");
  } else {
    Serial.print("Out of range");
  }

  Serial.print(" | ToF2: ");
  if (m2.RangeStatus != 4) {
    Serial.print(m2.RangeMilliMeter);
    Serial.println(" mm");
  } else {
    Serial.println("Out of range");
  }

  delay(200);
}