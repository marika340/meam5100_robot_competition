#ifndef ATTACKER_H
#define ATTACKER_H

#include <Arduino.h>

class Attacker {
public:
    // Pass in your pre-defined constants
    Attacker(int pin, int channel, int window) 
      : _pin(pin), _channel(channel), _window(window), 
        _active(false), _left(0), _lastAttackTime(0) {}

    void begin() {
        // Setup LEDC for Servo (50Hz, 14-bit)
        ledcAttachChannel(_pin, 50, 14, _channel);
        stop();
    }

    void start() { _active = true; }
    void stop()  { _active = false; ledcWrite(_pin, 0); }
    void toggle() { if (_active) stop(); else start(); }

    // This is the concurrent logic from your original code
    void update() {
        if (_active) {
            if (millis() - _lastAttackTime >= _window) {
                _lastAttackTime = millis();
                if (_left == 0) {
                    _left = 1;
                    ledcWrite(_pin, 819);  // duty_1ms
                } else {
                    _left = 0;
                    ledcWrite(_pin, 1638); // duty_2ms
                }
            }
        } else {
            // attack_flag == 0 logic
            ledcWrite(_pin, 0);
        }
    }

private:
    int _pin, _channel, _window;
    bool _active;
    int _left;
    unsigned long _lastAttackTime;
};

#endif