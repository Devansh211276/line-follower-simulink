#ifndef CONFIGURATION_H
#define CONFIGURATION_H
#include <Arduino.h>
const uint8_t IN1 = 5, IN2 = 6, IN3 = 9, IN4 = 10;
const uint8_t SensorCount = 8;
const uint8_t SensorPins[SensorCount] = {A0,A1,A2,A3,A4,A5,A6,A7};
// Assumes sensor 0 is physically leftmost and both motors run forward
// for positive PWM. Confirm these assumptions before running on the track.
float Kp = 5.0f;                  // PWM units per sensor pitch
int baseSpeed = 90;
int steeringSign = 1;             // Change with SIGN:-1 if steering is reversed
bool whiteLine = false;           // BLACKLINE or WHITELINE command
const int MIN_CAL_SPAN = 80;      // Raw ADC units, assuming default 10-bit ADC
const unsigned long SAMPLE_MS = 20;
#endif
