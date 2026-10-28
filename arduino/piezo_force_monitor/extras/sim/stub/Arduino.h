// Host stub of the Arduino API used by piezo_force_monitor.ino (simulation only)
#pragma once
#include <cstdint>
#include <cstddef>
#include <cstdio>
#include <cmath>
#include <cstring>
#include <cstdlib>
#define HIGH 1
#define LOW 0
#define INPUT 0
#define OUTPUT 1
#define INPUT_PULLUP 2
#define A0 14
#define A1 15
#define A2 16
extern uint64_t g_ns;                       // simulated time, ns
void simAdvance(uint64_t ns);               // advances time; runs the "sampler thread"
int  simAnalogRead(int pin);
int  simDigitalRead(int pin);
inline unsigned long millis() { return (unsigned long)(g_ns / 1000000ULL); }
inline unsigned long micros() { return (unsigned long)(g_ns / 1000ULL); }
inline void delay(unsigned long ms) { simAdvance((uint64_t)ms * 1000000ULL); }
inline void pinMode(int, int) {}
inline int  digitalRead(int pin) { return simDigitalRead(pin); }
inline void analogReadResolution(int) {}
inline int  analogRead(int pin) { return simAnalogRead(pin); }
