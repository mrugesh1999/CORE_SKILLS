// Host stub of the UNO Q Bridge Monitor (each call costs RPC time)
#pragma once
#include <cstdint>
#include <cstddef>
struct MonitorStub {
  bool   begin(unsigned long = 0) { return true; }
  size_t write(const uint8_t* b, size_t n);
  int    available();
  int    read();
};
extern MonitorStub Monitor;
