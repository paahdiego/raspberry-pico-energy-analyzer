#pragma once
#include <stdint.h>

#include "config.h"

struct EanWaveform {
  uint32_t id;
  int16_t  voltage[SAMPLE_AMOUNT], current[SAMPLE_AMOUNT];
};

struct RMSWaveform {
  uint32_t id;
  float voltage, current, activePower, reactivePower, power, powerFactor;
  // int16_t  voltages[SAMPLE_AMOUNT], currents[SAMPLE_AMOUNT];
};

struct CalculateStuffArgs {
  void* waveFull;   
  void* waveEmpty;
  void* rmsFull;
  void* rmsEmpty;
  float iGain;
  float vGain;
};

struct DisplayStuffArgs {
  void* rmsFull;
  void* rmsEmpty;
};
