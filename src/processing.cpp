#include <cmath>
#include <cstdio>
#include "processing.h"
#include "globals.h"
#include "config.h"
#include <FreeRTOS.h>
#include <queue.h>
#include <task.h>
#include <cstring>

float calculateRMS(const int16_t *samples, int sampleAmount, float gain) {
  float sum = 0;
  float amp = 0;
  for (int i = 0; i < sampleAmount; i++) {
    sum += samples[i];
  }
  float meas = sum / (float)sampleAmount;
  
  for (int i = 0; i < sampleAmount; i++) {
    amp += (samples[i] - meas) * (samples[i] - meas);
  }
  float rms = sqrt(amp / (float)sampleAmount);
  return rms * gain;
}

void calculateStuff(CalculateStuffArgs *args) {
  while (true) {
    EanWaveform *eanBuffer = 0;
    RMSWaveform *rmsBuffer = 0;

    int N = SAMPLE_AMOUNT;

    while (!xQueueReceive((QueueHandle_t)args->waveFull, &eanBuffer, portMAX_DELAY));
    while (!xQueueReceive((QueueHandle_t)args->rmsEmpty, &rmsBuffer, portMAX_DELAY));

    // memcpy(rmsBuffer->voltages, eanBuffer->voltage, sizeof(eanBuffer->voltage));
    // memcpy(rmsBuffer->currents, eanBuffer->current, sizeof(eanBuffer->current));

    float voltageRMS = calculateRMS(eanBuffer->voltage, sizeof(eanBuffer->voltage) / sizeof(int16_t), args->vGain);
    float currentRMS = calculateRMS(eanBuffer->current, sizeof(eanBuffer->current) / sizeof(int16_t), args->iGain);
    
    float instantPowerSum = 0.0;
    float voltageMean = 0.0;
    float currentMean = 0.0;

    for (int i = 0; i < N; i++) {
      voltageMean += eanBuffer->voltage[i];
      currentMean += eanBuffer->current[i];
    }

    voltageMean = voltageMean / (float)N;
    currentMean = currentMean / (float)N;

    for (int i = 0; i < N; i++) {
      instantPowerSum += -(eanBuffer->voltage[i] - voltageMean) * (eanBuffer->current[i] - currentMean);
    }

    float power = voltageRMS * currentRMS;
    float powerGain = (args->vGain * args->iGain);
    float activePower = powerGain * (instantPowerSum / (float)N);

    float reactivePower = sqrt(pow(power, 2) - pow(activePower, 2));

    float powerFactor = activePower / power;

    rmsBuffer->id = eanBuffer->id;
    rmsBuffer->voltage = voltageRMS;
    rmsBuffer->current = currentRMS;
    rmsBuffer->power = power;
    rmsBuffer->activePower = activePower;
    rmsBuffer->reactivePower = reactivePower;
    rmsBuffer->powerFactor = powerFactor;
    while (!xQueueSend((QueueHandle_t)args->waveEmpty, &eanBuffer, portMAX_DELAY));
    while (!xQueueSend((QueueHandle_t)args->rmsFull, &rmsBuffer, portMAX_DELAY));
  }
}
