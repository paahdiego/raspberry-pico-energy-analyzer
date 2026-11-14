#pragma once
#include <stdbool.h>
#include "types.h"

#include <FreeRTOS.h>
#include <queue.h>

#ifdef __cplusplus
extern "C" {
#endif
extern volatile bool isConnected;
extern volatile bool isDisplayConnected;

extern volatile EanWaveform *buffer;
extern volatile int currentId;
extern volatile int currentBufferIndex;

extern QueueHandle_t waveFullQueue; 
extern QueueHandle_t waveEmptyQueue;
extern QueueHandle_t rmsFullQueue;
extern QueueHandle_t rmsEmptyQueue;

extern EanWaveform eanWaveformBuffers[6];
extern RMSWaveform rmsWaveformBuffers[6];
#ifdef __cplusplus
}
#endif
