#include "globals.h"

volatile bool isConnected = false;
volatile bool isDisplayConnected = false;

volatile EanWaveform *buffer = 0;
volatile int currentId = 0;
volatile int currentBufferIndex = 0;

QueueHandle_t waveFullQueue  = nullptr;
QueueHandle_t waveEmptyQueue = nullptr;
QueueHandle_t rmsFullQueue   = nullptr;
QueueHandle_t rmsEmptyQueue  = nullptr;

EanWaveform eanWaveformBuffers[6];
RMSWaveform rmsWaveformBuffers[6];
