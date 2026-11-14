#include <cstdio>
#include <cmath>

#include <FreeRTOS.h>
#include <queue.h>
#include <task.h>

#include "pico/stdlib.h"
#include "hardware/i2c.h"

#include "config.h"
#include "types.h"
#include "globals.h"
#include "adc.h"
#include "processing.h"
#include "display.h"
#include "wifi_net.h"
#include "influx.h"
#include "hardware/adc.h"

#include "ssd1306/SSD1306_OLED.hpp"

SSD1306 myOLED(myOLEDwidth, myOLEDheight);

// pico
// V = 1.02589950
// I =  0.87094189

// pico-debug
// V = 1.03605
// I = 0.86063

const float calibrationVGain = 1.02589950;
const float calibrationIGain = 0.87094189;

const float vGain = (1.0 / ((1 / 120000.0) * 50 * (47 / 4.7) * (4096 / 3.3))) * calibrationVGain;
const float iGain = (1.0 / (1. / 30. * 4096 / 3.3)) * calibrationIGain;

int main() {
  stdio_init_all();

  for (int i = 0; i < 6; ++i) {
    printf("Booting in %d...\n", 6 - i);
    sleep_ms(1000);
  }

  setupDisplay(5000);
  busy_wait_ms(1000);
  setupADC();
  busy_wait_ms(1000);

  waveFullQueue  = xQueueCreate(6, sizeof(EanWaveform *));
  waveEmptyQueue = xQueueCreate(6, sizeof(EanWaveform *));
  rmsFullQueue   = xQueueCreate(6, sizeof(RMSWaveform *));
  rmsEmptyQueue  = xQueueCreate(6, sizeof(RMSWaveform *));

  for (int i = 0; i < 6; i++) {
    EanWaveform *ean = &eanWaveformBuffers[i];
    RMSWaveform *rms = &rmsWaveformBuffers[i];
    xQueueSend((QueueHandle_t)waveEmptyQueue, &ean, portMAX_DELAY);
    xQueueSend((QueueHandle_t)rmsEmptyQueue,  &rms, portMAX_DELAY);
  }

  CalculateStuffArgs calculateArgs{ waveFullQueue, waveEmptyQueue, rmsFullQueue, rmsEmptyQueue, iGain, vGain };
  DisplayStuffArgs displayArgs{ rmsFullQueue, rmsEmptyQueue };

  printf("Creating tasks...\n\n");
  xTaskCreate((TaskFunction_t)connectWifiTask, "wifiTask", 4096, NULL, 24, NULL);
  xTaskCreate((TaskFunction_t)calculateStuff, "calculateStuff", 2048, &calculateArgs, 20, NULL);
  xTaskCreate((TaskFunction_t)displayStuff, "displayStuff", 4096, &displayArgs, 12, NULL);
  
  adc_run(true);
  printf("Starting FreeRTOS-SMP scheduler...\n\n");
  vTaskStartScheduler();

  turnOffDisplay();

  for(;;) { tight_loop_contents(); }
}
