#include <cstdio>
#include "display.h"
#include "config.h"
#include "globals.h"
#include "influx.h"
#include "ssd1306/SSD1306_OLED.hpp"

extern SSD1306 myOLED;  

static uint8_t screenBuffer[myScreenSize];
const uint16_t I2C_Speed = 100;
const uint8_t  I2C_Address = 0x3C;

void setupDisplay(uint32_t timeout_ms = 5000){
  busy_wait_ms(500);
  const uint32_t t0 = to_ms_since_boot(get_absolute_time());
  printf("Starting display setup...\n");

  const bool okBegin = myOLED.OLEDbegin(I2C_Address, i2c1, I2C_Speed, 18, 19);

  if (!okBegin) {
    while ((to_ms_since_boot(get_absolute_time()) - t0) < timeout_ms) {
      tight_loop_contents(); 
    }
    isDisplayConnected = false;
    printf("Display not found (timeout).\n\n");
    return;
  }

  if (myOLED.OLEDSetBufferPtr(myOLEDwidth, myOLEDheight, screenBuffer, sizeof(screenBuffer) / sizeof(uint8_t)) != 0) {
    isDisplayConnected = false;
    printf("Display buffer set failed. Running headless.\n\n");
    return; 
  }

  myOLED.OLEDFillScreen(0xF0, 0);
  printf("Finished display setup...\n\n");
  isDisplayConnected = true;
  busy_wait_ms(1000);
}

void turnOffDisplay() {
  if(isDisplayConnected){
    myOLED.OLEDPowerDown();
    myOLED.OLEDdeI2CInit();
  }
}

void printDisplayData(const RMSWaveform *r) {
  printf("V(RMS): %f\n", r->voltage);
  printf("I(RMS): %f\n", r->current);
  printf("S(VA): %f\n", r->power);
  printf("P(W): %f\n", r->activePower);
  printf("Q(VAr): %f\n", r->reactivePower);
  printf("FP: %f\n\n", r->powerFactor);

  if(isDisplayConnected){
    myOLED.OLEDclearBuffer();
    myOLED.setFont(pFontDefault);
    myOLED.setCursor(5, 5);
    myOLED.print("V(RMS): ");
    myOLED.print(r->voltage);
    myOLED.print("V\n\n");
    myOLED.print("I(RMS): ");
    myOLED.print(r->current);
    myOLED.print("A   ");
    if(isConnected) {
      myOLED.print("|\n");
    }
    else{
      myOLED.print("-\n");
    }
    myOLED.print("S: ");
    myOLED.print(r->power);
    myOLED.print("VA\n");
    myOLED.print("P: ");
    myOLED.print(r->activePower);
    myOLED.print("W  ");
    myOLED.print("fp: ");
    myOLED.print(r->powerFactor);
    myOLED.print("\n");
    myOLED.print("Q: ");
    myOLED.print(r->reactivePower);
    myOLED.print("VAr");
    myOLED.OLEDupdate();
  }
  
  // if(LOG_BUFFERS == 1){
  //   int n = SAMPLE_AMOUNT;
  //   printf("voltages: [");
  //   for (int i=0;i<n;i++) printf("%d%s", r->voltages[i], (i+1<n)?", ":"");
  //   printf("]\n");

  //   printf("currents: [");
  //   for (int i=0;i<n;i++) printf("%d%s", r->currents[i], (i+1<n)?", ":"");
  //   printf("]\n");
  // }
}

#include <FreeRTOS.h>
#include <queue.h>
#include <task.h>

void displayStuff(DisplayStuffArgs *args) {
  while (true) {
    RMSWaveform *rmsBuffer = 0;
    while (!xQueueReceive((QueueHandle_t)args->rmsFull, &rmsBuffer, portMAX_DELAY));
    printDisplayData(rmsBuffer);
    sendToInflux(rmsBuffer);
    vTaskDelay(5000);
    while (!xQueueSend((QueueHandle_t)args->rmsEmpty, &rmsBuffer, portMAX_DELAY));
  }
}
