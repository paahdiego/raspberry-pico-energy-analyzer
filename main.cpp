#define DEBUG_TAG() printf("Line: %d\n", __LINE__)
#include <FreeRTOS.h>
#include <queue.h>
#include <task.h>

#include "hardware/adc.h"
#include "hardware/gpio.h"
#include "pico/stdlib.h"
#include "ssd1306/SSD1306_OLED.hpp"
#include <cstdio>

#include <pico/cyw43_arch.h>
#include <pico/lwip_freertos.h>
#include <pico/async_context_freertos.h>

#include <lwip/raw.h>
#include <lwip/tcp.h>

#define SAMPLE_RATE 12000
#define SAMPLE_AMOUNT 100

#define myOLEDwidth 128
#define myOLEDheight 64
#define myScreenSize (myOLEDwidth * (myOLEDheight / 8))
uint8_t screenBuffer[myScreenSize];

const uint16_t I2C_Speed = 100;
const uint8_t I2C_Address = 0x3C;

SSD1306 myOLED(myOLEDwidth, myOLEDheight);


struct EanWaveform {
  uint32_t id;
  int16_t voltage[100], current[100];
};

struct RMSWaveform {
  uint32_t id;
  float voltage, current, activePower, reactivePower, power, powerFactor;
};

struct CalculateStuffArgs {
  QueueHandle_t waveFull, waveEmpty, rmsFull, rmsEmpty;
  float iGain;
  float vGain;
};

struct DisplayStuffArgs {
  QueueHandle_t rmsFull, rmsEmpty;
};

static void connectWifi(void *) {
    printf("Starting wifi...\n");
    if (cyw43_arch_init()) {
        printf("failed to initialise\n");
        while(true);
    }
    printf("Enabling station mode...\n");
    cyw43_arch_enable_sta_mode();
    printf("Connecting to %s...\n", WIFI_SSID);
    if (cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PASSWORD, CYW43_AUTH_WPA2_AES_PSK, 30000)) {
        printf("failed to connect.\n");
        exit(1);
    } else {
        printf("Connected.\n");
    }

    int i=0;
    while(true) {
        uint32_t ip = netif_ip4_addr(netif_default)->addr;
        printf("%d: %d.%d.%d.%d\n",
            i++,
            ip % 256,
            ip / 256 % 256,
            ip / 256 / 256 % 256,
            ip / 256 / 256 / 256 % 256
        );

        vTaskDelay(configTICK_RATE_HZ);
    }
}

void setupDisplay(void);
void turnOffDisplay();
void printDisplayData(const RMSWaveform *rmsWaveform);
void setupADC();
void adcIRQHandler();
float calculateRMS(const int16_t *samples, int sampleAmount, float gain);

static void calculateStuff(CalculateStuffArgs *args) {
  while (true) {
    EanWaveform *eanBuffer = 0;
    RMSWaveform *rmsBuffer = 0;

    while (!xQueueReceive(args->waveFull, &eanBuffer, portMAX_DELAY));
    
    while (!xQueueReceive(args->rmsEmpty, &rmsBuffer, portMAX_DELAY));
    
    float voltageRMS = calculateRMS(eanBuffer->voltage, sizeof(eanBuffer->voltage) / sizeof(int16_t), args->vGain);
    float currentRMS = calculateRMS(eanBuffer->current, sizeof(eanBuffer->current) / sizeof(int16_t), args->iGain);

    float instantPowerSum = 0.0;
    float voltageMean = 0.0;
    float currentMean = 0.0;

    for (int i = 0; i < 100; i++) {
      voltageMean += eanBuffer->voltage[i];
      currentMean += eanBuffer->current[i];
    }

    voltageMean = voltageMean / 100.0;
    currentMean = currentMean / 100.0;
    
    for (int i = 0; i < 100; i++) {
      instantPowerSum += (eanBuffer->voltage[i] - voltageMean) * (eanBuffer->current[i] - currentMean);
    }

    float power = voltageRMS * currentRMS;
    float powerGain = (args->vGain * args->iGain);
    float activePower = powerGain * (instantPowerSum / 100.0);
    
    float reactivePower = sqrt(pow(power, 2) - pow(activePower, 2));

    float powerFactor = activePower / power;

    rmsBuffer->id = eanBuffer->id;
    rmsBuffer->voltage = voltageRMS;
    rmsBuffer->current = currentRMS;
    rmsBuffer->power = power;
    rmsBuffer->activePower = activePower;
    rmsBuffer->reactivePower = reactivePower;
    rmsBuffer->powerFactor = powerFactor;

    while (!xQueueSend(args->waveEmpty, &eanBuffer, portMAX_DELAY));
    while (!xQueueSend(args->rmsFull, &rmsBuffer, portMAX_DELAY));
  }
}

static void displayStuff(DisplayStuffArgs *args) {
  while(true){
    RMSWaveform *rmsBuffer = 0;
    
    while (!xQueueReceive(args->rmsFull, &rmsBuffer, portMAX_DELAY));
        
    printDisplayData(rmsBuffer);
    
    while (!xQueueSend(args->rmsEmpty, &rmsBuffer, portMAX_DELAY));
    
  }
}

volatile EanWaveform *buffer = 0;
volatile int currentId = 0;
volatile int currentBufferIndex = 0;

QueueHandle_t waveFullQueue, waveEmptyQueue;
EanWaveform eanWaveformBuffers[6];

QueueHandle_t rmsFullQueue, rmsEmptyQueue;
RMSWaveform rmsWaveformBuffers[6];

int main() {
  stdio_init_all();
  gpio_init(5);
  gpio_set_dir(5, GPIO_OUT);
  gpio_put(5, 1);

  for (int i=0; i<10; ++i) {
    printf("Booting in %d...\n", 10-i);
    sleep_ms(1000);
  }

  printf("%s\n" , WIFI_SSID);
  printf("%s\n" , WIFI_PASSWORD);
  
  setupDisplay();
  setupADC();

  waveFullQueue = xQueueCreate(6, sizeof(EanWaveform *));
  waveEmptyQueue = xQueueCreate(6, sizeof(EanWaveform *));

  rmsFullQueue = xQueueCreate(6, sizeof(RMSWaveform *));
  rmsEmptyQueue = xQueueCreate(6, sizeof(RMSWaveform *));

  for (int i = 0; i < 6; i++) {
    EanWaveform *ean = &eanWaveformBuffers[i];
    RMSWaveform *rms = &rmsWaveformBuffers[i];
    xQueueSend(waveEmptyQueue, &ean, portMAX_DELAY);
    xQueueSend(rmsEmptyQueue, &rms, portMAX_DELAY);
  }

  float vGain = 1 /  ((1 / 120000.0) * 50 * (47/4.7) * (4096/3.3));
  float iGain = 1 / ((1 / 1000.0) * 50 * (10/4.7) * (4096/3.3));

  CalculateStuffArgs calculateArgs{
    waveFullQueue,
    waveEmptyQueue,
    rmsFullQueue,
    rmsEmptyQueue, 
    iGain,
    vGain
  };

  DisplayStuffArgs displayArgs{rmsFullQueue, rmsEmptyQueue};

  xTaskCreate(connectWifi, "connectWifi", 4096, NULL, 10, NULL);
  
  xTaskCreate((TaskFunction_t)calculateStuff, "calculateStuff", 2048, &calculateArgs, 15, NULL);
  xTaskCreate((TaskFunction_t)displayStuff, "displayStuff", 2048, &displayArgs,10, NULL);

  printf("Starting FreeRTOS-SMP scheduler...\n");
  vTaskStartScheduler();

  turnOffDisplay();
}

void setupDisplay(void) {
  busy_wait_ms(500);

  while (myOLED.OLEDbegin(I2C_Address, i2c1, I2C_Speed, 18, 19) != true) {
    busy_wait_ms(1500);
  }
  if (myOLED.OLEDSetBufferPtr(myOLEDwidth, myOLEDheight, screenBuffer,
                              sizeof(screenBuffer) / sizeof(uint8_t)) != 0) {
    while (1) {
      busy_wait_ms(1000);
    }
  }
  myOLED.OLEDFillScreen(0xF0, 0);
  busy_wait_ms(1000);
}

void printDisplayData(const RMSWaveform *rmsWaveform) {
  myOLED.OLEDclearBuffer();
  myOLED.setFont(pFontDefault);
  myOLED.setCursor(5, 5);
  myOLED.print("V(RMS): ");
  myOLED.print(rmsWaveform->voltage);
  myOLED.print("V\n\n");
  myOLED.print("I(RMS): ");
  myOLED.print(rmsWaveform->current);
  myOLED.print("A\n");
  myOLED.print("S: ");
  myOLED.print(rmsWaveform->power);
  myOLED.print("VA\n");
  myOLED.print("P: ");
  myOLED.print(rmsWaveform->activePower);
  myOLED.print("W  ");
  myOLED.print("fp: ");
  myOLED.print(rmsWaveform->powerFactor);
  myOLED.print("\n");
  myOLED.print("Q: ");
  myOLED.print(rmsWaveform->reactivePower);
  myOLED.print("VAr");
  myOLED.OLEDupdate();
}

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

void turnOffDisplay() {
  myOLED.OLEDPowerDown();
  myOLED.OLEDdeI2CInit();
};

void adcIRQHandler() {
  BaseType_t wake = 0;
  while (adc_fifo_get_level() >= 2) {
    int16_t v = adc_fifo_get();
    int16_t i = adc_fifo_get();
    currentId++;
    // v = 2000 + 1900 * sin(currentId * (60 * 2 * M_PI/6000));
    // i = 2000 + 1900 * sin(currentId * (60 * 2 * M_PI/6000));
    if (!buffer) {
      BaseType_t response =
          xQueueReceiveFromISR(waveEmptyQueue, &buffer, &wake);
      if (response != pdFALSE) {
        currentBufferIndex = 0;
        buffer->id = currentId;
      } else {
        buffer = 0;
      }
    }
    if (buffer) {
      if (currentBufferIndex < 100) {
        
        buffer->voltage[currentBufferIndex] = v;
        buffer->current[currentBufferIndex] = i;
        
        currentBufferIndex++;
      }
      if (currentBufferIndex >= 100) {
        BaseType_t response = xQueueSendFromISR(waveFullQueue, &buffer, &wake);
        if (response) {
          buffer = 0;
        }
      }
    }
  }
  if (wake) {
    portYIELD_FROM_ISR(NULL);
  }
  irq_clear(ADC_IRQ_FIFO);
}

void setupADC() {
  adc_init();
  adc_gpio_init(26);
  adc_gpio_init(27);
  adc_set_round_robin(3);
  adc_irq_set_enabled(true);

  float adcClock = 48000000.0;           // 48 MHz clock padrão
  float clkdiv = adcClock / SAMPLE_RATE; // 12kHz

  adc_set_clkdiv(clkdiv);
  irq_set_exclusive_handler(ADC_IRQ_FIFO, adcIRQHandler);
  adc_fifo_setup(true, false, 2, false, false);
  adc_run(true);
  irq_set_enabled(ADC_IRQ_FIFO, true);
}
