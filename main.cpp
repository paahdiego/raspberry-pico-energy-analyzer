#define DEBUG_TAG() printf("Line: %d\n", __LINE__)
#include <FreeRTOS.h>
#include <queue.h>
#include <task.h>
#include <cstdio>
#include "hardware/adc.h"
#include "hardware/gpio.h"
#include "pico/stdlib.h"
#include "ssd1306/SSD1306_OLED.hpp"
#include <pico/cyw43_arch.h>
#include <lwip/tcp.h>

#define INFLUX_IP_1 192
#define INFLUX_IP_2 168
#define INFLUX_IP_3 0
#define INFLUX_IP_4 0
#define INFLUX_PORT 8086
#define INFLUX_ORG "changeme"
#define INFLUX_BUCKET "changeme"
#define INFLUX_TOKEN "changeme"

#define SAMPLE_RATE 12000
#define SAMPLE_AMOUNT 100

#define myOLEDwidth 128
#define myOLEDheight 64
#define myScreenSize (myOLEDwidth * (myOLEDheight / 8))
uint8_t screenBuffer[myScreenSize];

const uint16_t I2C_Speed = 100;
const uint8_t I2C_Address = 0x3C;

SSD1306 myOLED(myOLEDwidth, myOLEDheight);

char WIFI_SSID[] = "changeme";
char WIFI_PASSWORD[] = "changeme";

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

typedef struct {
    struct tcp_pcb *pcb;
    bool connected;
    bool complete;
    const RMSWaveform* data;
} influx_client_t;

volatile bool isConnected = false;

static influx_client_t influxState;

static err_t influxOnConnected(void *arg, struct tcp_pcb *tpcb, err_t err) {
    influx_client_t* state = (influx_client_t*)arg;

    if (err != ERR_OK) {
        printf("Connection Error: %d\n", err);
        state->complete = true;
        return err;
    }

    state->connected = true;
    printf("Connected to InfluxDB\n");

    const RMSWaveform* data = state->data;

    static char body[256];
    snprintf(body, sizeof(body),
        "pico2,source=pico_ev_charger id=%lu,voltage=%.2f,current=%.2f,activePower=%.2f,reactivePower=%.2f,power=%.2f,powerFactor=%.2f",
        data->id, data->voltage, data->current, data->activePower,
        data->reactivePower, data->power, data->powerFactor
    );

    static char request[512];
    snprintf(request, sizeof(request),
        "POST /api/v2/write?org=%s&bucket=%s&precision=s HTTP/1.1\r\n"
        "Host: %d.%d.%d.%d\r\n"
        "Authorization: Token %s\r\n"
        "Content-Type: text/plain\r\n"
        "Content-Length: %d\r\n"
        "Connection: close\r\n"
        "\r\n"
        "%s",
        INFLUX_ORG, INFLUX_BUCKET,
        INFLUX_IP_1, INFLUX_IP_2, INFLUX_IP_3, INFLUX_IP_4,
        INFLUX_TOKEN,
        strlen(body),
        body
    );

    tcp_write(tpcb, request, strlen(request), TCP_WRITE_FLAG_COPY);
    return tcp_output(tpcb);
}

static err_t influxOnReceive(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err) {
    influx_client_t* state = (influx_client_t*)arg;

    if (!p) {
        printf("Connection closed by server\n\n");
        state->complete = true;

        if (tpcb) {
            tcp_arg(tpcb, NULL);
            tcp_close(tpcb);
            state->pcb = NULL;
        }

        return ERR_OK;
    }

    char *response = (char*)malloc(p->tot_len + 1);
    pbuf_copy_partial(p, response, p->tot_len, 0);
    response[p->tot_len] = '\0';
    printf("InfluxDB response:\n%s\n", response);
    free(response);

    tcp_recved(tpcb, p->tot_len);
    pbuf_free(p);

    return ERR_OK;
}

static void influxOnError(void *arg, err_t err) {
    printf("TCP Error: %d\n", err);
    influx_client_t* state = (influx_client_t*)arg;

    if (state->pcb) {
        tcp_abort(state->pcb);
        state->pcb = NULL;
    }

    state->complete = true;
}

void sendToInflux(const RMSWaveform* data) {
    if (!isConnected || influxState.pcb != NULL) return;

    influxState = {0};
    influxState.pcb = tcp_new();
    influxState.data = data;

    if (!influxState.pcb) {
        printf("Failed to create PCB\n");
        return;
    }

    tcp_arg(influxState.pcb, &influxState);
    tcp_err(influxState.pcb, influxOnError);
    tcp_recv(influxState.pcb, influxOnReceive);

    ip_addr_t influx_ip;
    IP4_ADDR(&influx_ip, INFLUX_IP_1, INFLUX_IP_2, INFLUX_IP_3, INFLUX_IP_4);

    err_t err = tcp_connect(influxState.pcb, &influx_ip, INFLUX_PORT, influxOnConnected);
    if (err != ERR_OK) {
        printf("Failed to connect: %d\n", err);
        tcp_abort(influxState.pcb);
        influxState.pcb = NULL;
    }
}

void setupDisplay(void);
void printDisplayData(const RMSWaveform *rmsWaveform);
float calculateRMS(const int16_t *samples, int sampleAmount, float gain);
void turnOffDisplay();
void adcIRQHandler();
void setupADC();
static void calculateStuff(CalculateStuffArgs *);
static void displayStuff(DisplayStuffArgs *);
static void connectWifiTask(void *);

volatile EanWaveform *buffer = 0;
volatile int currentId = 0;
volatile int currentBufferIndex = 0;

QueueHandle_t waveFullQueue, waveEmptyQueue;
EanWaveform eanWaveformBuffers[6];

QueueHandle_t rmsFullQueue, rmsEmptyQueue;
RMSWaveform rmsWaveformBuffers[6];

int main() {
  stdio_init_all();
  gpio_init(2);
  gpio_init(5);
  gpio_set_dir(5, GPIO_OUT);
  gpio_set_dir(2, GPIO_OUT);
  gpio_put(2, 1);

  for (int i = 0; i < 6; ++i) {
    printf("Booting in %d...\n", 6 - i);
    sleep_ms(1000);
  }

  printf("WIFI_SSID: %s\n", WIFI_SSID);
  printf("WIFI_PASSWORD: %s\n\n", WIFI_PASSWORD);

  setupDisplay();
  busy_wait_ms(1000);
  setupADC();

  gpio_put(2, 0);
  gpio_put(5, 1);

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

  float vGain = 1 / ((1 / 120000.0) * 50 * (47 / 4.7) * (4096 / 3.3));
  float iGain = 1 / (1. / 30. * 4096 / 3.3);

  CalculateStuffArgs calculateArgs{waveFullQueue, waveEmptyQueue, rmsFullQueue, rmsEmptyQueue, iGain, vGain};

  DisplayStuffArgs displayArgs{rmsFullQueue, rmsEmptyQueue};

  printf("Creating tasks...\n\n");

  xTaskCreate(connectWifiTask, "wifiTask", 4096, NULL, 10, NULL);
  xTaskCreate((TaskFunction_t)calculateStuff, "calculateStuff", 2048, &calculateArgs, 8, NULL);
  xTaskCreate((TaskFunction_t)displayStuff, "displayStuff", 4096, &displayArgs, 3, NULL);

  printf("Starting FreeRTOS-SMP scheduler...\n\n");
  vTaskStartScheduler();

  turnOffDisplay();
}

void setupDisplay(void) {
  busy_wait_ms(500);
  printf("Starting display setup...\n");
  while (myOLED.OLEDbegin(I2C_Address, i2c1, I2C_Speed, 18, 19) != true) {
    printf("Connecting display...\n");
    busy_wait_ms(1500);
  }
  if (myOLED.OLEDSetBufferPtr(myOLEDwidth, myOLEDheight, screenBuffer, sizeof(screenBuffer) / sizeof(uint8_t)) != 0) {
    while (1) {
      busy_wait_ms(1000);
    }
  }
  myOLED.OLEDFillScreen(0xF0, 0);
  printf("Finished display setup...\n\n");
  busy_wait_ms(1000);
}

void printDisplayData(const RMSWaveform *rmsWaveform) {
  printf("V(RMS): %f\n", rmsWaveform->voltage);
  printf("I(RMS): %f\n\n", rmsWaveform->current);
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
    if (!buffer) {
      BaseType_t response = xQueueReceiveFromISR(waveEmptyQueue, &buffer, &wake);
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
  printf("Setuping ADC... \n");
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
  printf("Finished ADC setup...\n\n");
}

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
  while (true) {
    RMSWaveform *rmsBuffer = 0;

    while (!xQueueReceive(args->rmsFull, &rmsBuffer, portMAX_DELAY));
    printDisplayData(rmsBuffer);
    sendToInflux(rmsBuffer);
    vTaskDelay(1000);
    while (!xQueueSend(args->rmsEmpty, &rmsBuffer, portMAX_DELAY));
  }
}

static void connectWifiTask(void *) {
  if (cyw43_arch_init() != 0) {
    printf("WiFi init failed\n");
    vTaskDelete(NULL);
  }

  while (true) {
    printf("Checking Wifi...\n\n");
    if (!cyw43_wifi_link_status(&cyw43_state, CYW43_ITF_STA)) {
      cyw43_arch_enable_sta_mode();
      printf("Connecting to WiFi...\n");
      int err = cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PASSWORD, CYW43_AUTH_WPA2_AES_PSK, 30000);
      
      if (err) {
        printf("Failed to connect, retrying in 60s... %d\n", err);
        isConnected=false;
      } else {
        printf("Connected to WiFi!\n");
        vTaskPrioritySet(NULL, 2);
    
        int i=0;
        ip4_addr_t ip;
        while (true) {
          ip = *netif_ip4_addr(netif_default);  // pega o IP atual

          printf("%d: %d.%d.%d.%d\n",
                i++,
                ip4_addr1(&ip),
                ip4_addr2(&ip),
                ip4_addr3(&ip),
                ip4_addr4(&ip));

          if (!ip4_addr_isany_val(ip)) {
            printf("IP válido: %s\n", ip4addr_ntoa(&ip));
            isConnected = true;
            break;
          }

          vTaskDelay(pdMS_TO_TICKS(1000));
        }
      }
    }
    vTaskDelay(pdMS_TO_TICKS(60000));
  }
}
