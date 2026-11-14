#pragma once
// 0 = V & I; 1 = V; 2 = I;
#ifndef ADC_TEST_MODE
  #define ADC_TEST_MODE 0
#endif

#ifndef LOG_BUFFERS
  #define LOG_BUFFERS 0
#endif

#if ADC_TEST_MODE == 1
  #define CHANNELS      1
  #define SAMPLE_RATE   24000
  #define SAMPLE_AMOUNT 400
#elif ADC_TEST_MODE == 2
  #define CHANNELS      1
  #define SAMPLE_RATE   24000
  #define SAMPLE_AMOUNT 400
#else
  #define CHANNELS      2
  #define SAMPLE_RATE   24000
  #define SAMPLE_AMOUNT 400  
#endif

// Wi-Fi / Influx
#define WIFI_SSID     "change-me"
#define WIFI_PASSWORD "change-me"
#define INFLUX_PORT 443
#define INFLUX_HOSTNAME "your-influx-host.example.com"
#define INFLUX_PATH "/api/v2/write"

#define INFLUX_MEASUREMENT "pico"
#define INFLUX_ORG "change-me"
#define INFLUX_BUCKET "change-me"
#define INFLUX_TOKEN "change-me"
