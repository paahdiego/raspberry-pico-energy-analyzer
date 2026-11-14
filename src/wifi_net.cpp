#include <cstdio>
#include "wifi_net.h"
#include "config.h"

#include <pico/cyw43_arch.h>
#include <lwip/netif.h>
#include <lwip/ip4_addr.h>
#include <lwip/ip_addr.h>

#include <FreeRTOS.h>
#include <task.h>

void connectWifiTask(void*) {
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
        printf("Failed to connect, retrying in 30s... %d\n", err);
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
    vTaskDelay(pdMS_TO_TICKS(30000));
  }
}
