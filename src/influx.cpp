#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <inttypes.h>
#include <fcntl.h>

#include "influx.h"
#include "config.h"
#include "wifi_net.h"

#include "pico/rand.h"
#include "pico/time.h"

#include <lwip/sockets.h>
#include <lwip/netdb.h>
#include <lwip/errno.h>
#include <lwip/inet.h>

#include "FreeRTOS.h"
#include "task.h"

#ifdef poll
#undef poll
#endif

/* mbedTLS */
#include "mbedtls/ssl.h"
#include "mbedtls/ctr_drbg.h"
#include "mbedtls/entropy.h"
#include "mbedtls/error.h"
#include "mbedtls/net_sockets.h"
#include "mbedtls/platform_time.h"

#ifndef INFLUX_PORT
#define INFLUX_PORT 443
#endif

// ===== RNG para mbedTLS (usa pico/rand) =====
extern "C" int mbedtls_hardware_poll(void*, unsigned char* out, size_t len, size_t* olen) {
  if (!out || !olen) return MBEDTLS_ERR_ENTROPY_SOURCE_FAILED;
  size_t i = 0;
  while (i + 4 <= len) {
    uint32_t r = get_rand_32();
    memcpy(out + i, &r, 4);
    i += 4;
  }
  if (i < len) {
    uint32_t r = get_rand_32();
    memcpy(out + i, &r, len - i);
  }
  *olen = len;
  return 0;
}

/* Tempo em ms (requer MBEDTLS_PLATFORM_MS_TIME_ALT) */
extern "C" mbedtls_ms_time_t mbedtls_ms_time(void) {
  return (mbedtls_ms_time_t) to_ms_since_boot(get_absolute_time());
}

static inline uint64_t now_ms()  { return to_ms_since_boot(get_absolute_time()); }
static inline bool timed_out(uint64_t start, uint32_t ms) { return (now_ms() - start) > ms; }

/* ----------------------------------------------------------------------------
   BIO -> lwIP (não-bloqueante, sempre cede CPU)
---------------------------------------------------------------------------- */
static int tls_send(void *ctx, const unsigned char *buf, size_t len) {
  if (len == 0) return MBEDTLS_ERR_SSL_WANT_WRITE;
  int s = *(int*)ctx;

  int r = lwip_send(s, buf, (int)len, 0);
  int e = errno;

  if (r < 0) {
    if (e == EWOULDBLOCK || e == EAGAIN || e == EINTR) {
      vTaskDelay(pdMS_TO_TICKS(1));
      return MBEDTLS_ERR_SSL_WANT_WRITE;
    }
    printf("lwip_send errno=%d\n", e);
    return MBEDTLS_ERR_NET_SEND_FAILED;
  }
  if (r == 0) {
    vTaskDelay(pdMS_TO_TICKS(1));
    return MBEDTLS_ERR_SSL_WANT_WRITE;
  }
  return r;
}

static int tls_recv(void *ctx, unsigned char *buf, size_t len) {
  if (len == 0) return MBEDTLS_ERR_SSL_WANT_READ;
  int s = *(int*)ctx;

  int r = lwip_recv(s, buf, (int)len, 0);
  int e = errno;

  if (r < 0) {
    if (e == EWOULDBLOCK || e == EAGAIN || e == EINTR) {
      vTaskDelay(pdMS_TO_TICKS(1));
      return MBEDTLS_ERR_SSL_WANT_READ;
    }
    printf("lwip_recv errno=%d\n", e);
    return MBEDTLS_ERR_NET_RECV_FAILED;
  }
  return r; // r==0 => EOF ok
}

/* Conecta (não-bloqueante) com timeout usando select() fatiado + yield */
static bool connect_with_timeout(int sock, const struct sockaddr* sa, socklen_t salen, uint32_t timeout_ms) {
  int ret = lwip_connect(sock, sa, salen);
  int e = errno;

  if (ret == 0) return true;
  if (ret < 0 && e != EINPROGRESS) {
    printf("connect() fail errno=%d\n", e);
    return false;
  }

  uint64_t start = now_ms();
  uint64_t last_log = 0;
  while (!timed_out(start, timeout_ms)) {
    fd_set wfds;
    FD_ZERO(&wfds);
    FD_SET(sock, &wfds);
    struct timeval tv; tv.tv_sec = 0; tv.tv_usec = 200 * 1000; // 200ms

    int sret = lwip_select(sock + 1, NULL, &wfds, NULL, &tv);
    if (sret < 0) {
      printf("select(connect) ret=%d\n", sret);
      return false;
    }
    if (sret > 0) {
      int soerr = 0; socklen_t slen = sizeof(soerr);
      lwip_getsockopt(sock, SOL_SOCKET, SO_ERROR, &soerr, &slen);
      if (soerr == 0) return true;
      printf("SO_ERROR after connect: %d\n", soerr);
      return false;
    }

    // log a cada ~1s para sabermos que não travou
    uint64_t elapsed = now_ms() - start;
    if (elapsed - last_log >= 1000) {
      printf("[HTTPS] waiting connect... %llums\n", (unsigned long long)elapsed);
      last_log = elapsed;
    }
    vTaskDelay(pdMS_TO_TICKS(1));
  }
  printf("connect timeout\n");
  return false;
}

static int create_tcp_socket_retry(int family, int socktype, int proto, int attempts) {
  while (attempts-- > 0) {
    int s = lwip_socket(family, socktype, proto);
    int e = errno;
    if (s >= 0) return s;

    printf("[HTTPS] socket() fail errno=%d (attempts left=%d)\n", e, attempts);
    vTaskDelay(pdMS_TO_TICKS(10));
  }
  return -1;
}

static bool https_post(const char* host, uint16_t port, const char* req, size_t req_len) {
  printf("[HTTPS] Resolving %s...\n", host);

  // 1) DNS
  struct addrinfo hints = {};
  hints.ai_socktype = SOCK_STREAM;
  hints.ai_family   = AF_INET;
  char portstr[8]; snprintf(portstr, sizeof(portstr), "%u", port);

  struct addrinfo* res = nullptr;
  int gairet = lwip_getaddrinfo(host, portstr, &hints, &res);
  if (gairet != 0 || !res) {
    printf("[HTTPS] DNS fail: %s (ret=%d)\n", host, gairet);
    if (res) lwip_freeaddrinfo(res);
    return false;
  }

  // Log do IP resolvido
  const struct sockaddr_in* sin = (const struct sockaddr_in*)res->ai_addr;
  uint32_t ip = ntohl(sin->sin_addr.s_addr);
  printf("[HTTPS] DNS OK -> %u.%u.%u.%u:%u\n",
         (unsigned)((ip >> 24) & 0xFF), (unsigned)((ip >> 16) & 0xFF),
         (unsigned)((ip >>  8) & 0xFF), (unsigned)( ip        & 0xFF), (unsigned)port);

  // 2) Socket TCP (não-bloqueante) + retries
  int sock = create_tcp_socket_retry(res->ai_family, res->ai_socktype, 0 /*IPPROTO_IP*/, 3);
  if (sock < 0) { lwip_freeaddrinfo(res); return false; }

  // não-bloqueante
  int flags = lwip_fcntl(sock, F_GETFL, 0);
  lwip_fcntl(sock, F_SETFL, flags | O_NONBLOCK);

  // timeouts “suaves”
  struct timeval tv; tv.tv_sec = 10; tv.tv_usec = 0;
  lwip_setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
  lwip_setsockopt(sock, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));

  printf("[HTTPS] Connecting...\n");
  bool ok_conn = connect_with_timeout(sock, res->ai_addr, res->ai_addrlen, 10000);
  lwip_freeaddrinfo(res);
  if (!ok_conn) {
    lwip_close(sock);
    return false;
  }

  // 3) mbedTLS
  mbedtls_entropy_context entropy;   mbedtls_entropy_init(&entropy);
  mbedtls_ctr_drbg_context ctr;      mbedtls_ctr_drbg_init(&ctr);
  const char* pers = "pico-https";

  if (mbedtls_ctr_drbg_seed(&ctr, mbedtls_entropy_func, &entropy,
                            (const unsigned char*)pers, strlen(pers)) != 0) {
    printf("[HTTPS] ctr_drbg_seed fail\n");
    lwip_close(sock);
    mbedtls_entropy_free(&entropy);
    return false;
  }

  mbedtls_ssl_config conf; mbedtls_ssl_config_init(&conf);
  if (mbedtls_ssl_config_defaults(&conf,
        MBEDTLS_SSL_IS_CLIENT, MBEDTLS_SSL_TRANSPORT_STREAM, MBEDTLS_SSL_PRESET_DEFAULT) != 0) {
    printf("[HTTPS] ssl_config_defaults fail\n");
    lwip_close(sock);
    mbedtls_ctr_drbg_free(&ctr); mbedtls_entropy_free(&entropy);
    return false;
  }

  mbedtls_ssl_conf_authmode(&conf, MBEDTLS_SSL_VERIFY_NONE);
  mbedtls_ssl_conf_rng(&conf, mbedtls_ctr_drbg_random, &ctr);

  mbedtls_ssl_context ssl; mbedtls_ssl_init(&ssl);
  if (mbedtls_ssl_setup(&ssl, &conf) != 0) {
    printf("[HTTPS] ssl_setup fail\n");
    lwip_close(sock);
    mbedtls_ssl_config_free(&conf);
    mbedtls_ctr_drbg_free(&ctr); mbedtls_entropy_free(&entropy);
    return false;
  }

  mbedtls_ssl_set_hostname(&ssl, host);
  mbedtls_ssl_set_bio(&ssl, &sock, tls_send, tls_recv, nullptr);

  printf("[HTTPS] Handshake...\n");
  uint64_t hs_start = now_ms();
  for (;;) {
    int ret = mbedtls_ssl_handshake(&ssl);
    if (ret == 0) break;
    if (ret == MBEDTLS_ERR_SSL_WANT_READ || ret == MBEDTLS_ERR_SSL_WANT_WRITE) {
      if (timed_out(hs_start, 15000)) {
        printf("[HTTPS] handshake timeout\n");
        mbedtls_ssl_free(&ssl); mbedtls_ssl_config_free(&conf);
        mbedtls_ctr_drbg_free(&ctr); mbedtls_entropy_free(&entropy);
        lwip_close(sock);
        return false;
      }
      vTaskDelay(pdMS_TO_TICKS(1));
      continue;
    }
    printf("[HTTPS] handshake fail: -0x%04X\n", -ret);
    mbedtls_ssl_free(&ssl); mbedtls_ssl_config_free(&conf);
    mbedtls_ctr_drbg_free(&ctr); mbedtls_entropy_free(&entropy);
    lwip_close(sock);
    return false;
  }

  // 4) Envia request
  printf("[HTTPS] Sending request (%u bytes)...\n", (unsigned)req_len);
  size_t off = 0;
  uint64_t wr_deadline = now_ms();
  while (off < req_len) {
    int ret = mbedtls_ssl_write(&ssl, (const unsigned char*)req + off, req_len - off);
    if (ret > 0) { off += (size_t)ret; wr_deadline = now_ms(); continue; }
    if (ret == MBEDTLS_ERR_SSL_WANT_READ || ret == MBEDTLS_ERR_SSL_WANT_WRITE) {
      if (timed_out(wr_deadline, 10000)) {
        printf("[HTTPS] write timeout\n");
        break;
      }
      vTaskDelay(pdMS_TO_TICKS(1));
      continue;
    }
    printf("[HTTPS] ssl_write fail: -0x%04X\n", -ret);
    break;
  }
  if (off != req_len) {
    int r; do { r = mbedtls_ssl_close_notify(&ssl); } while (r == MBEDTLS_ERR_SSL_WANT_WRITE);
    mbedtls_ssl_free(&ssl); mbedtls_ssl_config_free(&conf);
    mbedtls_ctr_drbg_free(&ctr); mbedtls_entropy_free(&entropy);
    lwip_close(sock);
    return false;
  }

  // 5) Lê resposta com deadline (cede CPU entre tentativas)
  printf("[HTTPS] Reading response...\n");
  char buf[1024];
  uint64_t last_rx = now_ms();
  size_t total = 0;
  for (;;) {
    int ret = mbedtls_ssl_read(&ssl, (unsigned char*)buf, sizeof(buf) - 1);
    if (ret > 0) {
      buf[ret] = 0;
      printf("%s", buf);
      total += (size_t)ret;
      last_rx = now_ms();
      continue;
    }
    if (ret == 0) {
      printf("\n[HTTPS] server closed\n");
      break;
    }
    if (ret == MBEDTLS_ERR_SSL_WANT_READ || ret == MBEDTLS_ERR_SSL_WANT_WRITE) {
      if (timed_out(last_rx, 3000)) {
        printf("\n[HTTPS] read idle timeout (total=%u)\n", (unsigned)total);
        break;
      }
      vTaskDelay(pdMS_TO_TICKS(1));
      continue;
    }
    printf("\n[HTTPS] ssl_read end: -0x%04X\n", -ret);
    break;
  }

  int r; do { r = mbedtls_ssl_close_notify(&ssl); } while (r == MBEDTLS_ERR_SSL_WANT_WRITE);
  mbedtls_ssl_free(&ssl); mbedtls_ssl_config_free(&conf);
  mbedtls_ctr_drbg_free(&ctr); mbedtls_entropy_free(&entropy);

  lwip_close(sock);
  return true;
}

void sendToInflux(const RMSWaveform* data) {
  if (!isConnected) return;

  static char body[256];
  snprintf(body, sizeof(body),
    "%s,source=pico_ev_charger id=%" PRIu32 "i,voltage=%.2f,current=%.2f,activePower=%.2f,reactivePower=%.2f,power=%.2f,powerFactor=%.2f",
    INFLUX_MEASUREMENT, data->id, data->voltage, data->current, data->activePower,
    data->reactivePower, data->power, data->powerFactor);

  static char path_with_qs[256];
  snprintf(path_with_qs, sizeof(path_with_qs),
           "%s?org=%s&bucket=%s&precision=s", INFLUX_PATH, INFLUX_ORG, INFLUX_BUCKET);

  static char req[640];
  int n = snprintf(req, sizeof(req),
    "POST %s HTTP/1.1\r\n"
    "Host: %s\r\n"
    "Authorization: Token %s\r\n"
    "Content-Type: text/plain\r\n"
    "Content-Length: %d\r\n"
    "Connection: close\r\n"
    "\r\n"
    "%s",
    path_with_qs, INFLUX_HOSTNAME, INFLUX_TOKEN, (int)strlen(body), body);

  printf("Connected to InfluxDB over HTTPS\n");
  bool ok = https_post(INFLUX_HOSTNAME, INFLUX_PORT, req, (size_t)n);
  if (!ok) printf("HTTPS post failed\n");
}
