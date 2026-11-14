#pragma once

/* ---- Entropia por hardware (RP2040) ---- */
#define MBEDTLS_NO_PLATFORM_ENTROPY
#define MBEDTLS_ENTROPY_HARDWARE_ALT

/* ---- Tempo em ms (fornecido por mbedtls_ms_time em influx.cpp) ---- */
#define MBEDTLS_HAVE_TIME
#define MBEDTLS_PLATFORM_MS_TIME_ALT

/* ---- DESLIGAR timing.c (incompatível com bare-metal) ---- */
#if defined(MBEDTLS_TIMING_C)
#undef MBEDTLS_TIMING_C
#endif

/* ---- DESLIGAR acesso a filesystem (evita <dirent.h>) ---- */
#if defined(MBEDTLS_FS_IO)
#undef MBEDTLS_FS_IO
#endif

/* ---- DESLIGAR PSA / ITS (evita storage em arquivo e dependências) ---- */
#if defined(MBEDTLS_USE_PSA_CRYPTO)
#undef MBEDTLS_USE_PSA_CRYPTO
#endif
#if defined(MBEDTLS_PSA_CRYPTO_C)
#undef MBEDTLS_PSA_CRYPTO_C
#endif
#if defined(MBEDTLS_PSA_CRYPTO_STORAGE_C)
#undef MBEDTLS_PSA_CRYPTO_STORAGE_C
#endif
#if defined(MBEDTLS_PSA_ITS_FILE_C)
#undef MBEDTLS_PSA_ITS_FILE_C
#endif
#if defined(MBEDTLS_PSA_CRYPTO_DRIVERS)
#undef MBEDTLS_PSA_CRYPTO_DRIVERS
#endif
#if defined(MBEDTLS_PSA_CRYPTO_BUILTIN_KEYS)
#undef MBEDTLS_PSA_CRYPTO_BUILTIN_KEYS
#endif
#if defined(MBEDTLS_PSA_CRYPTO_SE_C)
#undef MBEDTLS_PSA_CRYPTO_SE_C
#endif

/* ---- DESLIGAR TLS 1.3 e extras que puxam PSA ---- */
#if defined(MBEDTLS_SSL_PROTO_TLS1_3)
#undef MBEDTLS_SSL_PROTO_TLS1_3
#endif
#if defined(MBEDTLS_SSL_TLS1_3_KEY_EXCHANGE_MODE_EPHEMERAL_ENABLED)
#undef MBEDTLS_SSL_TLS1_3_KEY_EXCHANGE_MODE_EPHEMERAL_ENABLED
#endif
#if defined(MBEDTLS_SSL_TLS1_3_KEY_EXCHANGE_MODE_PSK_EPHEMERAL_ENABLED)
#undef MBEDTLS_SSL_TLS1_3_KEY_EXCHANGE_MODE_PSK_EPHEMERAL_ENABLED
#endif
#if defined(MBEDTLS_SSL_TLS1_3_KEY_EXCHANGE_MODE_PSK_ENABLED)
#undef MBEDTLS_SSL_TLS1_3_KEY_EXCHANGE_MODE_PSK_ENABLED
#endif
#if defined(MBEDTLS_SSL_EARLY_DATA)
#undef MBEDTLS_SSL_EARLY_DATA
#endif

/* ---- DESLIGAR LMS (requer PSA) ---- */
#if defined(MBEDTLS_LMS_C)
#undef MBEDTLS_LMS_C
#endif

/* ---- DESLIGAR NET (impede build do mbedtls/net_sockets.c) ---- */
#if defined(MBEDTLS_NET_C)
#undef MBEDTLS_NET_C
#endif

/* ---- TLS cliente mínimo (TLS 1.2) ---- */
#define MBEDTLS_SSL_PROTO_TLS1_2
#define MBEDTLS_SSL_CLI_C
#define MBEDTLS_SSL_TLS_C

/* ---- PRNG/entropia ---- */
#define MBEDTLS_CTR_DRBG_C
#define MBEDTLS_ENTROPY_C
#define MBEDTLS_SHA256_C

/* ---- (Opcional) X.509 (mantido, mas sem FS) ---- */
#define MBEDTLS_X509_CRT_PARSE_C

/* ---- Buffers ---- */
#ifndef MBEDTLS_SSL_IN_CONTENT_LEN
#define MBEDTLS_SSL_IN_CONTENT_LEN  4096
#endif
#ifndef MBEDTLS_SSL_OUT_CONTENT_LEN
#define MBEDTLS_SSL_OUT_CONTENT_LEN 4096
#endif
