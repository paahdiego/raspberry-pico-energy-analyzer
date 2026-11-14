cmake_minimum_required(VERSION 3.13)

# Pull in SDK (must be before project)
include(pico_sdk_import.cmake)
include(FreeRTOS_Kernel_import.cmake)
set(PICOTOOL_SKIP_BUILD 1)

project(main C CXX ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

# --- mbedTLS: desabilitar samples e tests (evita syscalls/PSA) ---
# Mantém compatível com várias versões do CMake da mbedTLS
set(ENABLE_PROGRAMS OFF CACHE BOOL "" FORCE)
set(ENABLE_TESTING  OFF CACHE BOOL "" FORCE)
set(MBEDTLS_BUILD_PROGRAMS OFF CACHE BOOL "" FORCE)
set(MBEDTLS_BUILD_TESTING  OFF CACHE BOOL "" FORCE)
set(MBEDTLS_FATAL_WARNINGS OFF CACHE BOOL "" FORCE)

add_subdirectory(SSD1306_OLED_PICO)

# EXCLUDE_FROM_ALL ajuda a não puxar alvos "extras" por acidente
add_subdirectory(${PICO_SDK_PATH}/lib/mbedtls mbedtls_build EXCLUDE_FROM_ALL)

add_executable(main
    src/main.cpp
    src/adc.cpp
    src/processing.cpp
    src/display.cpp
    src/wifi_net.cpp
    src/influx.cpp
    src/globals.cpp
)

target_compile_definitions(main PRIVATE
    pico_stdlib
    pico_cyw43_arch_lwip_threadsafe_background
)

# --- mbedTLS user config: tornar visível para as libs mbed* ---
target_include_directories(mbedcrypto PRIVATE ${CMAKE_CURRENT_LIST_DIR}/include)
target_include_directories(mbedx509   PRIVATE ${CMAKE_CURRENT_LIST_DIR}/include)
target_include_directories(mbedtls    PRIVATE ${CMAKE_CURRENT_LIST_DIR}/include)

# Só define o arquivo de user config (as flags estão dentro dele)
target_compile_definitions(mbedcrypto PRIVATE MBEDTLS_USER_CONFIG_FILE="mbedtls_user_config.h")
target_compile_definitions(mbedx509   PRIVATE MBEDTLS_USER_CONFIG_FILE="mbedtls_user_config.h")
target_compile_definitions(mbedtls    PRIVATE MBEDTLS_USER_CONFIG_FILE="mbedtls_user_config.h")

# --- includes do app ---
target_include_directories(main PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/include
    ${CMAKE_CURRENT_LIST_DIR}
    ${PICO_SDK_PATH}/lib/mbedtls/include
)

# --- links ---
target_link_libraries(main
    pico_stdlib
    FreeRTOS-Kernel-Heap4
    pico_cyw43_arch_lwip_sys_freertos
    pico_lwip_mbedtls
    pico_rand            # usado por mbedtls_hardware_poll()
    hardware_adc
    hardware_irq
    hardware_i2c
    hardware_gpio
    pico_ssd1306
    mbedtls mbedx509 mbedcrypto
)

pico_enable_stdio_usb(main 1)
pico_enable_stdio_uart(main 0)
pico_add_extra_outputs(main)
