cmake_minimum_required(VERSION 3.12)

# Pull in SDK (must be before project)
include(pico_sdk_import.cmake)
include(FreeRTOS_Kernel_import.cmake)

project(main C CXX ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

add_subdirectory(SSD1306_OLED_PICO)

add_executable(main
        main.cpp
)

target_compile_definitions(main PRIVATE
        pico_stdlib
        pico_cyw43_arch_lwip_threadsafe_background
)

target_include_directories(main PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
)

target_link_libraries(main
        pico_stdlib
        FreeRTOS-Kernel-Heap4
        pico_cyw43_arch_lwip_sys_freertos
        hardware_adc
        hardware_irq
        hardware_i2c
        pico_ssd1306
)
pico_add_extra_outputs(main)
pico_enable_stdio_usb(main 1)
pico_enable_stdio_uart(main 0)
