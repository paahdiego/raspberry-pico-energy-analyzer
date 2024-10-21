# raspberry_pico_freertos

This repository contains a clean starting point for using Raspberry Pi Pico W with FreeRTOS.

# Setting up
Start by installing [pico-sdk](https://github.com/raspberrypi/pico-sdk) and it's dependencies:

```sh
sudo apt install -y cmake git \
                    gcc-arm-none-eabi \
                    libnewlib-arm-none-eabi \
                    libstdc++-arm-none-eabi-newlib 
sudo mkdir /opt/pico
cd /opt/pico
sudo git clone --depth 1 https://github.com/FreeRTOS/FreeRTOS-Kernel
sudo git clone --depth 1 https://github.com/raspberrypi/pico-sdk
cd /opt/pico/pico-sdk
sudo git submodule update --init --depth 1
```

For convenience, create a file at `/opt/pico/sourceme` which sets the required environment variables. If you choose not to create this file, then you will need to set the variables some other way.

```sh
export PICO_BOARD=pico_w
export PICO_SDK_PATH=/opt/pico/pico-sdk
export FREERTOS_KERNEL_PATH=/opt/pico/FreeRTOS-Kernel
export WIFI_SSID=your_wifi_name
export WIFI_PASSWORD=public_secret
```

Then, for building you should:

```sh
source /opt/pico/sourceme
cd /tmp
git clone https://gitlab.com/lucas.hartmann/raspberry_pico_freertos
mkdir raspberry_pico_freertos/build
cd raspberry_pico_freertos/build
cmake ..
make
```
