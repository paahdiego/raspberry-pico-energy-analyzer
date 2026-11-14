#pragma once
#include "types.h"
#ifdef __cplusplus
extern "C" {
#endif

#define myOLEDwidth 128
#define myOLEDheight 64
#define myScreenSize (myOLEDwidth * (myOLEDheight / 8))

void setupDisplay(uint32_t);
void turnOffDisplay(void);
void printDisplayData(const struct RMSWaveform* r);
void displayStuff(struct DisplayStuffArgs* args);
#ifdef __cplusplus
}
#endif
