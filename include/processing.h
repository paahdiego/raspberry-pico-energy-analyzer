#pragma once
#include "types.h"
#ifdef __cplusplus
extern "C" {
#endif
float calculateRMS(const int16_t *samples, int sampleAmount, float gain);
void calculateStuff(struct CalculateStuffArgs* args);
#ifdef __cplusplus
}
#endif
