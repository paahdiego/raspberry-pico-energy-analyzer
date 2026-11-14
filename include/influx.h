#pragma once
#include "types.h"
#ifdef __cplusplus
extern "C" {
#endif
void sendToInflux(const struct RMSWaveform* data);
#ifdef __cplusplus
}
#endif
