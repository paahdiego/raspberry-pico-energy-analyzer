#pragma once
#include <stdbool.h>
#ifdef __cplusplus
extern "C" {
#endif
extern volatile bool isConnected;   
void connectWifiTask(void*);  
#ifdef __cplusplus
}
#endif
