#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif
uint8_t Remote_IsOnline(void);
uint8_t Remote_IsHoldRequested(void);
uint8_t Remote_IsHostControl(void);
uint8_t Remote_HostControlSnapshot(uint32_t *mode_generation);
uint8_t Remote_ConsumeManualRearmRequest(void);
void Remote_Supervise(void);
#ifdef __cplusplus
}
#endif
