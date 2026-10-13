#pragma once

#include <stdint.h>

typedef void* osThreadId_t;
typedef int32_t osStatus_t;

#ifdef __cplusplus
extern "C" {
#endif
osStatus_t osThreadSuspend(osThreadId_t thread_id);
osStatus_t osThreadResume(osThreadId_t thread_id);
void osDelay(uint32_t ticks);
#ifdef __cplusplus
}
#endif
