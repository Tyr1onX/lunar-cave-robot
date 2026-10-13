#pragma once

#define UNUSED(x) ((void)(x))

#ifdef __cplusplus
extern "C" {
#endif
void HostTestEnterCritical(void);
void HostTestExitCritical(void);
#ifdef __cplusplus
}
#endif

#define taskENTER_CRITICAL() HostTestEnterCritical()
#define taskEXIT_CRITICAL() HostTestExitCritical()
