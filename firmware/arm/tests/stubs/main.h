#pragma once
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif
typedef enum { HAL_OK = 0 } HAL_StatusTypeDef;
typedef struct { void* Instance; uint32_t ErrorCode; } UART_HandleTypeDef;
typedef struct { int unused; } DMA_HandleTypeDef;
#define UART5 ((void*)5)
#define DMA_IT_HT 1
#define HAL_UART_ERROR_NONE 0
#define __HAL_DMA_DISABLE_IT(handle, interrupt) ((void)(handle), (void)(interrupt))
#define __HAL_UART_CLEAR_IDLEFLAG(handle) ((void)(handle))
#define __HAL_UART_CLEAR_PEFLAG(handle) ((void)(handle))
#define __HAL_UART_CLEAR_FEFLAG(handle) ((void)(handle))
#define __HAL_UART_CLEAR_NEFLAG(handle) ((void)(handle))
#define __HAL_UART_CLEAR_OREFLAG(handle) ((void)(handle))
static inline uint32_t __get_PRIMASK(void) { return 0; }
static inline void __disable_irq(void) {}
extern void (*test_irq_restore_hook)(void);
static inline void __set_PRIMASK(uint32_t value)
{
    (void)value;
    if (test_irq_restore_hook) {
        void (*hook)(void) = test_irq_restore_hook;
        test_irq_restore_hook = 0;
        hook();
    }
}
uint32_t HAL_GetTick(void);
HAL_StatusTypeDef HAL_UARTEx_ReceiveToIdle_DMA(UART_HandleTypeDef*, uint8_t*, uint16_t);
HAL_StatusTypeDef HAL_UART_AbortReceive(UART_HandleTypeDef*);
#ifdef __cplusplus
}
#endif
