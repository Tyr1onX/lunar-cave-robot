#pragma once

#include <stdint.h>

typedef enum { HAL_OK = 0, HAL_ERROR = 1 } HAL_StatusTypeDef;
typedef struct { void *hdmarx; } UART_HandleTypeDef;
typedef struct { uint32_t unused; } DMA_HandleTypeDef;

extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef test_dma;

uint32_t HAL_GetTick(void);
HAL_StatusTypeDef HAL_UARTEx_ReceiveToIdle_DMA(UART_HandleTypeDef *uart,
                                                uint8_t *buffer,
                                                uint16_t size);
HAL_StatusTypeDef HAL_UART_AbortReceive(UART_HandleTypeDef *uart);

#define DMA_IT_HT 0x01u
#define __HAL_DMA_DISABLE_IT(handle, interrupt) ((void)(handle), (void)(interrupt))
