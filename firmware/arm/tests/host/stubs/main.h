#pragma once

#include <stdint.h>

#define UNUSED(x) ((void)(x))

#ifdef __cplusplus
extern "C" {
#endif
uint32_t HAL_GetTick(void);
uint32_t __get_PRIMASK(void);
void __disable_irq(void);
void __set_PRIMASK(uint32_t value);

typedef struct
{
	volatile uint32_t CR3;
} USART_TypeDef;

typedef struct
{
	USART_TypeDef* Instance;
	uint32_t ErrorCode;
	uint32_t RxState;
} UART_HandleTypeDef;

typedef struct
{
	uint32_t State;
} DMA_HandleTypeDef;

typedef enum { HAL_OK = 0, HAL_ERROR = 1 } HAL_StatusTypeDef;
#define HOST_STUB_HAL_STATUS_DEFINED 1

extern USART_TypeDef uart5_instance;
#define UART5 (&uart5_instance)
#define USART_CR3_DMAR (1U << 6U)
#define HAL_UART_STATE_BUSY_RX 1U
#define HAL_DMA_STATE_BUSY 1U
#define HAL_UART_ERROR_NONE 0U
#define DMA_IT_HT 1U
#define __HAL_DMA_DISABLE_IT(handle, interrupt) ((void)(handle), (void)(interrupt))
#define __HAL_UART_CLEAR_IDLEFLAG(handle) ((void)(handle))
#define __HAL_UART_CLEAR_PEFLAG(handle) ((void)(handle))
#define __HAL_UART_CLEAR_FEFLAG(handle) ((void)(handle))
#define __HAL_UART_CLEAR_NEFLAG(handle) ((void)(handle))
#define __HAL_UART_CLEAR_OREFLAG(handle) ((void)(handle))

extern UART_HandleTypeDef huart5;
extern DMA_HandleTypeDef hdma_uart5_rx;
HAL_StatusTypeDef HAL_UARTEx_ReceiveToIdle_DMA(UART_HandleTypeDef*, uint8_t*, uint16_t);
HAL_StatusTypeDef HAL_UART_AbortReceive(UART_HandleTypeDef*);
HAL_StatusTypeDef HAL_UART_DeInit(UART_HandleTypeDef*);
void MX_UART5_Init(void);
#ifdef __cplusplus
}
#endif
