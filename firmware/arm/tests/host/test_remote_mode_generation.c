#include "Remote.h"
#include "usart.h"

#include <assert.h>
#include <stdio.h>
#include <string.h>

static uint32_t tick;
static uint8_t* rx_buffer;
USART_TypeDef uart5_instance;

UART_HandleTypeDef huart5 = {&uart5_instance, 0U, 0U};
DMA_HandleTypeDef hdma_uart5_rx = {0U};

uint32_t HAL_GetTick(void) { return tick; }
uint32_t __get_PRIMASK(void) { return 0U; }
void __disable_irq(void) {}
void __set_PRIMASK(uint32_t value) { (void)value; }
HAL_StatusTypeDef HAL_UARTEx_ReceiveToIdle_DMA(UART_HandleTypeDef* uart, uint8_t* buffer, uint16_t size)
{
	(void)size;
	rx_buffer = buffer;
	uart->RxState = HAL_UART_STATE_BUSY_RX;
	uart->Instance->CR3 |= USART_CR3_DMAR;
	hdma_uart5_rx.State = HAL_DMA_STATE_BUSY;
	return HAL_OK;
}
HAL_StatusTypeDef HAL_UART_AbortReceive(UART_HandleTypeDef* uart)
{
	uart->RxState = 0U;
	uart->Instance->CR3 &= ~USART_CR3_DMAR;
	hdma_uart5_rx.State = 0U;
	return HAL_OK;
}
HAL_StatusTypeDef HAL_UART_DeInit(UART_HandleTypeDef* uart)
{
	return HAL_UART_AbortReceive(uart);
}
void MX_UART5_Init(void) {}
void Joint_SetRemoteVelocity(unsigned char idx, float ratio, unsigned char sticks_neutral)
{
	(void)idx; (void)ratio; (void)sticks_neutral;
}
void Gripper_Command(unsigned char close_target) { (void)close_target; }

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef* uart, uint16_t size);

static void SendSelectorFrame(const uint8_t s1, const uint8_t s2)
{
	uint8_t frame[RC_FRAME_LENGTH] = {0U};
	frame[5] = (uint8_t)((s1 << 4U) | (s2 << 6U));
	assert(rx_buffer != NULL);
	memcpy(rx_buffer, frame, sizeof(frame));
	++tick;
	HAL_UARTEx_RxEventCallback(&huart5, sizeof(frame));
}

int main(void)
{
	Remote_Init();
	uint32_t generation = UINT32_MAX;
	assert(Remote_HostControlSnapshot(&generation) == 0U);
	assert(generation == 0U);

	SendSelectorFrame(RC_SW_MID, RC_SW_UP);
	assert(Remote_HostControlSnapshot(&generation) == 1U);
	assert(generation == 1U); /* initial valid selector frame is observable */

	SendSelectorFrame(RC_SW_MID, RC_SW_UP);
	(void)Remote_HostControlSnapshot(&generation);
	assert(generation == 1U); /* unchanged selectors do not create false transitions */

	SendSelectorFrame(RC_SW_MID, RC_SW_DOWN);
	SendSelectorFrame(RC_SW_MID, RC_SW_UP);
	assert(Remote_HostControlSnapshot(&generation) == 1U);
	assert(generation == 3U); /* away-and-back between task polls remains latched */

	SendSelectorFrame(RC_SW_UP, RC_SW_UP);
	(void)Remote_HostControlSnapshot(&generation);
	assert(generation == 4U); /* S1 group selector also invalidates the old authorization */

	puts("PASS production Remote.c selector-generation tests");
	return 0;
}
