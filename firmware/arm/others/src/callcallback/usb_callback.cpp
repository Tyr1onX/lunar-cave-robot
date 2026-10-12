#include "Host.h"

#include "ring_buffer.hpp"

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os2.h"

#ifdef __cplusplus
}
#endif

/* 定义在 variables.cpp（全工程变量集中处，避免初始化顺序问题） */
extern Ring_Queue<uint8_t, HOST_RX_QUEUE_SIZE> host_usb_queue;
extern volatile uint32_t host_link_tick;  /* 最近一次收到 USB 数据包的 HAL tick */
extern volatile uint32_t host_rx_drop;    /* 队列满导致的丢弃次数 */
extern osThreadId_t hostTaskHandle;       /* 定义在 Core/Src/freertos.c */

/* 只在 CDC_Receive_HS 里调用（USB 中断上下文）。
   在重新挂接接收之前把数据搬进环形队列：中断里只做搬运和唤醒，解析全部交给 Host_Task。 */
extern "C" void Host_UsbReceive(const uint8_t* data, uint32_t length)
{
	if (data == nullptr || length == 0U) { return; }

	/* 链路活跃度按"收到任意 USB 数据包"计，与帧是否合法无关 */
	host_link_tick = HAL_GetTick();

	const uint16_t count = (length > 0xFFFFU) ? 0xFFFFU : static_cast<uint16_t>(length);
	if (!host_usb_queue.PushBatch(data, count))
	{
		++host_rx_drop;  /* 队列满：整包丢弃并计数，不做半截写入 */
		return;
	}
	if (hostTaskHandle != nullptr)
	{
		(void)osThreadFlagsSet(hostTaskHandle, HOST_RX_FLAG);
	}
}
