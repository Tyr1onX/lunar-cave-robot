#include "Host.h"
#include "main.h"          /* UNUSED */
#include "threads.hpp"     /* Host_Task 原型（与 CubeMX freertos.c 对齐）+ cmsis_os2.h */


/* ============================ 上位机 USB-CDC 任务 ============================
   USB 链路的三个文件各管一段，互不越界：
     · callcallback/usb_callback.cpp —— USB 中断钩子：只搬字节进环形队列 + 置 HOST_RX_FLAG
     · app/Host.cpp                  —— 协议：帧同步、校验、拆包、执行目标、组包、发送
     · 本文件                        —— 任务体：等待标志 + 调 Host_Poll()
   数据流：USB 中断 → host_usb_queue + HOST_RX_FLAG → 本任务 → Host_Poll() → 回状态包

   前置条件：USB 设备由启动任务初始化（task_start.cpp 的 TaskInit 表里 MX_USB_DEVICE_Init），
   本任务只做收发调度、不做任何初始化；因此它被删掉不会导致 USB 不枚举。 */
void Host_Task(void *argument)
{
	UNUSED(argument);

	for (;;)
	{
		const uint32_t flags = osThreadFlagsWait(HOST_RX_FLAG, osFlagsWaitAny, osWaitForever);
		if ((flags & HOST_RX_FLAG) == 0U) { continue; }

		/* 一次唤醒处理完队列里当前所有完整帧；没有完整帧时 Host_Poll 立即返回 */
		Host_Poll();
	}
}
