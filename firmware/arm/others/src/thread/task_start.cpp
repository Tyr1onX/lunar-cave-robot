#include "threads.hpp"
#include "bsp_can.hpp"
#include "bsp_tim.hpp"
#include "usb_device.h"


#ifdef __cplusplus
extern "C" {
#endif

extern osThreadId_t defaultTaskHandle;

#ifdef __cplusplus
}
#endif


extern Timebase_Timer micro_base;

extern Bsp_CAN abc_can3;

/* implements the CubeMX "defaultTask" thread (weak stub in freertos.c) */

/* ============================ 任务初始化总表 ============================
   所有任务的一次性前置初始化都集中在启动任务里，按“谁依赖”分组列出；
   各任务体（roboARM_task / Host_Task / Error_tasks）里不再做初始化，
   要确认“某个任务依赖什么前置条件”，只看本函数即可。

   启动任务由 CubeMX 创建（osPriorityNormal，栈 256*4），优先级高于其它任务，
   因此它的初始化一定在其它任务开始跑之前完成；做完后自杀，不再占用调度。 */
void StartDefaultTask(void *argument)
{
	UNUSED(argument);

	/* --- 全局微秒时基（roboARM_task 的 2ms 截止时间、CAN 分组发送间隔都依赖它） --- */
	micro_base.Init();

	/* --- FDCAN3：7 个关节电机的反馈接收与 MIT 命令发送 --- */
	abc_can3.FilterConfig();
	abc_can3.Start();

	/* --- USB 设备（OTG_HS 全速 CDC）：上位机链路，Host_Task 只负责等标志与解析 --- */
	MX_USB_DEVICE_Init();

	osThreadTerminate(defaultTaskHandle);  // obtained by osThreadNew or osThreadGetId
}
