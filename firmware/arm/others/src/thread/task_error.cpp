#include "threads.hpp"
#include "roboarm.hpp"
#include "Remote.h"

#ifdef __cplusplus
extern "C" {
#endif


extern osThreadId_t roboARMHandle;


#ifdef __cplusplus
}
#endif


extern Joint j1_left;
extern Joint j2_left;
extern Joint j3_left;
extern Joint j4_left;
extern Joint j5_left;
extern Joint j6_left;
extern Joint j7_left;

/* 挂起：先挂住执行任务（不再发命令），再失能全部电机。
   遥控接收由 UART 中断完成，Error_tasks 自身不能挂起，否则无法检测恢复。 */
static void ArmHold()
{
	using namespace Motor_DM_N;
	osThreadSuspend(roboARMHandle);
	Joint_CaptureHoldAll(); // 失能前保存，不能等到恢复时再读取被外力移动后的位置
	j1_left.Ctrl(order_byte_e::disable);
	j2_left.Ctrl(order_byte_e::disable);
	j3_left.Ctrl(order_byte_e::disable);
	j4_left.Ctrl(order_byte_e::disable);
	osDelay(1);
	// 分两组发（4 + 3）：FDCAN 的 TX FIFO 很小，一次连发 7 帧会被静默丢掉
	j5_left.Ctrl(order_byte_e::disable);
	j6_left.Ctrl(order_byte_e::disable);
	j7_left.Ctrl(order_byte_e::disable);
	osDelay(1);
}

/* 挂起期间的反馈保活（Error_tasks 每轮调一次，10ms）。

   达妙电机是“收一帧回一帧”：执行任务一挂起就没人再发命令帧，电机也跟着停发反馈帧，
   于是 pos/vel/torque 以及上层 angle.theta 全部冻结 —— 上位机在急停下看到的角度就停在
   挂起那一刻，无法用来判断“没使能时手动掰到哪了”。
   这里给每个电机补发一帧“失能”指令：失能帧不产生任何力矩，只换取电机回一帧状态，
   让角度继续刷新。必须发与当前状态一致的失能帧，不能发 MIT 控制帧。 */
static void ArmFeedbackKeepAlive()
{
	using namespace Motor_DM_N;
	j1_left.Ctrl(order_byte_e::disable);
	j2_left.Ctrl(order_byte_e::disable);
	j3_left.Ctrl(order_byte_e::disable);
	j4_left.Ctrl(order_byte_e::disable);
	osDelay(1);
	// 同样分两组发，理由见 ArmHold
	j5_left.Ctrl(order_byte_e::disable);
	j6_left.Ctrl(order_byte_e::disable);
	j7_left.Ctrl(order_byte_e::disable);
	osDelay(1);
}

/* 解挂起：清错 -> 全部关节缓启动 -> 使能 -> 恢复执行任务。
   顺序很重要：缓启动必须在使能之前、且在执行任务挂起期间做，
   这样恢复后的第一帧控制命令就是从当前实测位置起算的，不会因挂起期间的位置漂移而猛冲。 */
static void ArmResume()
{
	using namespace Motor_DM_N;
	j1_left.Ctrl(order_byte_e::clear_error);
	j2_left.Ctrl(order_byte_e::clear_error);
	j3_left.Ctrl(order_byte_e::clear_error);
	j4_left.Ctrl(order_byte_e::clear_error);
	osDelay(1);
	j5_left.Ctrl(order_byte_e::clear_error);
	j6_left.Ctrl(order_byte_e::clear_error);
	j7_left.Ctrl(order_byte_e::clear_error);
	osDelay(1);
	Joint_SoftStartAll();  // 起点取最新反馈，终点保留急停前保存的位置
	osDelay(1);
	j1_left.Ctrl(order_byte_e::enable);
	j2_left.Ctrl(order_byte_e::enable);
	j3_left.Ctrl(order_byte_e::enable);
	j4_left.Ctrl(order_byte_e::enable);
	osDelay(1);
	j5_left.Ctrl(order_byte_e::enable);
	j6_left.Ctrl(order_byte_e::enable);
	j7_left.Ctrl(order_byte_e::enable);
	osDelay(1);
	osThreadResume(roboARMHandle);
}

/* implements the CubeMX "Error" thread (weak stub in freertos.c)
 *
 * 系统生命周期状态机（挂起/解挂起统一在这里做；遥控接收只在中断里更新快照，不参与状态机）：
 *   挂起条件 = 遥控丢失（连续 3s 无有效帧，上电未接遥控也算）或 急停（s1/s2 都打下）
 *   挂起     = 挂起执行任务 + 保存恢复位置 + 失能全部电机
 *   挂起期间 = 每轮补发一帧失能指令保活反馈（见 ArmFeedbackKeepAlive）
 *   解挂起   = 清错 + 从反馈规划回保存位置 + 使能 + 恢复执行任务
 *
 * 上电后 holding 初值为 false，而遥控未上线时 Remote_IsHoldRequested() 就为 1，
 * 所以第一轮循环就会挂起；收到遥控器信息且没有急停后才第一次解挂起。 */
void Error_tasks(void *argument)
{
	UNUSED(argument);
	bool holding = false;

	for (;;)
	{
		/* 接收看门狗：UART5 的 IDLE+DMA 接收可能因一次错误被 HAL 悄悄打死
		   （见 Remote_Supervise 注释），这里每轮兜底重建，
		   避免出现“冷启动接遥控后必须复位一次才能退出急停”。 */
		Remote_Supervise();

		if (Remote_IsHoldRequested() != 0U)
		{
			if (!holding)
			{
				holding = true;
				ArmHold();
			}
			ArmFeedbackKeepAlive();  // 挂起期间保持角度反馈持续刷新，见函数注释
		}
		else if (holding)
		{
			holding = false;
			ArmResume();
		}

		osDelay(10);  // 10ms 轮询：与遥控/控制周期同量级，状态切换延迟可忽略
	}
}
