#include "threads.hpp"
#include "roboarm.hpp"
#include "Remote.h"
#include "Host.h"
#include "arm_safety_state.h"
#include "arm_output_gate.h"
#include "bsp_can.hpp"
#include "main.h"

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
extern Bsp_CAN abc_can3;

namespace
{
constexpr uint32_t arm_feedback_stale_after_ms = HOST_COMMAND_LEASE_MS;
constexpr uint32_t arm_resume_ack_timeout_ms = 100U;

Joint* const arm_joints[7U] =
{
	&j1_left, &j2_left, &j3_left, &j4_left, &j5_left, &j6_left, &j7_left
};

bool ArmFeedbackHealthy(const uint32_t now)
{
	for (Joint* const joint : arm_joints)
	{
		if (!joint->FeedbackHealthy(now, arm_feedback_stale_after_ms)) return false;
	}
	return true;
}

bool ArmResumeAuthorizationCurrent(void)
{
	return arm_output_gate_authorization_current() != 0U &&
		!abc_can3.IsBusOff() && ArmFeedbackHealthy(HAL_GetTick());
}

bool WaitForFeedbackAfter(const uint16_t before[7U], const bool require_enabled)
{
	const uint32_t start_tick = HAL_GetTick();
	for (;;)
	{
		if (!ArmResumeAuthorizationCurrent()) return false;
		bool all_updated = true;
		bool all_enabled = true;
		for (uint8_t i = 0U; i < 7U; ++i)
		{
			all_updated = all_updated && arm_joints[i]->FeedbackUpdatedAfter(before[i]);
			all_enabled = all_enabled && arm_joints[i]->DriverEnabled();
		}
		if (all_updated && (!require_enabled || all_enabled)) return true;
		if ((HAL_GetTick() - start_tick) >= arm_resume_ack_timeout_ms) return false;
		osDelay(1U);
	}
}

bool SendRecoveryCommand(Joint& joint, const Motor_DM_N::order_byte_e command,
	const uint8_t feedback_index, uint16_t before[7U])
{
	if (!ArmResumeAuthorizationCurrent()) return false;
	before[feedback_index] = joint.FeedbackCount();
	return joint.Ctrl(command) && ArmResumeAuthorizationCurrent();
}
}

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
static bool ArmResume(const uint8_t host_control, const uint32_t remote_mode_generation,
	const can_health_snapshot_s& can_baseline)
{
	using namespace Motor_DM_N;
	if (arm_output_gate_begin_resume(host_control, remote_mode_generation,
		can_baseline.tx_failure_count,
		can_baseline.rx_failure_count, can_baseline.error_event_count) == 0U ||
		!ArmResumeAuthorizationCurrent())
	{
		return false;
	}
	uint16_t feedback_before[7U] {};
	if (!SendRecoveryCommand(j1_left, order_byte_e::clear_error, 0U, feedback_before) ||
		!SendRecoveryCommand(j2_left, order_byte_e::clear_error, 1U, feedback_before) ||
		!SendRecoveryCommand(j3_left, order_byte_e::clear_error, 2U, feedback_before) ||
		!SendRecoveryCommand(j4_left, order_byte_e::clear_error, 3U, feedback_before)) return false;
	osDelay(1);
	if (!SendRecoveryCommand(j5_left, order_byte_e::clear_error, 4U, feedback_before) ||
		!SendRecoveryCommand(j6_left, order_byte_e::clear_error, 5U, feedback_before) ||
		!SendRecoveryCommand(j7_left, order_byte_e::clear_error, 6U, feedback_before)) return false;
	osDelay(1);
	if (!WaitForFeedbackAfter(feedback_before, false)) return false;
	if (!ArmResumeAuthorizationCurrent()) return false;
	Joint_SoftStartAll();  // 起点取最新反馈，终点保留急停前保存的位置
	osDelay(1);
	if (!ArmResumeAuthorizationCurrent()) return false;
	const uint16_t before_enable[7U] =
	{
		j1_left.FeedbackCount(), j2_left.FeedbackCount(), j3_left.FeedbackCount(),
		j4_left.FeedbackCount(), j5_left.FeedbackCount(), j6_left.FeedbackCount(),
		j7_left.FeedbackCount()
	};
	if (!SendRecoveryCommand(j1_left, order_byte_e::enable, 0U, feedback_before) ||
		!SendRecoveryCommand(j2_left, order_byte_e::enable, 1U, feedback_before) ||
		!SendRecoveryCommand(j3_left, order_byte_e::enable, 2U, feedback_before) ||
		!SendRecoveryCommand(j4_left, order_byte_e::enable, 3U, feedback_before)) return false;
	osDelay(1);
	if (!ArmResumeAuthorizationCurrent()) return false;
	if (!SendRecoveryCommand(j5_left, order_byte_e::enable, 4U, feedback_before) ||
		!SendRecoveryCommand(j6_left, order_byte_e::enable, 5U, feedback_before) ||
		!SendRecoveryCommand(j7_left, order_byte_e::enable, 6U, feedback_before)) return false;
	osDelay(1);
	for (uint8_t i = 0U; i < 7U; ++i) feedback_before[i] = before_enable[i];
	if (!WaitForFeedbackAfter(feedback_before, true)) return false;
	return ArmResumeAuthorizationCurrent();
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
	bool holding = true;
	arm_safety_state_t safety;
	arm_safety_state_init(&safety);
	arm_output_gate_revoke();
	Host_SetSafetyArmed(0U);
	ArmHold();
	Host_ResetSafetyHandshake();
	can_health_snapshot_s previous_can_health = abc_can3.HealthSnapshot();

	for (;;)
	{
		/* 接收看门狗：UART5 的 IDLE+DMA 接收可能因一次错误被 HAL 悄悄打死
		   （见 Remote_Supervise 注释），这里每轮兜底重建，
		   避免出现“冷启动接遥控后必须复位一次才能退出急停”。 */
		Remote_Supervise();

		arm_safety_inputs_t inputs = {};
		inputs.rc_online = Remote_IsOnline();
		uint32_t remote_mode_generation = 0U;
		inputs.host_control = Remote_HostControlSnapshot(&remote_mode_generation);
		inputs.hold_requested =
			(Remote_IsHoldRequested() != 0U || inputs.rc_online == 0U) ? 1U : 0U;
		inputs.manual_rearm_action = Remote_ConsumeManualRearmRequest();
		if (inputs.manual_rearm_action != 0U)
		{
			/* A manual re-arm first invalidates every pre-fault host command/edge. */
			arm_output_gate_revoke();
			Host_SetSafetyArmed(0U);
			Host_ResetSafetyHandshake();
		}
		inputs.host_disable_event = Host_ConsumeSafetyDisableEvent();
		inputs.host_command_fresh = Host_IsCommandFresh();
		inputs.host_disabled_seen = Host_HasDisabledSinceSafetyReset();
		inputs.host_enable_after_disabled = Host_HasEnableAfterDisabled();
		inputs.feedback_healthy = ArmFeedbackHealthy(HAL_GetTick()) ? 1U : 0U;
		const can_health_snapshot_s current_can_health = abc_can3.HealthSnapshot();
		inputs.can_healthy = (current_can_health.bus_off == 0U &&
			current_can_health.tx_failure_count == previous_can_health.tx_failure_count &&
			current_can_health.rx_failure_count == previous_can_health.rx_failure_count &&
			current_can_health.error_event_count == previous_can_health.error_event_count) ? 1U : 0U;
		previous_can_health = current_can_health;

		arm_safety_state_e next_state = arm_safety_state_update(&safety, &inputs);
		if ((next_state != ARM_SAFETY_ARMED) && !holding)
		{
			arm_output_gate_revoke();
			Host_SetSafetyArmed(0U);
			holding = true;
			ArmHold();
			Host_ResetSafetyHandshake();
		}
		else if ((next_state == ARM_SAFETY_ARMED) && holding)
		{
			bool resume_ok = ArmResume(inputs.host_control, remote_mode_generation, current_can_health);
			if (resume_ok)
			{
				Host_SetSafetyArmed(inputs.host_control);
				resume_ok = arm_output_gate_activate() != 0U;
			}
			if (resume_ok)
			{
				osThreadResume(roboARMHandle);
				holding = false;
			}
			else
			{
				arm_output_gate_trip();
				Host_SetSafetyArmed(0U);
				Host_ResetSafetyHandshake();
				arm_safety_state_latch_fault(&safety);
				next_state = ARM_SAFETY_FAULT_LATCHED;
				ArmHold();
			}
		}

		if (next_state != ARM_SAFETY_ARMED)
		{
			ArmFeedbackKeepAlive();  // 挂起期间保持角度反馈持续刷新，见函数注释
		}

		osDelay(10);  // MCU 侧命令租约为 100ms；本任务每 10ms 检查一次。
	}
}
