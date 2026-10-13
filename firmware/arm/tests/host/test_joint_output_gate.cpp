#include "Host.h"
#include "Remote.h"
#include "arm_output_gate.h"
#include "bsp_can.hpp"
#include "main.h"
#include "roboarm.hpp"
#include "usart.h"

#include <cassert>
#include <cstdint>
#include <cstdlib>
#include <cstdio>
#include <cstring>

namespace
{
uint32_t now_tick = 1000U;
uint32_t can_tx_count = 0U;
uint32_t critical_depth = 0U;
uint8_t* remote_rx_buffer = nullptr;
FDCAN_HandleTypeDef fdcan_handle {};

constexpr Motor_DM_N::param_factor_s motor_factors = Motor_DM_N::MakeFactors(6.28f, 30.f, 10.f);
constexpr pid_pos_cfg_s pid_cfg {1.f, 0.f, 0.f, 0.002f, 0.001f, 0.f, 0.f, 0.f, 1.f, 1.f};
constexpr Joint_N::angle_limit_s joint_limit {2.f, -2.f};
constexpr RoboArm_N::mdh_param_s mdh {0.f, 0.f, 0.f, 0.f, {0.f, 0.f, 1.f}};
}

Bsp_CAN abc_can3(&fdcan_handle);
uint8_t BeginOutputResume(const uint8_t host_control)
{
	const can_health_snapshot_s can_health = abc_can3.HealthSnapshot();
	uint32_t remote_mode_generation = 0U;
	assert(Remote_HostControlSnapshot(&remote_mode_generation) == host_control);
	return arm_output_gate_begin_resume(host_control, remote_mode_generation,
		can_health.tx_failure_count,
		can_health.rx_failure_count, can_health.error_event_count);
}
Joint j1_left(0x11U, 0x01U, motor_factors, abc_can3, 0.f, joint_limit, 0.f, mdh,
	pid_cfg, 1U, 1U, 0.f, 1.f);
Joint j2_left(0x12U, 0x02U, motor_factors, abc_can3, 0.f, joint_limit, 0.f, mdh,
	pid_cfg, 1U, 1U, 0.f, 1.f);
Joint j3_left(0x13U, 0x03U, motor_factors, abc_can3, 0.f, joint_limit, 0.f, mdh,
	pid_cfg, 1U, 1U, 0.f, 1.f);
Joint j4_left(0x14U, 0x04U, motor_factors, abc_can3, 0.f, joint_limit, 0.f, mdh,
	pid_cfg, 1U, 1U, 0.f, 1.f);
Joint j5_left(0x15U, 0x05U, motor_factors, abc_can3, 0.f, joint_limit, 0.f, mdh,
	pid_cfg, 1U, 1U, 0.f, 1.f);
Joint j6_left(0x16U, 0x06U, motor_factors, abc_can3, 0.f, joint_limit, 0.f, mdh,
	pid_cfg, 1U, 1U, 0.f, 1.f);
Joint j7_left(0x17U, 0x07U, motor_factors, abc_can3, 0.f, joint_limit, 0.f, mdh,
	pid_cfg, 1U, 1U, 0.f, 1.f);

USART_TypeDef uart5_instance {};
UART_HandleTypeDef huart5 {&uart5_instance, 0U, 0U};
DMA_HandleTypeDef hdma_uart5_rx {0U};

extern "C" uint32_t HAL_GetTick(void) { return now_tick; }
extern "C" uint32_t __get_PRIMASK(void) { return 0U; }
extern "C" void __disable_irq(void) {}
extern "C" void __set_PRIMASK(uint32_t) {}
extern "C" void HostTestEnterCritical(void) { ++critical_depth; }
extern "C" void HostTestExitCritical(void)
{
	assert(critical_depth != 0U);
	--critical_depth;
}
extern "C" HAL_StatusTypeDef HAL_UARTEx_ReceiveToIdle_DMA(
	UART_HandleTypeDef* uart, uint8_t* buffer, uint16_t)
{
	remote_rx_buffer = buffer;
	uart->RxState = HAL_UART_STATE_BUSY_RX;
	uart->Instance->CR3 |= USART_CR3_DMAR;
	hdma_uart5_rx.State = HAL_DMA_STATE_BUSY;
	return HAL_OK;
}
extern "C" HAL_StatusTypeDef HAL_UART_AbortReceive(UART_HandleTypeDef* uart)
{
	uart->RxState = 0U;
	uart->Instance->CR3 &= ~USART_CR3_DMAR;
	hdma_uart5_rx.State = 0U;
	return HAL_OK;
}
extern "C" HAL_StatusTypeDef HAL_UART_DeInit(UART_HandleTypeDef* uart)
{
	return HAL_UART_AbortReceive(uart);
}
extern "C" void MX_UART5_Init(void) {}
extern "C" void Joint_SetRemoteVelocity(unsigned char, float, unsigned char) {}
extern "C" void Gripper_Command(unsigned char) {}
extern "C" uint8_t Host_IsCommandFresh(void) { return 1U; }
extern "C" HAL_StatusTypeDef HAL_FDCAN_AddMessageToTxFifoQ(
	FDCAN_HandleTypeDef*, const FDCAN_TxHeaderTypeDef*, const uint8_t*)
{
	++can_tx_count;
	return HAL_OK;
}
extern "C" HAL_StatusTypeDef HAL_FDCAN_ConfigFilter(FDCAN_HandleTypeDef*, const FDCAN_FilterTypeDef*) { return HAL_OK; }
extern "C" HAL_StatusTypeDef HAL_FDCAN_ConfigGlobalFilter(FDCAN_HandleTypeDef*, uint32_t, uint32_t, uint32_t, uint32_t) { return HAL_OK; }
extern "C" HAL_StatusTypeDef HAL_FDCAN_ConfigFifoWatermark(FDCAN_HandleTypeDef*, uint32_t, uint32_t) { return HAL_OK; }
extern "C" HAL_StatusTypeDef HAL_FDCAN_Start(FDCAN_HandleTypeDef*) { return HAL_OK; }
extern "C" HAL_StatusTypeDef HAL_FDCAN_ActivateNotification(FDCAN_HandleTypeDef*, uint32_t, uint32_t) { return HAL_OK; }
extern "C" void Error_Handler(void) { std::abort(); }

extern "C" void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef* uart, uint16_t size);

void SendRemoteSelectors(const uint8_t s1, const uint8_t s2)
{
	uint8_t frame[RC_FRAME_LENGTH] {};
	frame[5] = static_cast<uint8_t>((s1 << 4U) | (s2 << 6U));
	assert(remote_rx_buffer != nullptr);
	std::memcpy(remote_rx_buffer, frame, sizeof(frame));
	++now_tick;
	HAL_UARTEx_RxEventCallback(&huart5, sizeof(frame));
}

void SetRemoteHostControl(const uint8_t host_control)
{
	SendRemoteSelectors(RC_SW_MID, host_control != 0U ? RC_SW_UP : RC_SW_DOWN);
}

int main()
{
	Remote_Init();
	SendRemoteSelectors(RC_SW_MID, RC_SW_DOWN);
	Joint* const joints[7U] = {&j1_left, &j2_left, &j3_left, &j4_left, &j5_left, &j6_left, &j7_left};
	uint8_t feedback[8U] {0U, 0x7FU, 0xFFU, 0x7FU, 0xF0U, 0x7FU, 25U, 25U};
	for (Joint* const joint : joints)
	{
		joint->UpdateFeedback(feedback);
		joint->SetTarget(0.5f);
		joint->UpdateReference();
	}

	j1_left.ControlStep();
	assert(can_tx_count == 0U); /* output remains closed before the recovery state is armed */

	assert(BeginOutputResume(0U) != 0U);
	assert(arm_output_gate_activate() != 0U);
	j1_left.ControlStep();
	assert(can_tx_count == 1U); /* the real Joint::ControlStep reaches gated MIT output */

	SetRemoteHostControl(1U);
	j1_left.ControlStep();
	assert(can_tx_count == 1U); /* selector change blocks the very next output */
	SetRemoteHostControl(0U);
	j1_left.ControlStep();
	assert(can_tx_count == 1U); /* returning to the old mode does not silently re-arm */

	arm_output_gate_revoke();
	SendRemoteSelectors(RC_SW_MID, RC_SW_DOWN);
	for (Joint* const joint : joints) joint->UpdateFeedback(feedback);
	assert(BeginOutputResume(0U) != 0U);
	assert(arm_output_gate_activate() != 0U);
	SetRemoteHostControl(1U);
	SetRemoteHostControl(0U); /* fast mode transition and return between Joint::ControlStep calls */
	j1_left.ControlStep();
	assert(can_tx_count == 1U);

	arm_output_gate_revoke();
	SetRemoteHostControl(0U); /* refresh the real remote lease before recovery */
	for (Joint* const joint : joints) joint->UpdateFeedback(feedback);
	assert(BeginOutputResume(0U) != 0U);
	assert(arm_output_gate_activate() != 0U);
	j1_left.ControlStep();
	assert(can_tx_count == 2U);
	now_tick += 100U;
	j1_left.ControlStep();
	assert(can_tx_count == 3U); /* feedback is valid at the inclusive 100 ms boundary */
	++now_tick;
	j1_left.ControlStep();
	assert(can_tx_count == 3U); /* stale feedback blocks before the next output */

	arm_output_gate_revoke();
	for (Joint* const joint : joints) joint->UpdateFeedback(feedback);
	SendRemoteSelectors(RC_SW_MID, RC_SW_DOWN); /* replace the expired RC lease */
	assert(BeginOutputResume(0U) != 0U);
	assert(arm_output_gate_activate() != 0U);
	feedback[0] = static_cast<uint8_t>(Motor_DM_N::error_state_e::over_temperature_mos) << 4U;
	j4_left.UpdateFeedback(feedback);
	j1_left.ControlStep();
	assert(can_tx_count == 3U); /* one joint driver fault closes all motion output immediately */
	assert(critical_depth == 0U);

	std::puts("PASS production Joint::ControlStep output gate tests");
	return 0;
}
