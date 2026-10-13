#include "Host.h"
#include "FreeRTOS.h"
#include "arm_output_gate.h"
#include "bsp_can.hpp"
#include "cmsis_os2.h"
#include "main.h"
#include "motor_dm.hpp"
#include "ring_buffer.hpp"
#include "roboarm.hpp"
#include "threads.hpp"
#include "usbd_cdc_if.h"

#include <cassert>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <cstdlib>
#include <initializer_list>
#include <stdexcept>
#include <cstdio>

extern Ring_Queue<uint8_t, HOST_RX_QUEUE_SIZE> host_usb_queue;
extern Bsp_CAN abc_can3;
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef* hfdcan, uint32_t rx_fifo_interrupts);
void HAL_FDCAN_ErrorStatusCallback(FDCAN_HandleTypeDef* hfdcan, uint32_t error_status);

namespace
{
struct stop_test_t {};

enum class task_scenario_e
{
	none,
	resume_success,
	disconnect_during_resume,
	disconnect_during_enable,
	one_axis_fault
};

uint32_t now_tick = 1000U;
uint32_t critical_depth = 0U;
uint32_t can_tx_attempts = 0U;
HAL_StatusTypeDef can_tx_result = HAL_OK;
uint32_t cdc_tx_count = 0U;
uint8_t cdc_last_tx[HOST_TX_RESPONSE_LEN] {};
uint8_t remote_online = 1U;
uint8_t remote_hold = 0U;
uint8_t remote_host_control = 1U;
uint32_t remote_mode_generation = 0U;
uint8_t manual_rearm_actions = 0U;
bool roboarm_suspended = false;
bool roboarm_resumed = false;
task_scenario_e task_scenario = task_scenario_e::none;
uint32_t delay_calls = 0U;
uint32_t mock_protocol_bus_off = 0U;
uint32_t mock_protocol_last_error = 0U;
uint32_t mock_can_tx_errors = 0U;
uint32_t mock_can_rx_errors = 0U;
HAL_StatusTypeDef mock_rx_read_status = HAL_OK;
uint32_t mock_rx_identifier = 0x11U;
uint32_t mock_rx_data_length = FDCAN_DLC_BYTES_8;
uint8_t mock_rx_payload[8U] {};

FDCAN_HandleTypeDef test_fdcan {};
USBD_CDC_HandleTypeDef cdc_handle {};

void CompleteUsbResponse()
{
	cdc_handle.TxState = 0U;
	Host_TxComplete();
}

void SetRemoteHostControl(const uint8_t host_control)
{
	const uint8_t normalized = host_control != 0U ? 1U : 0U;
	if (normalized != remote_host_control) ++remote_mode_generation;
	remote_host_control = normalized;
}

uint8_t BeginOutputResume(const uint8_t host_control)
{
	const can_health_snapshot_s can_health = abc_can3.HealthSnapshot();
	return arm_output_gate_begin_resume(host_control, remote_mode_generation,
		can_health.tx_failure_count,
		can_health.rx_failure_count, can_health.error_event_count);
}

void PushHostFrame(const uint8_t enable, const float angle)
{
	uint8_t frame[HOST_RX_FRAME_LEN] {};
	frame[0] = 0xA5U;
	frame[13] = 0x4AU;
	frame[14] = 0xB5U;
	frame[15] = enable;
	for (uint8_t i = 0U; i < 6U; ++i)
	{
		const float target = angle + static_cast<float>(i) * 0.1f;
		std::memcpy(&frame[20U + static_cast<uint16_t>(i) * sizeof(float)], &target, sizeof(target));
	}
	frame[44] = 0U;
	frame[45] = 0x4BU;
	assert(host_usb_queue.PushBatch(frame, HOST_RX_FRAME_LEN));
	Host_Poll();
	CompleteUsbResponse();
}

void ResetJoints()
{
	Joint* const joints[7U] = {&j1_left, &j2_left, &j3_left, &j4_left, &j5_left, &j6_left, &j7_left};
	for (Joint* const joint : joints)
	{
		joint->ResetForTest();
	}
}

void ResetHostAndGate()
{
	Host_ResetSafetyHandshake();
	Host_SetSafetyArmed(0U);
	arm_output_gate_revoke();
	remote_online = 1U;
	remote_hold = 0U;
	SetRemoteHostControl(1U);
	CompleteUsbResponse();
}

void EstablishHostHandshake()
{
	PushHostFrame(0U, 0.25f);
	PushHostFrame(1U, 0.50f);
	assert(Host_HasDisabledSinceSafetyReset() != 0U);
	assert(Host_HasEnableAfterDisabled() != 0U);
	assert(Host_IsCommandFresh() != 0U);
}

void TestHostParserAndLeaseBoundaries()
{
	ResetJoints();
	ResetHostAndGate();
	now_tick = 1000U;
	EstablishHostHandshake();
	assert(host_debug_adopted == 0U); /* handshake does not adopt targets */

	Host_SetSafetyArmed(1U);
	PushHostFrame(1U, 1.25f);
	assert(host_debug_adopted == 1U);
	assert(j1_left.target_count == 1U);
	assert(std::fabs(j1_left.last_target - 1.25f) < 1.e-6f);

	now_tick += HOST_COMMAND_LEASE_MS;
	assert(Host_IsCommandFresh() != 0U); /* the lease is inclusive at 100 ms */
	now_tick += 1U;
	assert(Host_IsCommandFresh() == 0U); /* 101 ms expires and latches */
	const uint32_t targets_before_stale_frame = j1_left.target_count;
	PushHostFrame(1U, 3.0f);
	assert(Host_IsCommandFresh() == 0U); /* a late enable frame cannot renew an expired lease */
	assert(host_debug_adopted == 0U);
	assert(j1_left.target_count == targets_before_stale_frame);
	assert(Host_ConsumeSafetyDisableEvent() != 0U);

	ResetHostAndGate();
	ResetJoints();
	now_tick = 3000U;
	EstablishHostHandshake();
	Host_SetSafetyArmed(1U);
	PushHostFrame(1U, 0.75f);
	assert(host_debug_adopted == 1U);
	now_tick += HOST_COMMAND_LEASE_MS;
	PushHostFrame(1U, 0.85f); /* a timely duplicate enable refreshes a live lease */
	assert(host_debug_adopted == 1U);
	assert(Host_IsCommandFresh() != 0U);
}

void TestMotorOutputGateAndDriverParser()
{
	ResetHostAndGate();
	now_tick = 5000U;
	ResetJoints();
	EstablishHostHandshake();
	Host_SetSafetyArmed(1U);
	PushHostFrame(1U, 0.2f);
	assert(BeginOutputResume(1U) != 0U);

	const auto factors = Motor_DM_N::MakeFactors(6.28f, 45.f, 54.f);
	Motor_DM motor(0x11U, 0x01U, factors, abc_can3);
	const uint32_t before_resume_tx = can_tx_attempts;
	assert(motor.Ctrl(Motor_DM_N::order_byte_e::enable));
	assert(!motor.Ctrl(0.f, 0.f, 0U, 0U, 0.f)); /* motion is forbidden while resuming */
	assert(can_tx_attempts == before_resume_tx + 1U);
	assert(arm_output_gate_activate() != 0U);
	assert(motor.Ctrl(0.2f, 0.f, 0U, 0U, 0.f));

	const uint32_t before_duplicate_enable = can_tx_attempts;
	assert(!motor.Ctrl(Motor_DM_N::order_byte_e::enable)); /* duplicate enable cannot bypass resume */
	assert(can_tx_attempts == before_duplicate_enable);

	SetRemoteHostControl(0U); /* mode switch closes the gate before the next joint output */
	assert(!motor.Ctrl(0.3f, 0.f, 0U, 0U, 0.f));
	SetRemoteHostControl(1U);
	assert(!motor.Ctrl(0.3f, 0.f, 0U, 0U, 0.f)); /* revocation stays latched */

	ResetHostAndGate();
	now_tick = 6500U;
	ResetJoints();
	EstablishHostHandshake();
	Host_SetSafetyArmed(1U);
	PushHostFrame(1U, 0.4f);
	assert(BeginOutputResume(1U) != 0U);
	assert(arm_output_gate_activate() != 0U);
	SetRemoteHostControl(0U);
	SetRemoteHostControl(1U); /* both selector transitions occur before the next task poll */
	const uint32_t before_selector_aba = can_tx_attempts;
	assert(!motor.Ctrl(0.3f, 0.f, 0U, 0U, 0.f));
	assert(can_tx_attempts == before_selector_aba);

	ResetHostAndGate();
	now_tick = 7000U;
	ResetJoints();
	EstablishHostHandshake();
	Host_SetSafetyArmed(1U);
	PushHostFrame(1U, 0.4f);
	assert(BeginOutputResume(1U) != 0U);
	assert(arm_output_gate_activate() != 0U);
	const uint32_t before_lease_edge = can_tx_attempts;
	now_tick += HOST_COMMAND_LEASE_MS;
	assert(motor.Ctrl(0.4f, 0.f, 0U, 0U, 0.f));
	now_tick += 1U;
	assert(!motor.Ctrl(0.5f, 0.f, 0U, 0U, 0.f));
	assert(can_tx_attempts == before_lease_edge + 1U);

	ResetHostAndGate();
	now_tick = 9000U;
	ResetJoints();
	EstablishHostHandshake();
	Host_SetSafetyArmed(1U);
	PushHostFrame(1U, 0.1f);
	assert(BeginOutputResume(1U) != 0U);
	assert(arm_output_gate_activate() != 0U);
	can_tx_result = HAL_ERROR;
	assert(!motor.Ctrl(0.1f, 0.f, 0U, 0U, 0.f));
	assert(abc_can3.TxFailureCount() == 1U);
	can_tx_result = HAL_OK;
	assert(!motor.Ctrl(0.2f, 0.f, 0U, 0U, 0.f));

	uint8_t feedback[8U] {static_cast<uint8_t>(0x0BU << 4U), 0U, 0U, 0U, 0U, 0U, 25U, 25U};
	motor.UpdateFeedback(feedback);
	assert(motor.HasDriverFault()); /* DM status nibble is real feedback, not inferred */
	feedback[0] = 0U;
	motor.UpdateFeedback(feedback);
	assert(!motor.HasDriverFault());

	ResetHostAndGate();
	now_tick = 10000U;
	ResetJoints();
	EstablishHostHandshake();
	Host_SetSafetyArmed(1U);
	PushHostFrame(1U, 0.1f);
	assert(BeginOutputResume(1U) != 0U);
	assert(arm_output_gate_activate() != 0U);
	mock_protocol_bus_off = 1U;
	mock_protocol_last_error = 3U;
	HAL_FDCAN_ErrorStatusCallback(&test_fdcan, FDCAN_IT_BUS_OFF);
	assert(abc_can3.IsBusOff());
	assert(abc_can3.ErrorEventCount() != 0U);
	assert(!motor.Ctrl(0.1f, 0.f, 0U, 0U, 0.f));
	mock_protocol_bus_off = 0U;
	HAL_FDCAN_ErrorStatusCallback(&test_fdcan, FDCAN_IT_ERROR_WARNING);
	assert(!motor.Ctrl(0.2f, 0.f, 0U, 0U, 0.f)); /* the active gate stays tripped */

	ResetHostAndGate();
	now_tick = 10500U;
	ResetJoints();
	SetRemoteHostControl(0U);
	const can_health_snapshot_s pre_resume_health = abc_can3.HealthSnapshot();
	HAL_FDCAN_ErrorStatusCallback(&test_fdcan, FDCAN_IT_ERROR_WARNING);
	assert(arm_output_gate_begin_resume(0U, remote_mode_generation,
		pre_resume_health.tx_failure_count,
		pre_resume_health.rx_failure_count, pre_resume_health.error_event_count) == 0U);
	/* A new CAN error between state sampling and resume is compared with that same snapshot. */

	ResetHostAndGate();
	now_tick = 11000U;
	ResetJoints();
	SetRemoteHostControl(0U);
	assert(BeginOutputResume(0U) != 0U);
	assert(arm_output_gate_activate() != 0U);
	const uint16_t feedback_before = j1_left.FeedbackCount();
	mock_rx_payload[0] = static_cast<uint8_t>(Motor_DM_N::error_state_e::over_temperature_mos) << 4U;
	HAL_FDCAN_RxFifo0Callback(&test_fdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE);
	assert(j1_left.FeedbackCount() == static_cast<uint16_t>(feedback_before + 1U));
	assert(!motor.Ctrl(0.2f, 0.f, 0U, 0U, 0.f)); /* a decoded single-axis fault closes the gate immediately */

	ResetHostAndGate();
	now_tick = 12000U;
	ResetJoints();
	SetRemoteHostControl(0U);
	assert(BeginOutputResume(0U) != 0U);
	assert(arm_output_gate_activate() != 0U);
	mock_rx_read_status = HAL_ERROR;
	const uint16_t feedback_before_failed_read = j1_left.FeedbackCount();
	HAL_FDCAN_RxFifo0Callback(&test_fdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE);
	assert(j1_left.FeedbackCount() == feedback_before_failed_read); /* failed read cannot replay old header */
	assert(abc_can3.RxFailureCount() == 1U);
	assert(!motor.Ctrl(0.2f, 0.f, 0U, 0U, 0.f));
	mock_rx_read_status = HAL_OK;

	ResetHostAndGate();
	now_tick = 13000U;
	ResetJoints();
	SetRemoteHostControl(0U);
	assert(BeginOutputResume(0U) != 0U);
	assert(arm_output_gate_activate() != 0U);
	mock_rx_data_length = 4U;
	const uint16_t feedback_before_short_frame = j1_left.FeedbackCount();
	const uint32_t rx_failures_before_short_frame = abc_can3.RxFailureCount();
	HAL_FDCAN_RxFifo0Callback(&test_fdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE);
	assert(j1_left.FeedbackCount() == feedback_before_short_frame);
	assert(abc_can3.RxFailureCount() == rx_failures_before_short_frame + 1U);
	assert(!motor.Ctrl(0.2f, 0.f, 0U, 0U, 0.f));
	mock_rx_data_length = FDCAN_DLC_BYTES_8;
}

void ResetTaskScenario(const task_scenario_e scenario)
{
	now_tick = 12000U;
	ResetJoints();
	ResetHostAndGate();
	Host_ResetSafetyHandshake();
	Host_SetSafetyArmed(0U);
	remote_online = 1U;
	remote_hold = 0U;
	SetRemoteHostControl(1U);
	manual_rearm_actions = 1U;
	roboarm_suspended = false;
	roboarm_resumed = false;
	task_scenario = scenario;
	delay_calls = 0U;
	if (scenario == task_scenario_e::one_axis_fault) j4_left.fault = true;
}

void RunTaskScenario(const task_scenario_e scenario)
{
	ResetTaskScenario(scenario);
	try
	{
		Error_tasks(nullptr);
		assert(false && "Error_tasks test loop should be interrupted by the scheduler stub");
	}
	catch (const stop_test_t&)
	{
	}
	task_scenario = task_scenario_e::none;
}

void TestTaskRecoveryAndFaultBoundaries()
{
	RunTaskScenario(task_scenario_e::resume_success);
	assert(roboarm_resumed && !roboarm_suspended);
	assert(j1_left.clear_count == 1U && j7_left.clear_count == 1U);
	assert(j1_left.enable_count == 1U && j7_left.enable_count == 1U);
	assert(j1_left.soft_start_count == 1U && j7_left.soft_start_count == 1U);
	assert(arm_output_gate_begin(ARM_OUTPUT_MOTION) != 0U);
	arm_output_gate_end();

	RunTaskScenario(task_scenario_e::disconnect_during_resume);
	assert(!roboarm_resumed && roboarm_suspended);
	assert(j1_left.clear_count == 1U && j4_left.clear_count == 1U);
	assert(j5_left.clear_count == 0U && j7_left.clear_count == 0U);
	assert(j1_left.enable_count == 0U && j7_left.enable_count == 0U);

	RunTaskScenario(task_scenario_e::disconnect_during_enable);
	assert(!roboarm_resumed && roboarm_suspended);
	assert(j1_left.clear_count == 1U && j7_left.clear_count == 1U);
	assert(j1_left.enable_count == 1U && j4_left.enable_count == 1U);
	assert(j5_left.enable_count == 0U && j7_left.enable_count == 0U);

	RunTaskScenario(task_scenario_e::one_axis_fault);
	assert(!roboarm_resumed && roboarm_suspended);
	assert(j4_left.clear_count == 0U && j4_left.enable_count == 0U);
	assert(j1_left.enable_count == 0U && j7_left.enable_count == 0U);
}
}

USBD_HandleTypeDef hUsbDeviceHS {USBD_STATE_CONFIGURED, &cdc_handle};
Bsp_CAN abc_can3(&test_fdcan);
Joint j1_left(abc_can3), j2_left(abc_can3), j3_left(abc_can3), j4_left(abc_can3);
Joint j5_left(abc_can3), j6_left(abc_can3), j7_left(abc_can3);
Ring_Queue<uint8_t, HOST_RX_QUEUE_SIZE> host_usb_queue;
volatile uint32_t host_link_tick = 0U;
volatile uint32_t host_rx_frames = 0U;
volatile uint32_t host_rx_drop = 0U;
volatile uint32_t host_tx_drop = 0U;
volatile uint8_t gripper_close_target = 0U;
float host_debug_target[HOST_DEBUG_DOF] {};
uint8_t host_debug_enable = 0U;
uint8_t host_debug_gripper = 0U;
uint8_t host_debug_adopted = 0U;
osThreadId_t roboARMHandle = reinterpret_cast<osThreadId_t>(1U);

extern "C" uint32_t HAL_GetTick(void) { return now_tick; }
extern "C" uint32_t __get_PRIMASK(void) { return 0U; }
extern "C" void __disable_irq(void) {}
extern "C" void __set_PRIMASK(uint32_t) {}
extern "C" void HostTestEnterCritical(void) { ++critical_depth; }
extern "C" void HostTestExitCritical(void) { assert(critical_depth != 0U); --critical_depth; }
extern "C" uint8_t Remote_IsOnline(void) { return remote_online; }
extern "C" uint8_t Remote_IsHoldRequested(void) { return remote_hold; }
extern "C" uint8_t Remote_IsHostControl(void) { return remote_online && remote_host_control; }
extern "C" uint8_t Remote_HostControlSnapshot(uint32_t* mode_generation)
{
	if (mode_generation != nullptr) *mode_generation = remote_mode_generation;
	return remote_online && remote_host_control;
}
extern "C" uint8_t Remote_ConsumeManualRearmRequest(void)
{
	if (manual_rearm_actions == 0U) return 0U;
	--manual_rearm_actions;
	return 1U;
}
extern "C" void Remote_Supervise(void) {}
extern "C" void Gripper_Command(const uint8_t target) { gripper_close_target = target != 0U; }
extern "C" float Gripper_OpenPosition(void) { return 1.f; }
extern "C" float Gripper_ClosePosition(void) { return 0.f; }
void Joint_CaptureHoldAll(void)
{
	for (Joint* const joint : {&j1_left, &j2_left, &j3_left, &j4_left, &j5_left, &j6_left, &j7_left})
		joint->CaptureHoldPosition();
}
void Joint_SoftStartAll(void)
{
	for (Joint* const joint : {&j1_left, &j2_left, &j3_left, &j4_left, &j5_left, &j6_left, &j7_left})
		joint->SoftStart();
}

extern "C" osStatus_t osThreadSuspend(osThreadId_t) { roboarm_suspended = true; return 0; }
extern "C" osStatus_t osThreadResume(osThreadId_t) { roboarm_resumed = true; roboarm_suspended = false; return 0; }
extern "C" void osDelay(const uint32_t ticks)
{
	now_tick += ticks;
	++delay_calls;
	if (task_scenario == task_scenario_e::resume_success && delay_calls == 3U)
	{
		PushHostFrame(0U, 0.2f);
		PushHostFrame(1U, 0.3f);
	}
	if (task_scenario == task_scenario_e::disconnect_during_resume)
	{
		if (delay_calls == 3U)
		{
			PushHostFrame(0U, 0.2f);
			PushHostFrame(1U, 0.3f);
		}
		if (delay_calls == 6U) remote_online = 0U;
		if (delay_calls >= 11U) throw stop_test_t{};
	}
	if (task_scenario == task_scenario_e::disconnect_during_enable)
	{
		if (delay_calls == 3U)
		{
			PushHostFrame(0U, 0.2f);
			PushHostFrame(1U, 0.3f);
		}
		if (delay_calls == 9U) remote_online = 0U;
		if (delay_calls >= 14U) throw stop_test_t{};
	}
	if (task_scenario == task_scenario_e::one_axis_fault)
	{
		if (delay_calls == 3U)
		{
			PushHostFrame(0U, 0.2f);
			PushHostFrame(1U, 0.3f);
		}
		if (delay_calls >= 6U) throw stop_test_t{};
	}
	if (task_scenario == task_scenario_e::resume_success && roboarm_resumed && delay_calls >= 8U)
		throw stop_test_t{};
}

extern "C" uint8_t USBD_CDC_SetTxBuffer(USBD_HandleTypeDef* device, uint8_t* buffer, const uint16_t length)
{
	auto* cdc = static_cast<USBD_CDC_HandleTypeDef*>(device->pClassData);
	cdc->TxBuffer = buffer;
	cdc->TxLength = length;
	return USBD_OK;
}
extern "C" uint8_t USBD_CDC_TransmitPacket(USBD_HandleTypeDef* device)
{
	auto* cdc = static_cast<USBD_CDC_HandleTypeDef*>(device->pClassData);
	assert(cdc->TxLength == HOST_TX_RESPONSE_LEN);
	std::memcpy(cdc_last_tx, cdc->TxBuffer, HOST_TX_RESPONSE_LEN);
	cdc->TxState = 1U;
	++cdc_tx_count;
	return USBD_OK;
}

extern "C" HAL_StatusTypeDef HAL_FDCAN_ConfigFilter(FDCAN_HandleTypeDef*, const FDCAN_FilterTypeDef*) { return HAL_OK; }
extern "C" HAL_StatusTypeDef HAL_FDCAN_ConfigGlobalFilter(FDCAN_HandleTypeDef*, uint32_t, uint32_t, uint32_t, uint32_t) { return HAL_OK; }
extern "C" HAL_StatusTypeDef HAL_FDCAN_ConfigFifoWatermark(FDCAN_HandleTypeDef*, uint32_t, uint32_t) { return HAL_OK; }
extern "C" HAL_StatusTypeDef HAL_FDCAN_Start(FDCAN_HandleTypeDef*) { return HAL_OK; }
extern "C" HAL_StatusTypeDef HAL_FDCAN_ActivateNotification(FDCAN_HandleTypeDef*, uint32_t, uint32_t) { return HAL_OK; }
extern "C" HAL_StatusTypeDef HAL_FDCAN_AddMessageToTxFifoQ(FDCAN_HandleTypeDef*, const FDCAN_TxHeaderTypeDef*, const uint8_t*)
{
	++can_tx_attempts;
	return can_tx_result;
}
extern "C" HAL_StatusTypeDef HAL_FDCAN_GetRxMessage(FDCAN_HandleTypeDef*, uint32_t, FDCAN_RxHeaderTypeDef* header, uint8_t* payload)
{
	if (mock_rx_read_status != HAL_OK) return mock_rx_read_status;
	header->Identifier = mock_rx_identifier;
	header->DataLength = mock_rx_data_length;
	std::memcpy(payload, mock_rx_payload, sizeof(mock_rx_payload));
	return HAL_OK;
}
extern "C" HAL_StatusTypeDef HAL_FDCAN_GetErrorCounters(FDCAN_HandleTypeDef*, FDCAN_ErrorCountersTypeDef* counters)
{
	counters->TxErrorCnt = mock_can_tx_errors;
	counters->RxErrorCnt = mock_can_rx_errors;
	return HAL_OK;
}
extern "C" HAL_StatusTypeDef HAL_FDCAN_GetProtocolStatus(FDCAN_HandleTypeDef*, FDCAN_ProtocolStatusTypeDef* status)
{
	status->LastErrorCode = mock_protocol_last_error;
	status->BusOff = mock_protocol_bus_off;
	return HAL_OK;
}
extern "C" uint32_t HAL_FDCAN_GetError(FDCAN_HandleTypeDef*) { return 0U; }
extern "C" void Error_Handler(void) { std::abort(); }

int main()
{
	TestHostParserAndLeaseBoundaries();
	TestMotorOutputGateAndDriverParser();
	TestTaskRecoveryAndFaultBoundaries();
	assert(critical_depth == 0U);
	assert(cdc_tx_count != 0U);
	std::puts("PASS arm host parser, output gate, and RTOS recovery integration tests");
	return 0;
}
