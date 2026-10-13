#include "arm_output_gate.h"

#include "Host.h"
#include "Remote.h"
#include "bsp_can.hpp"
#include "roboarm.hpp"

#include "FreeRTOS.h"
#include "task.h"

extern Bsp_CAN abc_can3;
extern Joint j1_left;
extern Joint j2_left;
extern Joint j3_left;
extern Joint j4_left;
extern Joint j5_left;
extern Joint j6_left;
extern Joint j7_left;

namespace
{
enum class gate_state_e : uint8_t
{
	disarmed = 0U,
	resuming,
	armed
};

volatile gate_state_e gate_state = gate_state_e::disarmed;
volatile uint8_t expected_host_control = 0U;
uint32_t expected_remote_mode_generation = 0U;
uint32_t expected_can_tx_failures = 0U;
uint32_t expected_can_rx_failures = 0U;
uint32_t expected_can_error_events = 0U;

constexpr uint32_t arm_feedback_stale_after_ms = HOST_COMMAND_LEASE_MS;
Joint* const output_joints[7U] =
{
	&j1_left, &j2_left, &j3_left, &j4_left, &j5_left, &j6_left, &j7_left
};

bool InputsStillAuthorize(void)
{
	if ((Remote_IsOnline() == 0U) || (Remote_IsHoldRequested() != 0U)) return false;
	const can_health_snapshot_s can_health = abc_can3.HealthSnapshot();
	if ((can_health.bus_off != 0U) ||
		(can_health.tx_failure_count != expected_can_tx_failures) ||
		(can_health.rx_failure_count != expected_can_rx_failures) ||
		(can_health.error_event_count != expected_can_error_events)) return false;
	const uint32_t now = HAL_GetTick();
	for (Joint* const joint : output_joints)
	{
		if (!joint->FeedbackHealthy(now, arm_feedback_stale_after_ms)) return false;
	}
	uint32_t remote_mode_generation = 0U;
	const uint8_t host_control = Remote_HostControlSnapshot(&remote_mode_generation);
	if ((host_control != expected_host_control) ||
		(remote_mode_generation != expected_remote_mode_generation)) return false;
	return (host_control == 0U) || (Host_IsCommandFresh() != 0U);
}

void RevokeUnlocked(void)
{
	gate_state = gate_state_e::disarmed;
}
}

extern "C" uint8_t arm_output_gate_begin(const uint8_t output_kind)
{
	taskENTER_CRITICAL();
	const bool state_allows = (output_kind == ARM_OUTPUT_RECOVERY)
		? (gate_state == gate_state_e::resuming)
		: ((output_kind == ARM_OUTPUT_MOTION) && (gate_state == gate_state_e::armed));
	if (!state_allows)
	{
		taskEXIT_CRITICAL();
		return 0U;
	}
	if (InputsStillAuthorize()) return 1U;
	RevokeUnlocked();
	taskEXIT_CRITICAL();
	return 0U;
}

extern "C" void arm_output_gate_end(void)
{
	taskEXIT_CRITICAL();
}

extern "C" void arm_output_gate_revoke(void)
{
	taskENTER_CRITICAL();
	RevokeUnlocked();
	taskEXIT_CRITICAL();
}

extern "C" void arm_output_gate_trip(void)
{
	arm_output_gate_revoke();
}

extern "C" uint8_t arm_output_gate_begin_resume(const uint8_t host_control,
	const uint32_t remote_mode_generation,
	const uint32_t can_tx_failure_count, const uint32_t can_rx_failure_count,
	const uint32_t can_error_event_count)
{
	taskENTER_CRITICAL();
	expected_host_control = (host_control != 0U) ? 1U : 0U;
	expected_remote_mode_generation = remote_mode_generation;
	expected_can_tx_failures = can_tx_failure_count;
	expected_can_rx_failures = can_rx_failure_count;
	expected_can_error_events = can_error_event_count;
	gate_state = gate_state_e::resuming;
	if (InputsStillAuthorize())
	{
		taskEXIT_CRITICAL();
		return 1U;
	}
	RevokeUnlocked();
	taskEXIT_CRITICAL();
	return 0U;
}

extern "C" uint8_t arm_output_gate_activate(void)
{
	taskENTER_CRITICAL();
	if ((gate_state == gate_state_e::resuming) && InputsStillAuthorize())
	{
		gate_state = gate_state_e::armed;
		taskEXIT_CRITICAL();
		return 1U;
	}
	RevokeUnlocked();
	taskEXIT_CRITICAL();
	return 0U;
}

extern "C" uint8_t arm_output_gate_authorization_current(void)
{
	taskENTER_CRITICAL();
	const bool allowed = (gate_state != gate_state_e::disarmed) && InputsStillAuthorize();
	if (!allowed) RevokeUnlocked();
	taskEXIT_CRITICAL();
	return allowed ? 1U : 0U;
}
