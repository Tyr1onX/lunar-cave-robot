#pragma once

#include <stdint.h>
#include "main.h"
#include "motor_dm.hpp"

namespace Joint_N
{
inline constexpr uint8_t dof = 7U;
}

class Joint : public Motor_DM
{
public:
	explicit Joint(Bsp_CAN& can)
		: Motor_DM(0x01U, 0x11U, factors, can)
	{
		ResetForTest();
	}

	bool IsInited() const { return inited; }
	float PosUnwrapped() const { return position; }
	void UpdateFeedback(const uint8_t* frame)
	{
		if (frame == nullptr) return;
		Motor_DM::UpdateFeedback(frame);
		inited = true;
		feedback_tick = HAL_GetTick();
	}
	void SetStreamTarget(float target) { last_target = target; ++target_count; }
	bool FeedbackHealthy(uint32_t now, uint32_t stale_ms) const
	{
		return inited && !fault && !HasDriverFault() && (now - feedback_tick <= stale_ms);
	}
	bool FeedbackUpdatedAfter(uint16_t previous) const
	{
		const uint16_t delta = static_cast<uint16_t>(FeedbackCount() - previous);
		return delta != 0U && delta < 0x8000U;
	}
	bool DriverEnabled() const { return DriverStatus() == Motor_DM_N::error_state_e::enabled; }
	bool Ctrl(Motor_DM_N::order_byte_e command)
	{
		if (!Motor_DM::Ctrl(command)) return false;
		switch (command)
		{
		case Motor_DM_N::order_byte_e::clear_error:
			++clear_count;
			break;
		case Motor_DM_N::order_byte_e::enable:
			++enable_count;
			break;
		case Motor_DM_N::order_byte_e::disable:
			++disable_count;
			break;
		case Motor_DM_N::order_byte_e::set_zero:
			break;
		}
		uint8_t response[8U] {};
		if (command == Motor_DM_N::order_byte_e::enable)
			response[0] = static_cast<uint8_t>(Motor_DM_N::error_state_e::enabled) << 4U;
		UpdateFeedback(response);
		return true;
	}
	void ResetForTest()
	{
		inited = true;
		fault = false;
		feedback_tick = HAL_GetTick();
		position = 0.25f;
		last_target = 0.f;
		target_count = 0U;
		enable_count = 0U;
		clear_count = 0U;
		disable_count = 0U;
		capture_count = 0U;
		soft_start_count = 0U;
		rx_cnt = 1U;
		error = Motor_DM_N::error_state_e::disabled;
	}
	void CaptureHoldPosition() { ++capture_count; }
	void SoftStart() { ++soft_start_count; }

	bool inited{true};
	bool fault{false};
	uint32_t feedback_tick{0U};
	float position{0.25f};
	float last_target{0.f};
	uint32_t target_count{0U};
	uint32_t enable_count{0U};
	uint32_t clear_count{0U};
	uint32_t disable_count{0U};
	uint32_t capture_count{0U};
	uint32_t soft_start_count{0U};

private:
	static constexpr Motor_DM_N::param_factor_s factors =
		Motor_DM_N::MakeFactors(6.28f, 45.f, 54.f);
};

extern Joint j1_left, j2_left, j3_left, j4_left, j5_left, j6_left, j7_left;
void Joint_CaptureHoldAll(void);
void Joint_SoftStartAll(void);
#ifdef __cplusplus
extern "C" {
#endif
void Gripper_Command(uint8_t close_target);
float Gripper_OpenPosition(void);
float Gripper_ClosePosition(void);
#ifdef __cplusplus
}
#endif
