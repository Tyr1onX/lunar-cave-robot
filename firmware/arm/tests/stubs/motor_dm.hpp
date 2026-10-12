#pragma once
#include "main.h"

struct Bsp_CAN {};
namespace Motor_DM_N
{
	enum class order_byte_e : uint8_t { enable, disable, clear_error };
	struct param_factor_s { float torque_to_bit; };
}

/* 仅替换硬件边界；测试编译真正的 Joint、Remote.c 和两种规划器。 */
class Motor_DM
{
public:
	Motor_DM(uint8_t, uint8_t, const Motor_DM_N::param_factor_s& f, Bsp_CAN&) : facs(f) {}
	void UpdateFeedback(const uint8_t*) {}
	void Ctrl(Motor_DM_N::order_byte_e) const {}
	void Ctrl(float p, float v, uint16_t kp, uint16_t kd, float t) const
	{
		last_pos = p; last_vel = v; last_torque = t; last_kp = kp; last_kd = kd; ++commands;
	}
	void Feedback(float p, float v = 0.f) { pos = p; vel = v; }
	mutable float last_pos{0.f}, last_vel{0.f}, last_torque{0.f};
	mutable int commands{0};
	mutable uint16_t last_kp{0}, last_kd{0};
protected:
	float pos{0.f}, vel{0.f};
	const Motor_DM_N::param_factor_s& facs;
};
