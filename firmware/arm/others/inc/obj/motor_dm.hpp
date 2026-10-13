#pragma once

#include "bsp_can.hpp"


namespace Motor_DM_N
{
	enum class error_state_e : uint8_t
	{
		disabled = 0x00,
		enabled = 0x01,
		over_voltage = 0x08,
		under_voltage = 0x09,
		over_current = 0x0A,
		over_temperature_mos = 0x0B,
		over_temperature_rotor = 0x0C,
		comms_lost = 0x0D,
		overload = 0x0E
	};

	enum class order_byte_e : uint8_t
	{
		clear_error = 0xFB,
		enable = 0xFC,
		disable = 0xFD,
		set_zero = 0xFE
	};

	struct param_factor_s
	{
		const float bit_to_pos;
		const float bit_to_vel;
		const float bit_to_torque;
		const float pos_to_bit;
		const float vel_to_bit;
		const float torque_to_bit;
	};

	/* 由「位置量程、速度量程、力矩量程」生成双向换算系数 */
	constexpr param_factor_s MakeFactors(const float pos_max, const float vel_max, const float torque_max)
	{
		return {pos_max / 32767.5f, vel_max / 2047.5f,
			torque_max / 2047.5f, 32767.5f / pos_max,
			2047.5f / vel_max, 2047.5f / torque_max};
	}
}

class Motor_DM
{
	public:
	explicit Motor_DM(uint8_t host_id_, uint8_t motor_id_, const Motor_DM_N::param_factor_s& facs_, Bsp_CAN& can_)
		: host_id(host_id_), motor_id(motor_id_), error(Motor_DM_N::error_state_e::disabled), pos(0.f), vel(0.f),
		torque(0.f), temperature_mos(0), temperature_rotor(0), facs(facs_), can(can_), tx_cnt(0), rx_cnt(0)
	{}

	/* 反馈入口：CAN 接收回调里调用 */
	void UpdateFeedback(const uint8_t* data_);
	Motor_DM_N::error_state_e DriverStatus() const { return error; }
	bool HasDriverFault() const
	{
		return error != Motor_DM_N::error_state_e::disabled &&
			error != Motor_DM_N::error_state_e::enabled;
	}
	uint16_t FeedbackCount() const { return rx_cnt; }

	/* MIT 命令：kp/kd 是直接打包进 12 位字段的原始值，不是物理增益 */
	bool Ctrl(float pos_, float vel_, uint16_t kp_, uint16_t kd_, float torque_ff_) const;

	/* 单字节命令：使能 / 失能 / 清错 */
	bool Ctrl(Motor_DM_N::order_byte_e order_) const;

	protected:
	const uint8_t host_id;
	const uint8_t motor_id;
	volatile Motor_DM_N::error_state_e error;  // 电机反馈的状态/故障码
	float pos;                        // rad
	float vel;                        // rad/s
	float torque;                     // Nm
	int8_t temperature_mos;
	int8_t temperature_rotor;
	const Motor_DM_N::param_factor_s& facs;
	Bsp_CAN& can;
	mutable uint16_t tx_cnt;  // 在 const 方法里也会自增
	volatile uint16_t rx_cnt;
};
