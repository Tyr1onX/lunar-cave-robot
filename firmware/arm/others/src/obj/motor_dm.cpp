#include "motor_dm.hpp"
#include <cstring>


using namespace Motor_DM_N;

/* 反馈入口：解析一帧电机反馈（位置 / 速度 / 力矩 / 温度 / 故障状态） */
void Motor_DM::UpdateFeedback(const uint8_t* const data_)
{
	if (__builtin_expect(data_ == nullptr, 0)) return;
	error = static_cast<error_state_e>(data_[0] >> 4);
	pos = (static_cast<float>(data_[1] << 8 | data_[2]) - 32767.f) * facs.bit_to_pos;  // 0 -> 32767(.5) -> 0.f
	vel = (static_cast<float>(data_[3] << 4 | data_[4] >> 4) - 2047.f) * facs.bit_to_vel;
	torque = (static_cast<float>((data_[4] & 0xF) << 8 | data_[5]) - 2047.f) * facs.bit_to_torque;
	temperature_mos = static_cast<int8_t>(data_[6]);
	temperature_rotor = static_cast<int8_t>(data_[7]);
	++rx_cnt;
}

/*
 * MIT 协议命令
 * _pos / _vel / _torque_ff 的取值范围由主机决定
 * _kp 范围 [0, 500]，_kd 范围 [0, 5]（直接打包的原始值，不是物理增益）
 */
void Motor_DM::Ctrl(const float pos_, const float vel_, const uint16_t kp_, const uint16_t kd_,
	const float torque_ff_) const
{
	const auto angle = static_cast<uint16_t>(pos_ * facs.pos_to_bit + 32767.5f);
	const auto speed = static_cast<uint16_t>(vel_ * facs.vel_to_bit + 2047.5f);
	const auto torque_ff = static_cast<uint16_t>(torque_ff_ * facs.torque_to_bit + 2047.5f);
	const uint8_t tx_data[8] {static_cast<uint8_t>(angle >> 8), static_cast<uint8_t>(angle),
        static_cast<uint8_t>(speed >> 4), static_cast<uint8_t>((speed & 0xF) << 4 | (kp_ >> 8 & 0xF)),
        static_cast<uint8_t>(kp_), static_cast<uint8_t>(kd_ >> 4),
        static_cast<uint8_t>((kd_ & 0xF) << 4 | (torque_ff >> 8 & 0xF)), static_cast<uint8_t>(torque_ff)};
	can.Transmit(motor_id, tx_data);
	++tx_cnt;
}

/* 单字节命令（使能 / 失能 / 清错） */
void Motor_DM::Ctrl(const order_byte_e order_) const
{
	const uint8_t tx_data[8] {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, static_cast<uint8_t>(order_)};
	can.Transmit(motor_id, tx_data);
	++tx_cnt;
}
