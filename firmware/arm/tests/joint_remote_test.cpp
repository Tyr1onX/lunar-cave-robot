#include "roboarm.hpp"
#include "Remote.h"
#include <cassert>
#include <cstdio>
#include <cstring>

static uint32_t tick;
static uint8_t* rx_buffer;
static Joint* joints[7];
static float ratios[7];
static unsigned writes;
extern "C"
{
	void (*test_irq_restore_hook)(void) = nullptr;
	UART_HandleTypeDef huart5{UART5, 0};
	DMA_HandleTypeDef hdma_uart5_rx{};
	uint32_t HAL_GetTick(void) { return tick; }
	HAL_StatusTypeDef HAL_UARTEx_ReceiveToIdle_DMA(UART_HandleTypeDef*, uint8_t* p, uint16_t)
	{ rx_buffer = p; return HAL_OK; }
	HAL_StatusTypeDef HAL_UART_AbortReceive(UART_HandleTypeDef*) { return HAL_OK; }
	void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef*, uint16_t);
	void Joint_SetRemoteVelocity(unsigned char idx, float ratio, unsigned char sticks_neutral)
	{
		++writes;
		ratios[idx] = ratio;
		if (joints[idx]) joints[idx]->SetRemoteVelocity(ratio, sticks_neutral != 0U);
	}
}

static void Frame(int s1, int s2, int ch0 = 0, int ch1 = 0, int ch2 = 0, int ch3 = 0)
{
	std::memset(rx_buffer, 0, RC_FRAME_LENGTH);
	const uint16_t values[]{static_cast<uint16_t>(ch0 + 1024), static_cast<uint16_t>(ch1 + 1024),
		static_cast<uint16_t>(ch2 + 1024), static_cast<uint16_t>(ch3 + 1024)};
	for (unsigned channel = 0; channel < 4; ++channel)
		for (unsigned bit = 0; bit < 11; ++bit)
			if (values[channel] & (1U << bit)) rx_buffer[(channel * 11 + bit) / 8] |= 1U << ((channel * 11 + bit) % 8);
	rx_buffer[5] |= (s1 << 4) | (s2 << 6);
	rx_buffer[17] = 4;
	HAL_UARTEx_RxEventCallback(&huart5, RC_FRAME_LENGTH);
}

static void Feedback(Joint& joint, float pos, float vel = 0.f)
{
	joint.Feedback(pos, vel);
	joint.UpdateFeedback(nullptr);
}

static void Cycle(Joint& joint, bool follow = true)
{
	Remote_UpdateJointVelocity();
	const float q = joint.RefPos(), v = joint.RefVel();
	if (follow) Feedback(joint, q, v);
	joint.UpdateReference();
	joint.ControlStep();
	tick += 2;
}

static void ReferenceCycle(Joint& joint, bool follow)
{
	if (follow) Feedback(joint, joint.RefPos(), joint.RefVel());
	joint.UpdateReference();
	joint.ControlStep();
}

static Joint* interrupted_joint;
static void SuspendDuringUpdate()
{
	interrupted_joint->CaptureHoldPosition();
	Feedback(*interrupted_joint, 0.6f);
	interrupted_joint->SoftStart();
}

static void TestPositionMemory(Bsp_CAN& can, const Motor_DM_N::param_factor_s& factors,
	const RoboArm_N::mdh_param_s& mdh, const pid_pos_cfg_s& pid)
{
	Joint held(1, 1, factors, can, 1.5f, {1.f, -1.f}, 0.f, mdh, pid, 600, 4000, 2.f, 1.2f);
	held.SoftStart(); // 首次上线没有旧目标，仍在当前位置保持
	Feedback(held, 0.2f);
	held.UpdateReference();
	const float original = held.HoldPos();
	assert(!held.IsVelocityMode() && Abs(original - 0.2f) < 1.e-6f);
	for (float pushed : {0.85f, -0.85f, 1.05f, -1.05f})
	{
		Feedback(held, pushed);
		held.UpdateReference();
		const float start = Limit(pushed, 0.998f, -0.998f);
		assert(Abs(held.RefPos() - start) < 0.001f); // 从新反馈平滑起段，没有直接跳回原目标
		for (int i = 0; i < 1500; ++i)
		{
			held.SetRemoteVelocity(0.f);
			ReferenceCycle(held, false); // 外力持续按住：允许调整临时参考，但不能丢失目标
			assert(held.HoldPos() == original);
			assert(held.RefPos() <= 0.998f && held.RefPos() >= -0.998f);
			if (pushed > 1.f) assert(held.last_torque <= 0.f && held.last_kd == 0);
			if (pushed < -1.f) assert(held.last_torque >= 0.f && held.last_kd == 0);
		}
		for (int i = 0; i < 2000; ++i) ReferenceCycle(held, true);
		assert(held.HoldPos() == original && Abs(held.RefPos() - original) < 1.e-5f);
	}

	// 手动移动再松杆：保存的是减速结束位置，之后不再被跟踪窗口覆盖。
	for (int i = 0; i < 100; ++i) { held.SetRemoteVelocity(0.5f); ReferenceCycle(held, true); }
	for (int i = 0; i < 400; ++i) { held.SetRemoteVelocity(0.f); ReferenceCycle(held, true); }
	const float released = held.HoldPos();
	assert(!held.IsVelocityMode() && Abs(released - original) > 0.05f);
	Feedback(held, -0.7f);
	ReferenceCycle(held, false);
	assert(held.HoldPos() == released);
	for (int i = 0; i < 2000; ++i) ReferenceCycle(held, true);
	assert(held.HoldPos() == released && Abs(held.RefPos() - released) < 1.e-5f);

	// 静止/自动复位中急停，原保持点不变；多次急停不能把偏移处误记成新原位。
	held.CaptureHoldPosition();
	Feedback(held, -0.6f);
	held.SoftStart();
	for (int i = 0; i < 20; ++i)
	{
		held.SetRemoteVelocity(1.f); // 恢复时仍偏转的摇杆不接管
		ReferenceCycle(held, true);
		assert(!held.IsVelocityMode() && held.HoldPos() == released);
	}
	held.CaptureHoldPosition();
	Feedback(held, -0.4f);
	held.SoftStart();
	for (int i = 0; i < 2000; ++i) { held.SetRemoteVelocity(1.f); ReferenceCycle(held, true); }
	assert(!held.IsVelocityMode() && held.HoldPos() == released && Abs(held.RefPos() - released) < 1.e-5f);
	held.SetRemoteVelocity(0.f);
	held.SetRemoteVelocity(-1.f);
	ReferenceCycle(held, true);
	assert(held.IsVelocityMode() && held.RefVel() < 0.f); // 回中后新的手动输入可接管

	// 运动中急停：回到触发瞬间的实际位置，不重放旧速度/远端位置目标。
	for (bool position_mode : {false, true})
	{
		Joint moving(1, 1, factors, can, 0.f, {1.f, -1.f}, 0.f, mdh, pid, 0, 4000, 0.f, 1.2f);
		Feedback(moving, 0.f);
		moving.UpdateReference();
		if (position_mode) moving.SetTarget(0.8f);
		else moving.SetVelocity(1.2f);
		for (int i = 0; i < 50; ++i) ReferenceCycle(moving, false);
		Feedback(moving, -0.1f);
		moving.CaptureHoldPosition();
		const float captured = moving.HoldPos();
		assert(Abs(captured + 0.1f) < 1.e-6f);
		Feedback(moving, 0.8f);
		moving.SoftStart();
		const int sends = moving.commands;
		moving.ControlStep();
		assert(moving.commands == sends);
		for (int i = 0; i < 2000; ++i) { moving.SetRemoteVelocity(1.f); ReferenceCycle(moving, i > 0); }
		assert(!moving.IsVelocityMode() && moving.HoldPos() == captured && Abs(moving.RefPos() - captured) < 1.e-5f);
	}

	// 挂起发生在控制函数中间：恢复后旧的局部位置请求不得覆盖急停快照。
	Joint interrupted(1, 1, factors, can, 0.f, {1.f, -1.f}, 0.f, mdh, pid, 0, 4000, 0.f, 1.2f);
	Feedback(interrupted, 0.2f);
	interrupted.UpdateReference();
	const float before_suspend = interrupted.HoldPos();
	interrupted.SetTarget(0.7f);
	interrupted_joint = &interrupted;
	test_irq_restore_hook = SuspendDuringUpdate;
	interrupted.UpdateReference();
	assert(test_irq_restore_hook == nullptr && interrupted.HoldPos() == before_suspend);
	const int sends = interrupted.commands;
	interrupted.ControlStep();
	assert(interrupted.commands == sends);
	for (int i = 0; i < 2000; ++i) ReferenceCycle(interrupted, i > 0);
	assert(interrupted.HoldPos() == before_suspend && Abs(interrupted.RefPos() - before_suspend) < 1.e-5f);
	std::puts("persistent hold / displacement recovery / emergency resume tests passed");
}

static void TestLargeJointRecovery(Bsp_CAN& can, const Motor_DM_N::param_factor_s& factors,
	const RoboArm_N::mdh_param_s& mdh)
{
	// j1 的真实限位/PID：反馈连续跨过 ±pi，大惯量轴滞后或暂时被外力按住。
	for (bool resume : {false, true})
		for (bool reverse : {false, true})
		for (bool stall_late : {false, true})
		{
			const float saved = reverse ? 3.f : -1.2f;
			const float displaced = reverse ? -1.3f : 3.25f;
			Joint joint(1, 1, factors, can, 0.f, {Math_N::two_pi - 2.8f, -1.7f},
				-0.14122f, mdh, RoboArm_Pid::j1_left, 600, 4000, 0.f, 1.2f);
			Feedback(joint, WrapRad(saved));
			joint.UpdateReference();
			const float goal = joint.HoldPos();
			if (resume) joint.CaptureHoldPosition();
			for (int i = 1; i <= 100; ++i)
				Feedback(joint, WrapRad(saved + (displaced - saved) * (i / 100.f)));
			if (resume) joint.SoftStart();
			joint.UpdateReference(); // 只有建立回位起点的这一拍允许重新锚定
			joint.ControlStep();
			assert(Abs(joint.RefPos() - displaced) < 0.001f);
			float q = displaced, v = 0.f;
			float max_q_step = 0.f, max_v_step = 0.f, max_command_step = 0.f;
			for (int i = 0; i < 16000; ++i)
			{
				const float old_q = joint.RefPos(), old_v = joint.RefVel(), old_command = joint.last_vel;
				if (stall_late && i < 350) // 另测回位已经加速后突然受阻
				{
					q = old_q;
					v = old_v;
				}
				else if (i >= 2000) // 暂时按住，再以低速度/加速度模拟跟踪能力有限的大关节
				{
					v += Limit(Limit(joint.last_vel, 0.35f) - v, 0.8f * Joint_N::ctrl_dt);
					q += v * Joint_N::ctrl_dt;
				}
				else v = 0.f;
				Feedback(joint, WrapRad(q), v);
				joint.UpdateReference();
				joint.ControlStep();
				max_q_step = fmaxf(max_q_step, Abs(joint.RefPos() - old_q));
				max_v_step = fmaxf(max_v_step, Abs(joint.RefVel() - old_v));
				max_command_step = fmaxf(max_command_step, Abs(joint.last_vel - old_command));
				assert(Abs(joint.HoldPos() - goal) < 1.e-6f);
				assert(joint.RefPos() > -1.7f && joint.RefPos() < Math_N::two_pi - 2.8f);
				assert(Abs(joint.RefAcc()) <= RoboArm_Pid::j1_left.accel_max * Joint_N::plan_accel_scale + 1.e-3f);
				// 调慢轨迹时钟后，输出速度必须仍与位置变化一致。
				assert(Abs(joint.RefPos() - old_q - 0.5f * (old_v + joint.RefVel()) * Joint_N::ctrl_dt) < 1.e-5f);
			}
			std::printf("j1 recovery (resume=%d reverse=%d late stall=%d): max dq=%.6f dv=%.6f command step=%.6f, error=%.6f\n",
				resume, reverse, stall_late, max_q_step, max_v_step, max_command_step, Abs(q - goal));
			std::fflush(stdout);
			assert(max_q_step <= 1.2f * Joint_N::plan_vel_scale * Joint_N::ctrl_dt + 1.e-5f);
			assert(max_v_step <= RoboArm_Pid::j1_left.accel_max * Joint_N::plan_accel_scale * Joint_N::ctrl_dt + 1.e-4f);
			assert(max_command_step <= RoboArm_Pid::j1_left.accel_max * (1.f + Joint_N::plan_accel_scale) * Joint_N::ctrl_dt + 1.e-4f);
			assert(Abs(q - goal) < 0.001f && Abs(joint.RefPos() - goal) < 1.e-5f);
		}

	// 显式定位也需平滑等待；等待期间新目标/手动接管不能继承一个停住的规划时钟。
	Joint commanded(1, 1, factors, can, 0.f, {Math_N::two_pi - 2.8f, -1.7f},
		-0.14122f, mdh, RoboArm_Pid::j1_left, 600, 4000, 0.f, 1.2f);
	Feedback(commanded, 0.f);
	commanded.UpdateReference();
	commanded.SetTarget(2.f);
	for (int i = 0; i < 2000; ++i) ReferenceCycle(commanded, false);
	assert(Abs(commanded.HoldPos() - 2.f) < 1.e-5f && Abs(commanded.RefVel()) < 1.e-4f);
	assert(commanded.RefPos() > 0.3f && commanded.RefPos() < 0.5f);
	commanded.SetTarget(-0.5f);
	for (int i = 0; i < 3000; ++i) ReferenceCycle(commanded, true);
	assert(Abs(commanded.RefPos() + 0.5f) < 1.e-5f);
	commanded.SetRemoteVelocity(1.f);
	ReferenceCycle(commanded, true);
	assert(commanded.IsVelocityMode() && commanded.RefVel() > 0.f);
	std::puts("large-joint lag / stall / wrapped-feedback recovery continuity tests passed");
}

int main()
{
	Bsp_CAN can;
	const Motor_DM_N::param_factor_s factors{100.f};
	const RoboArm_N::mdh_param_s mdh{0.f, 0.f, 0.f, 0.f, {0.f, 0.f, 1.f}};
	const pid_pos_cfg_s pid{20.f, 0.f, 0.f, 0.002f, 0.f, 0.f, 0.f, 0.f, 4.f, 16.f};
	Joint joint(1, 1, factors, can, 1.5f, {1.f, -1.f}, 0.f, mdh, pid, 0, 1000, 0.f, 1.2f);
	joints[0] = &joint;
	Remote_Init();
	assert(Remote_IsOnline() == 0 && Remote_IsLost() == 1 && Remote_IsHoldRequested() == 1);
	Feedback(joint, 0.1f);
	joint.UpdateReference();
	assert(Abs(joint.RefPos() - 0.1f) < 1.e-6f && joint.RefVel() == 0.f); // 不追赶构造时的旧位置目标
	assert(Remote_NormalizeChannel(13) == 0.f && Remote_NormalizeChannel(-13) == 0.f);
	assert(Remote_NormalizeChannel(14) > 0.f && Remote_NormalizeChannel(14) < 0.002f);
	assert(Remote_NormalizeChannel(900) == 1.f && Remote_NormalizeChannel(-900) == -1.f);

	Frame(RC_SW_MID, RC_SW_UP, 660, -660, 330, -330);
	writes = 0;
	Remote_UpdateJointVelocity();
	assert(writes == 7 && ratios[0] == 1.f && ratios[1] == -1.f);
	assert(ratios[2] < 0.f && ratios[3] > 0.f && ratios[4] == 0.f && ratios[6] == 0.f);
	for (int i = 0; i < 150; ++i) { Frame(RC_SW_MID, RC_SW_UP, 660); Cycle(joint); }
	assert(joint.IsVelocityMode() && Abs(joint.RefVel() - 1.2f) < 1.e-5f);

	// 切到腕部时原组显式收零，不能把上一组的速度留在关节里。
	Frame(RC_SW_UP, RC_SW_UP, 660);
	Remote_UpdateJointVelocity();
	assert(ratios[0] == 0.f && ratios[4] == 1.f);
	for (int i = 0; i < 250; ++i) { Frame(RC_SW_UP, RC_SW_UP, 660); Cycle(joint); }
	assert(joint.RefVel() == 0.f);
	const float stopped = joint.RefPos();
	for (int i = 0; i < 50; ++i) Cycle(joint);
	assert(joint.RefPos() == stopped);

	// 零遥控不会覆盖 SetTarget；保留位置接口，准确到达后停住。
	joint.SetTarget(-0.4f);
	for (int i = 0; i < 1500; ++i) { Frame(RC_SW_MID, RC_SW_UP); Cycle(joint); }
	assert(!joint.IsVelocityMode() && Abs(joint.RefPos() + 0.4f) < 1.e-5f && joint.RefVel() == 0.f);
	joint.SetTarget(2.f);
	for (int i = 0; i < 1500; ++i) Cycle(joint);
	assert(Abs(joint.RefPos() - (1.f - Joint_N::limit_margin)) < 1.e-5f); // 两模式统一限位裕量

	// 非零遥控主动接管；S2 中位仍逐轴提交零并减速。
	for (int i = 0; i < 100; ++i) { Frame(RC_SW_MID, RC_SW_UP, -660); Cycle(joint); }
	assert(joint.IsVelocityMode() && joint.RefVel() < -1.f);
	for (int i = 0; i < 300; ++i) { Frame(RC_SW_MID, RC_SW_MID, -660); Cycle(joint); }
	assert(joint.RefVel() == 0.f);

	// 100ms 失联开始减速，3s 挂起判据和双拨杆急停仍保留。
	for (int i = 0; i < 100; ++i) { Frame(RC_SW_MID, RC_SW_UP, 660); Cycle(joint); }
	tick = rc_ctrl.update_tick + 101;
	Remote_UpdateJointVelocity();
	assert(ratios[0] == 0.f && Remote_IsHoldRequested() == 0);
	for (int i = 0; i < 300; ++i) Cycle(joint);
	assert(joint.RefVel() == 0.f);
	tick = rc_ctrl.update_tick + 3001;
	assert(Remote_IsHoldRequested() == 1);
	Frame(RC_SW_DOWN, RC_SW_DOWN);
	assert(Remote_IsHoldRequested() == 1);

	// 恢复保留旧保持点，但规划起点从反馈重建；未消费的旧请求不能覆盖保存位置。
	const float resume_target = joint.HoldPos();
	joint.SetTarget(0.8f);
	joint.CaptureHoldPosition();
	Feedback(joint, -0.2f);
	joint.SoftStart();
	const int sends = joint.commands;
	joint.ControlStep();
	assert(joint.commands == sends);
	joint.UpdateReference();
	assert(!joint.IsVelocityMode() && Abs(joint.RefPos() + 0.2f) < 0.001f && joint.HoldPos() == resume_target);
	for (int i = 0; i < 1500; ++i) { Frame(RC_SW_MID, RC_SW_UP); Cycle(joint); }
	assert(Abs(joint.RefPos() - resume_target) < 1.e-5f);

	// 冻结、未选中通道与超时产生的零量不能冒充恢复后的真实回中。
	joint.CaptureHoldPosition();
	Feedback(joint, 0.f);
	joint.SoftStart();
	Frame(RC_SW_MID, RC_SW_MID, 660);
	Cycle(joint, false);
	Frame(RC_SW_UP, RC_SW_UP, 660); // 当前轴未选中，实际摇杆仍偏转
	Cycle(joint);
	tick = rc_ctrl.update_tick + 101;
	Cycle(joint);
	for (int i = 0; i < 20; ++i) { Frame(RC_SW_MID, RC_SW_UP, 660); Cycle(joint); }
	assert(!joint.IsVelocityMode() && joint.HoldPos() == resume_target);
	Frame(RC_SW_MID, RC_SW_UP);
	Cycle(joint);
	Frame(RC_SW_MID, RC_SW_UP, 660);
	Cycle(joint);
	assert(joint.IsVelocityMode());

	// 编码器反馈完全静止也不无限积累速度积分。
	Feedback(joint, -0.2f);
	for (int i = 0; i < 10000; ++i) { Frame(RC_SW_MID, RC_SW_UP, 660); Cycle(joint, false); }
	assert(joint.RefPos() <= -0.2f + Joint_N::tracking_window + 1.e-5f && Abs(joint.RefVel()) < 0.001f);

	// 运动中速度→位置→速度切换：起点不跳回反馈，速度连续；接管后的加速度受 jerk 约束。
	for (int i = 0; i < 80; ++i) { Frame(RC_SW_MID, RC_SW_UP, -660); Cycle(joint); }
	joint.SetTarget(-0.7f);
	Frame(RC_SW_MID, RC_SW_UP);
	const float before_q = joint.RefPos(), before_v = joint.RefVel();
	Cycle(joint);
	assert(!joint.IsVelocityMode());
	assert(Abs(joint.RefPos() - before_q) < 0.005f && Abs(joint.RefVel() - before_v) < 0.025f);
	const float before_a = joint.RefAcc();
	const float position_q = joint.RefPos(), position_v = joint.RefVel();
	Frame(RC_SW_MID, RC_SW_UP, 660);
	Cycle(joint);
	assert(joint.IsVelocityMode());
	assert(Abs(joint.RefPos() - position_q) < 0.005f && Abs(joint.RefVel() - position_v) < 0.025f);
	assert(Abs(joint.RefAcc() - before_a) <= 240.f * 0.002f + 0.001f);

	// 实测触限：覆盖所有力矩来源，包括电机内部 kp/kd，及量化后的力矩方向。
	for (float sign : {-1.f, 1.f})
	{
		Joint guarded(1, 1, factors, can, 0.f, {1.f, -1.f}, 0.f, mdh, pid, 600, 4000, 2.f, 1.2f);
		Feedback(guarded, sign * 1.01f); // 已经越界，上电也必须立即采用参考硬限位
		guarded.UpdateReference();
		const float boundary = sign * (1.f - Joint_N::limit_margin);
		assert(Abs(guarded.RefPos() - boundary) < 1.e-6f);
		for (int i = 0; i < 10000; ++i)
		{
			guarded.SetRemoteVelocity(sign);
			guarded.UpdateReference();
			guarded.tg = sign * 5.f;
			guarded.td = sign * 5.f;
			guarded.ControlStep();
			assert(guarded.RefPos() == boundary && guarded.RefVel() == 0.f && guarded.RefAcc() == 0.f);
			assert(sign * guarded.last_vel <= 0.f && sign * guarded.last_torque <= 0.f);
			assert(guarded.last_kp == 0 && guarded.last_kd == 0);
		}
		// 压杆 20s 后反拨，首拍即形成内向参考和内向力矩，无位置/纠偏历史欠账。
		guarded.SetRemoteVelocity(-sign);
		guarded.UpdateReference();
		guarded.ControlStep();
		assert(sign * guarded.RefVel() < 0.f && sign * guarded.RefPos() < sign * boundary);
		assert(sign * guarded.last_vel < 0.f && sign * guarded.last_torque < 0.f);

		// 内向/外向实测运动、不同前馈符号：发送命令不产生外向合力矩。
		for (float measured_vel : {-2.f, 0.f, 2.f})
			for (float feedforward : {-10.f, 0.f, 10.f})
			{
				Feedback(guarded, sign * 1.01f, measured_vel);
				guarded.tg = feedforward;
				guarded.td = feedforward;
				guarded.ControlStep();
				assert(guarded.last_kp == 0 && guarded.last_kd == 0);
				assert(sign * guarded.last_torque <= 0.f);
				const auto encoded = static_cast<uint16_t>(guarded.last_torque * factors.torque_to_bit + 2047.5f);
				assert(sign > 0.f ? encoded <= 2047U : encoded >= 2048U);
			}
		// 滞回只限制外向动作，向内离开超过滞回后恢复原 MIT 增益。
		Feedback(guarded, sign * (1.f - Joint_N::limit_margin - 0.0004f));
		guarded.ControlStep();
		assert(guarded.last_kp == 0 && guarded.last_kd == 0);
		Feedback(guarded, sign * (1.f - Joint_N::limit_margin - 0.002f));
		guarded.ControlStep();
		assert(guarded.last_kp == 600 && guarded.last_kd == 4000);

		// 反馈在规划之后才触限，输出阶段也能拦截，且不破坏向内绝对定位入口。
		Feedback(guarded, 0.f);
		guarded.SoftStart();
		guarded.UpdateReference();
		guarded.SetTarget(sign * 2.f);
		for (int i = 0; i < 30; ++i) { guarded.UpdateReference(); guarded.ControlStep(); }
		Feedback(guarded, sign * 1.01f, sign);
		guarded.tg = sign * 10.f;
		guarded.ControlStep();
		assert(sign * guarded.last_torque <= 0.f && guarded.last_kd == 0);
		guarded.SetTarget(0.f);
		guarded.UpdateReference();
		assert(!guarded.IsVelocityMode() && sign * guarded.RefVel() < 0.f);
	}
	std::puts("joint / remote integration and directional limit tests passed");
	TestPositionMemory(can, factors, mdh, pid);
	TestLargeJointRecovery(can, factors, mdh);
}
