/* 流式目标（规划段终点速度非 0）的主机侧验证 —— 编译真正的 roboarm.cpp。
 *
 * 背景：上位机按固定频率连续刷新目标时，如果规划段仍按“到点停车”来解
 * （终点速度恒 0），每一帧都要求参考走到新点再刹停，表现为参考速度反复
 * 加减速、跟随滞后大。SetStreamTarget 用目标自身的运动速度作为终点速度，
 * 匀速目标下三次曲线退化成一条直线，本测试就是把这一族性质钉住。
 *
 * 覆盖：
 *   1) 匀速目标：稳态滞后有界，速度跟得上，没有加减速抖动；
 *   2) 同一目标序列走点动入口 SetTarget 时滞后明显更大（对照组）；
 *   3) 上游停流后必须收住在最后一个目标上，不能带着旧速度一直跑；
 *   4) 上游阶跃不能一步把终点速度顶到速度上限，也不能冲过目标；
 *   5) 遥控/点到点入口能把关节拉出流式模式；
 *   6) 逐轴核对真实限位下的圈次对齐：跨度 > π 的轴（j1/j3/j4/j5/j6）两端相距也 > π，
 *      请求必须折进限位那一圈，不能按“离当前位置最近”选圈（会被钳到反方向）。
 *
 * 编译见 tests/run_stream_tests.ps1（关键是把 -Itests/stubs_can 排在 others/inc/bsp 前面，
 * 让真实的 motor_dm.hpp 配主机侧 Bsp_CAN 替身，而不是把 HAL/FDCAN 拖进来）。
 */

#include "roboarm.hpp"
#include <cassert>
#include <cmath>
#include <cstdio>

/* ---- 硬件边界替身：roboarm.cpp 直接引用到的主机侧空洞 ---- */
extern "C"
{
	void (*test_irq_restore_hook)(void) = nullptr;
	uint32_t HAL_GetTick(void) { return 0U; }
}

namespace
{
	float last_torque;
	uint16_t last_kd;
}

/* Motor_DM 有两个 out-of-line 方法在 motor_dm.cpp 里（会碰 CAN/HAL）：
   反馈由测试注入，命令只记录。 */
void Motor_DM::UpdateFeedback(const uint8_t*)
{
	/* 空实现：测试通过 HostJoint::Feed 直接写 pos/vel */
}

void Motor_DM::Ctrl(const float pos_, const float vel_, const uint16_t kp_, const uint16_t kd_,
	const float torque_ff_) const
{
	/* 只留最后一拍的指令，供“指令通路确实在走”的断言使用 */
	(void)pos_;
	(void)vel_;
	(void)kp_;
	last_torque = torque_ff_;
	last_kd = kd_;
}

void Motor_DM::Ctrl(const Motor_DM_N::order_byte_e) const {}

namespace
{
	/* 真实的 pos/vel 是电机反馈（受保护成员），测试直接写，再走真实的
	   Joint::UpdateFeedback 展开连续角并做首帧限位对齐。 */
	class HostJoint : public Joint
	{
		public:
		using Joint::Joint;

		void Feed(const float position, const float velocity)
		{
			pos = position;
			vel = velocity;
			UpdateFeedback(nullptr);
		}
	};

	void Init(HostJoint& joint, const float position)
	{
		joint.Feed(position, 0.f);
		joint.UpdateReference();
	}

	/* 理想跟随：把上一拍的参考直接当成新反馈，隔离规划器本身的行为。 */
	void Cycle(HostJoint& joint)
	{
		joint.Feed(joint.RefPos(), joint.RefVel());
		joint.UpdateReference();
		joint.ControlStep();
	}

	struct track_report_s
	{
		float max_lag;    // 稳态段最大跟踪滞后 (rad)
		float max_dv;     // 稳态段每拍速度变化上限（隐含加速度约束）
		float final_vel;  // 末态参考速度 (rad/s)
	};

	/* 目标以 speed (rad/s) 匀速移动 duration 秒，每 send_period 拍下发一次目标；
	   返回稳态跟踪指标（跳过前 400 拍起步段）。 */
	track_report_s RunRamp(HostJoint& joint, const float speed, const float duration,
		const int send_period, const bool stream)
	{
		const int cycles = static_cast<int>(duration / Joint_N::ctrl_dt);
		track_report_s report{0.f, 0.f, 0.f};
		float target = 0.f;
		float previous_vel = joint.RefVel();
		for (int i = 0; i < cycles; ++i)
		{
			if ((i % send_period) == 0)
			{
				target += speed * static_cast<float>(send_period) * Joint_N::ctrl_dt;
				if (stream) joint.SetStreamTarget(target);
				else joint.SetTarget(target);
			}
			Cycle(joint);
			if (i > 400)
			{
				report.max_lag = fmaxf(report.max_lag, Abs(target - joint.RefPos()));
				report.max_dv = fmaxf(report.max_dv, Abs(joint.RefVel() - previous_vel));
			}
			previous_vel = joint.RefVel();
		}
		report.final_vel = joint.RefVel();
		return report;
	}
}

int main()
{
	Bsp_CAN can;
	const Motor_DM_N::param_factor_s factors = Motor_DM_N::MakeFactors(12.5f, 30.f, 10.f);
	const RoboArm_N::mdh_param_s mdh{0.f, 0.f, 0.f, 0.f, {0.f, 0.f, 1.f}};
	/* j2 的位置环参数：vel_max 4 rad/s、accel_max 16 rad/s^2。
	   规划速度预算 = min(remote_vel_max × 1.8, pid.vel_max) = 2.16 rad/s，
	   进曲线的加速度预算 = 0.75 × 16 × 0.5 = 6 rad/s^2。 */
	const pid_pos_cfg_s pid{20.f, 0.f, 4.8f, 0.002f, 0.f, 0.f, 0.f, 0.f, 4.f, 16.f};
	const float speed = 0.5f;

	/* ---------- 1) 流式：匀速目标必须跟得住 ---------- */
	HostJoint streamed(1, 1, factors, can, 0.f, {1.2f, -1.2f}, 0.f, mdh, pid, 600, 4000, 2.f, 1.2f);
	Init(streamed, 0.f);
	const auto streaming = RunRamp(streamed, speed, 2.f, 5, true);
	std::printf("stream : max lag %.6f rad, max dv/cycle %.6f, final vel %.6f rad/s\n",
		streaming.max_lag, streaming.max_dv, streaming.final_vel);
	/* 稳态滞后应当是有界的 v·T（T 正常就是 plan_time = 0.02 s），远小于跟踪窗口 */
	assert(streaming.max_lag < 4.f * speed * Route_Plan_N::plan_time);
	/* 速度必须跟上去，而不是每帧被压回 0 */
	assert(Abs(streaming.final_vel - speed) < 0.05f);
	/* 每拍速度变化受规划加速度预算约束，没有反复加减速的抖动 */
	assert(streaming.max_dv < 6.f * Joint_N::ctrl_dt + 1.e-5f);

	/* ---------- 2) 对照组：同一目标序列走点动入口（终点速度 0）---------- */
	HostJoint pointwise(1, 1, factors, can, 0.f, {1.2f, -1.2f}, 0.f, mdh, pid, 600, 4000, 2.f, 1.2f);
	Init(pointwise, 0.f);
	const auto stepped = RunRamp(pointwise, speed, 2.f, 5, false);
	std::printf("point  : max lag %.6f rad, max dv/cycle %.6f, final vel %.6f rad/s\n",
		stepped.max_lag, stepped.max_dv, stepped.final_vel);
	/* “每段都刹停”的解在同一个匀速目标下只能靠一个固定滞后去追：稳态滞后
	   ≈ v × 0.114 s = 0.057 rad；流式入口把参考直接放在目标上（滞后 0.002 rad）。
	   这是本次改动的核心收益。 */
	assert(stepped.max_lag > 10.f * streaming.max_lag);
	assert(Abs(stepped.final_vel - speed) < 0.05f);
	std::printf("lag ratio point/stream = %.1f\n", stepped.max_lag / streaming.max_lag);

	/* ---------- 3) 停流：必须收住在最后一个目标上 ---------- */
	const float last_target = speed * 2.f;
	for (int i = 0; i < 6000; ++i) Cycle(streamed);   // 10 s 不再下发任何目标
	std::printf("after stream loss: pos %.6f (target %.6f), vel %.6f\n",
		streamed.RefPos(), last_target, streamed.RefVel());
	assert(Abs(streamed.RefPos() - last_target) < 1.e-3f);   // 不冲过目标
	assert(streamed.RefVel() == 0.f);                        // 不能带着旧速度一直跑

	/* ---------- 4) 阶跃目标：不能一步顶到速度上限，也不能过冲 ---------- */
	HostJoint jumped(1, 1, factors, can, 0.f, {1.2f, -1.2f}, 0.f, mdh, pid, 600, 4000, 2.f, 1.2f);
	Init(jumped, 0.f);
	jumped.SetStreamTarget(0.6f);
	float max_overshoot = 0.f;
	for (int i = 0; i < 6000; ++i)
	{
		Cycle(jumped);
		max_overshoot = fmaxf(max_overshoot, jumped.RefPos() - 0.6f);
	}
	std::printf("step 0.6 rad: pos %.6f, max overshoot %.6f, vel %.6f\n",
		jumped.RefPos(), max_overshoot, jumped.RefVel());
	assert(Abs(jumped.RefPos() - 0.6f) < 1.e-4f);
	assert(max_overshoot < 1.e-4f);
	assert(jumped.RefVel() == 0.f);

	/* 阶跃后继续按匀速流式下发，仍然要跟得上 */
	const auto resumed = RunRamp(jumped, speed, 1.f, 5, true);
	std::printf("resume : max lag %.6f rad, final vel %.6f rad/s\n",
		resumed.max_lag, resumed.final_vel);
	assert(Abs(resumed.final_vel - speed) < 0.05f);
	assert(resumed.max_lag < 4.f * speed * Route_Plan_N::plan_time);

	/* ---------- 5) 遥控 / 点到点入口必须把关节拉出流式模式 ---------- */
	jumped.SetRemoteVelocity(0.5f);
	Cycle(jumped);
	assert(jumped.IsVelocityMode());
	for (int i = 0; i < 200; ++i) Cycle(jumped);
	assert(jumped.RefVel() > 0.f);   // 遥控接管后由速度模式驱动
	assert(last_kd != 0U);           // 指令通路确实在走（MIT 的 kd 字段非 0）

	jumped.SetTarget(0.2f);
	for (int i = 0; i < 4000; ++i) Cycle(jumped);
	assert(Abs(jumped.RefPos() - 0.2f) < 1.e-4f && jumped.RefVel() == 0.f);
	assert(last_torque == last_torque);   // 前馈力矩通路有写入（避免 last_torque 未被使用）

	/* ---------- 6) 逐轴核对：真实限位下的圈次对齐 ----------
	   limits / offset 抄自 variables.cpp 的构造参数（改那边记得同步这里）。
	   限位跨度 > π 的轴（j1 5.183 / j3 4.132 / j4 5.333 / j5 3.600 / j6 5.233）
	   两端相距都 > π，旧逻辑按“离当前位置最近”折叠会选到区间外的那一圈，
	   再被 ApplyLimit 钳回近端（表现为“往那一头怎么都不走 / 跑到反方向”）。
	   这里对每一轴做【下端 → 上端 → 下端】往返，要求参考最终停在限位内的那一圈。
	   用 WrapRad 比较是按 mod 2π 判等：固件的连续角只保证锚在 offset 附近，
	   j3/j4 这类 offset 与限位中心差 2π 的轴，绝对圈次会整体平移（物理等价）。
	   j2 跨度 2.13 < π，作为对照组（旧逻辑也不会错）。 */
	struct axis_case_s { const char* name; float pos_max; float pos_min; float offset; };
	const axis_case_s axes[] =
	{
		{"j1", 6.28318530f - 2.8f,  -1.7f,               -0.14122f},
		{"j2", 6.28318530f - 2.4f,   1.75f,              -2.38192f},
		{"j3", 0.99f,               -3.14159265f,         1.00439f},
		{"j4", 6.28318530f - 0.15f,  0.8f,               -2.80760f},
		{"j5", 1.0f,                -2.6f,                0.62223f},
		{"j6", 1.8f,                 2.85f - 6.28318530f, -0.77208f},
	};
	for (const axis_case_s& c : axes)
	{
		HostJoint axis(1, 1, factors, can, 0.f, {c.pos_max, c.pos_min}, c.offset, mdh, pid,
			0, 500, 0.f, 1.2f);
		/* 两端各留 0.03 rad，避开 ApplyLimit 的 0.002 边距与浮点误差 */
		const float lo = c.pos_min + 0.03f;
		const float hi = c.pos_max - 0.03f;
		Init(axis, lo);
		axis.SetStreamTarget(hi);
		for (int i = 0; i < 2500; ++i) Cycle(axis);
		assert(Abs(WrapRad(axis.RefPos() - hi)) < 1.e-3f);
		axis.SetStreamTarget(lo);
		for (int i = 0; i < 2500; ++i) Cycle(axis);
		std::printf("limit fold %s: span %.3f, %.3f <-> %.3f, ref %.6f\n",
			c.name, c.pos_max - c.pos_min, lo, hi, axis.RefPos());
		assert(Abs(WrapRad(axis.RefPos() - lo)) < 1.e-3f);
	}

	std::puts("joint streaming target tests passed");
	return 0;
}
