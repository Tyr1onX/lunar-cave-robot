#pragma once

#include "mathetics.hpp"
#include <array>


struct pid_cfg_s
{
	const float _kp;
	const float _ki;
	const float _kd;
	const float _deadzone;
	const float _integral_limit;
	const float _output_limit;
};

//template <typename T>
//concept pid = (std::constructible_from<T, const pid_cfg_t&>
//	|| std::constructible_from<T, const pid_cfg_t&, const pid_cfg_t&>)
//	&& requires(T t)
//	{
//		t.output;
//		t.Update(float{}, float{});
//		t.Clear();
//	};

struct pid_s
{
	float kp;
	float ki;
	float kd;
	float deadzone;
	float error;
	float error_prev;
	float integral_limit;
	float integral;
	float output_limit;
	float output;
	explicit pid_s(const pid_cfg_s& _pid_cfg)
		: kp(_pid_cfg._kp), ki(_pid_cfg._ki), kd(_pid_cfg._kd), deadzone(_pid_cfg._deadzone), error(0), error_prev(0),
		integral_limit(_pid_cfg._integral_limit), integral(0), output_limit(_pid_cfg._output_limit), output(0)
	{}
	void Update(const float _target, const float _feedback)
	{
		error_prev = error;
		error = _target - _feedback;
		if (error < deadzone && error > -deadzone) return;
		integral = Limit(integral + error, integral_limit, -integral_limit);
		output = (error - error_prev) * kd + integral * ki + error * kp;
		output = Limit(output, output_limit, -output_limit);
	}
	void Clear()
	{
		error = 0;
		error_prev = 0;
		integral = 0;
		output = 0;
	}
};

struct pid_diff_ahead_s
{
	float kp;
	float ki;
	float kd;
	float deadzone;
	float error;
	float feedback_prev;
	float integral_limit;
	float integral;
	float output_limit;
	float output;
	explicit pid_diff_ahead_s(const pid_cfg_s& _pid_cfg)
		: kp(_pid_cfg._kp), ki(_pid_cfg._ki), kd(_pid_cfg._kd), deadzone(_pid_cfg._deadzone), error(0), feedback_prev(0),
		integral_limit(_pid_cfg._integral_limit), integral(0), output_limit(_pid_cfg._output_limit), output(0)
	{}
	void Update(const float _target, const float _feedback)
	{
		error = _target - _feedback;
		if (error < deadzone && error > -deadzone)
		{
			feedback_prev = _feedback;  // doubt its necessity
			return;
		}
		const float derivative = (_feedback - feedback_prev) * kd;
		feedback_prev = _feedback;
		integral = Limit(integral + error, integral_limit, -integral_limit);
		output = Limit(error * kp + integral * ki - derivative, output_limit, -output_limit);
	}
	void Clear()
	{
		error = 0;
		feedback_prev = 0;
		integral = 0;
		output = 0;
	}
};

/* ============ 位置环 PID（输出速度指令 rad/s）============
 * 统一管理关节位置环的 PID 参数与计算：ki 置 0 即纯 PD。
 * 各关节的 PID 参数在下方 RoboArm_Pid 命名空间集中定义，
 * 业务代码（variables.cpp）只引用配置常量，不再散落初始化列表。
 *
 * 积分：**当前弃用**（2026-09-25 决定）。所有轴 ki = 0，且 int_max / sep_pos / sep_vel
 * 一并留 0 —— 条件积分在 sep_pos = 0 时永不成立，积分恒为 0，等价纯 PD。
 * 背景：静止保持只有 P 通路时，静差必须靠一个位置误差来维持力矩，这是设计上的取舍；
 * 期间试过用积分把静差吃掉，但静止时出现周期性力矩上下变动，因此先退回纯 PD。
 * 静止偏差改走 kp/kd（速度通道）与驱动器 MIT kp（kp_pos 字段）。
 * 机制与踩坑记录保留在 Update() 与 integral_leak 的注释里，以后要重开必须先读：
 * 抗饱和只能用"冻结 + 泄漏"，**不能**用"窗口外清零"（会在判据边界上两态切换，正是那次
 * 周期性力矩波动的来源）；且积分带宽低（约 1/T_i），对冲击的**瞬态**偏移没有帮助。
 */
struct pid_pos_cfg_s
{
	const float kp;        // 位置环比例增益
	const float ki;        // 位置环积分增益（置 0 即纯 PD）
	const float kd;        // 位置环速度阻尼增益（由 rad/s 反馈产生 rad/s 修正）
	const float dt;        // 控制周期（s）
	const float deadzone;  // 位置误差死区（rad）
	const float int_max;   // 积分限幅（rad·s）
	const float sep_pos;   // 积分分离：位置误差窗口（rad）
	const float sep_vel;   // 积分分离：速度窗口（rad/s）
	const float vel_max;   // 输出侧速度限幅（rad/s）
	const float accel_max; // output acceleration limit (rad/s^2)
};

struct pid_pos_s
{
	/* 注意：积分当前已弃用（所有轴 ki = 0），本常量与下面的条件积分只在重新启用时生效。
	   积分"冻结"时的泄漏率 (1/s)：时间常数 0.5 s。冻结（而不是清零）是为了避免
	   在判据边界上反复"累积→清零→再累积"形成两态切换；加泄漏是为了让冻结不会把旧
	   积分长期挂账（运动段、饱和期间积分缓慢归零）。 */
	static constexpr float integral_leak = 2.f;

	explicit pid_pos_s(const pid_pos_cfg_s& _cfg)
		: cfg(_cfg), err(0.f), err_prev(0.f), integral(0.f), output(0.f), output_prev(0.f)
	{}
	float Update(const float target, const float feedback, const float feedback_vel)
	{
		err = target - feedback;
		if (err < cfg.deadzone && err > -cfg.deadzone)
		{
			err_prev = err;
			integral = 0.f;  // 死区内视为不需要纠偏，积分一并清掉，避免离开死区时带着旧积分冲出去
			output = 0.f;
			output_prev = output;
			return output;
		}
		const float d_feedback = feedback_vel;  // measured velocity (rad/s)
		err_prev = err;
		const float pd_raw = cfg.kp * err - cfg.kd * d_feedback;
		/* ⚠️ 积分当前已弃用（所有轴 ki = 0、sep_pos = 0）：下面这段恒不累积、
		 * integral 恒为 0，输出就是 pd_raw。保留代码是为了以后能直接重开。
		 *
		 * 条件积分（抗饱和），三种状态：
		 *   ① |err| ≥ sep_pos：偏差已大到该由规划/限位介入（冲击、轨迹段、未到位），
		 *      积分不参与，**清零**；
		 *   ② 近静止且 P/D 未顶到 vel_max：正常**累积**，这是吃静差的正常工作区；
		 *   ③ 其余（运动段 |v| ≥ sep_vel、或输出已饱和）：**冻结 + 缓慢泄漏**。
		 * 注意③不能写成清零：清零型条件积分在边界上会反复"累积→清零→再累积"，
		 * 表现为静止时力矩周期性上下变动（积分把关节推过速度窗口 → 积分归零 → 关节
		 * 退回 → 再累积）；冻结保留状态，泄漏保证不长期挂账，也不会在退饱和时过冲。
		 * 运动段（含遥控把定滞后、轨迹跟随误差）仍交给 P 路径，积分不参与，
		 * 否则会吃掉「规划速度与 vel_max 之间」留给纠偏的余量。 */
		if (Abs(err) >= cfg.sep_pos)
			integral = 0.f;
		else if ((Abs(feedback_vel) < cfg.sep_vel) && (Abs(pd_raw) < cfg.vel_max))
			integral = Limit(integral + err * cfg.dt, cfg.int_max);
		else
			integral -= integral * integral_leak * cfg.dt;
		// PID 输出 → 加速度限幅（限制输出变化率）→ 速度限幅
		const float raw = pd_raw + cfg.ki * integral;
		const float a_step = cfg.accel_max * cfg.dt;
		output = Limit(raw, output_prev + a_step, output_prev - a_step);
		output = Limit(output, cfg.vel_max);
		output_prev = output;
		return output;
	}
	void Clear()
	{
		err = 0.f;
		err_prev = 0.f;
		integral = 0.f;
		output = 0.f;
		output_prev = 0.f;
	}
	const pid_pos_cfg_s cfg;
	float err;
	float err_prev;
	float integral;
	float output;
	float output_prev;
};

/* 机械臂各关节位置环 PID 参数（统一在此管理，variables.cpp 只引用） */
namespace RoboArm_Pid
{
/* kp, ki, kd, dt, deadzone, int_max, sep_pos, sep_vel, vel_max, accel_max
	 *
	 * 积分已弃用（2026-09-25）：**全部轴 ki = 0，int_max / sep_pos / sep_vel 也全留 0**，
	 * 条件积分永不成立 → 积分恒为 0，控制器等价纯 PD（字段与代码保留，便于以后重开）。
	 * 原因：静止保持加积分后出现周期性力矩上下变动（见 Update() 里的踩坑记录）；
	 * 静止偏差目前靠 kp/kd（速度通道）与驱动器 MIT kp（kp_pos 字段）处理。
	 * 要重开时的起点参数（j2 曾用过的值）：ki = kp（T_i = 1 s）、int_max 使
	 * ki × int_max ≈ 20~30% vel_max、sep_pos 0.25 rad、sep_vel 0.2 rad/s；
	 * 且抗饱和必须是"冻结 + 泄漏"，不要写成"窗口外清零"。 */
	// kp, ki, kd, dt, deadzone, int_max, sep_pos, sep_vel, vel_max, accel_max
	inline constexpr pid_pos_cfg_s j1_left{13.f, 0.f, 2.f, 0.002f, 
		0.f, 0.f, 0.f, 0.f, 
		8.f, 24.f};
	inline constexpr pid_pos_cfg_s j2_left{20.f, 0.f, 4.8f, 0.002f, 
		0.f, 0.f, 0.f, 0.f, 
		4.f, 16.f};
	inline constexpr pid_pos_cfg_s j3_left{25.f, 0.f, 3.f, 0.002f, 
		0.f, 0.f, 0.f, 0.f, 
		5.f, 20.f};
	inline constexpr pid_pos_cfg_s j4_left{30.f, 0.f, 4.f, 0.002f, 
		0.f, 0.f, 0.f, 0.f, 
		5.f, 20.f};
	inline const pid_pos_cfg_s j5_left{15.f, 0.f, 0.03f, 0.002f, 
		0.f, 0.f, 0.f, 0.f, 
		5.f, 20.f};
	inline constexpr pid_pos_cfg_s j6_left{25.f, 0.f, 3.f, 0.002f, 
		0.f, 0.f, 0.f, 0.f,
		5.f, 20.f};
	inline constexpr pid_pos_cfg_s j7_left{0.f, 0.f, 0.f, 0.002f, 
		0.f, 0.f, 0.f, 0.f, 
		2.f, 6.f};
}
