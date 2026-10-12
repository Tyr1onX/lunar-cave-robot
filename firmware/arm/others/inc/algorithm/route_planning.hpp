#pragma once

#include "mathetics.hpp"

/* ============================ 三次多项式位置规划 ============================
 * 一段轨迹用归一化时间 u = t / T（t ∈ [0, T]）表示：
 *
 *     pos(u) = b0 + b1·u + b2·u² + b3·u³
 *     vel(t) = (b1 + 2·b2·u + 3·b3·u²) / T
 *     acc(t) = (2·b2 + 6·b3·u) / T²
 *
 * 对外接口就是 5 个边界条件：
 *
 *     目标位置 pos_target、完成时间 T、目标速度 vel_target、
 *     起始位置 pos_start、起始速度 vel_start
 *
 * 由它们解出系数：
 *
 *     b0 = pos_start
 *     b1 = T·vel_start
 *     b2 = 3(pos_target - pos_start) - T(2·vel_start + vel_target)
 *     b3 = -2(pos_target - pos_start) + T(vel_start + vel_target)
 *
 * 使用注意（都很容易踩）：
 *  1. 三次多项式只保证位置、速度连续（C¹）；重规划交界处加速度可能跳变。
 *     需要加速度连续（C²）时应换成一般边界五次多项式。
 *  2. 起止速度为零时曲线单调：峰值速度 = 1.5|Δq|/T（在中点），峰值加速度 = 6|Δq|/T²（在两端）。
 *     T 不是「随便给」的，它直接决定峰值速度和峰值加速度 —— 小 T 等于大加速度。
 *  3. 起始速度与位移方向相反（换向）时，段内可能超调，近限位或长距离运动必须检查极值。
 *     本文件的 CubicPeaks() 用闭式解给出精确峰值，不要靠采样去「看着差不多」。
 *  4. 重规划要「先采样旧轨迹取当前状态 (q, v)，再以它为起点规划新段」，
 *     这样速度连续；不要每次把起始速度清零，也不要每周期把参考强制改成实测值。
 *  5. 完成时间不要「按剩余距离反解」。剩余距离里含有跟踪滞后，
 *     距离大→T 大→滞后更大，会形成正反馈把参考越甩越远。
 *     本文件的做法：从小的基准时长起步，只取「满足峰值限幅的最短时间」。
 *  6. 以下为旧版「摇杆累计位置 + 本规划器」的宿主仿真记录（u=1.2 rad/s、a_max=12），
 *     不代表非零终点速度普遍不稳定。当前遥控已改用 velocity_planning.hpp；本文件保留用于绝对定位：
 *       - vel_target = 0（每段都朝「累计目标」刹车）：满偏保持时滞后收敛到 ≈0.15 rad(9°) 且不随时间增长，
 *         松杆后 0.2 s 精确停在累计目标上，无残余误差。当前绝对定位仍使用零终点速度。
 *       - vel_target = 摇杆速度：滞后会发散（20 s 内涨到 11.9 rad），不要用在「累计目标」这种
 *         会自己往前跑的终点上。该参数留给「终点是预先规划的航点」的多段衔接场景。
 * ========================================================================== */

namespace Route_Plan_N
{
	inline constexpr float time_min = 0.002f;   // 完成时间下限 (s)：避免除零与无限加速度
	inline constexpr float time_max = 60.f;     // 完成时间上限 (s)：兜底，正常用不到
	inline constexpr float plan_time = 0.02f;   // 完成时间基准值 (s)：小→跟随紧、滞后小；大→更柔但滞后大

	/* 规划输入：5 个边界条件 */
	struct cubic_cfg_s
	{
		float pos_start;
		float vel_start;
		float pos_target;
		float vel_target;
		float time_total;  // T (s)
	};

	/* 规划输出：同一时刻的参考量 */
	struct cubic_ref_s
	{
		float pos;
		float vel;
		float acc;
	};

	/* 解出的多项式系数（u 形式）。预存 T、1/T、1/T²，避免在控制周期里做除法。 */
	struct cubic_coeff_s
	{
		float b0;
		float b1;
		float b2;
		float b3;
		float T;
		float inv_T;
		float inv_T2;
	};

	/* 对外接口①：由 5 个边界条件求系数 */
	[[nodiscard]] inline cubic_coeff_s CubicCoeff(const cubic_cfg_s& cfg)
	{
		const float T = (cfg.time_total > time_min) ? cfg.time_total : time_min;
		const float dq = cfg.pos_target - cfg.pos_start;
		cubic_coeff_s c;
		c.b0 = cfg.pos_start;
		c.b1 = T * cfg.vel_start;
		c.b2 = 3.f * dq - T * (2.f * cfg.vel_start + cfg.vel_target);
		c.b3 = -2.f * dq + T * (cfg.vel_start + cfg.vel_target);
		c.T = T;
		c.inv_T = 1.f / T;
		c.inv_T2 = c.inv_T * c.inv_T;
		return c;
	}

	/* 对外接口②：采样 t 时刻的参考位置/速度/加速度。t 夹到 [0, T]，不外推。 */
	[[nodiscard]] inline cubic_ref_s CubicSample(const cubic_coeff_s& c, const float t)
	{
		const float tc = (t > 0.f) ? ((t < c.T) ? t : c.T) : 0.f;
		const float u = tc * c.inv_T;
		cubic_ref_s out;
		out.pos = c.b0 + u * (c.b1 + u * (c.b2 + u * c.b3));  // Horner
		out.vel = (c.b1 + u * (2.f * c.b2 + 3.f * c.b3 * u)) * c.inv_T;
		out.acc = (2.f * c.b2 + 6.f * c.b3 * u) * c.inv_T2;
		return out;
	}

	/* 一段轨迹的峰值 |速度| 与 |加速度|（关于 u 的闭式极值，不是采样、也不是上界）：
	       vel(u) = (b1 + 2·b2·u + 3·b3·u²) / T   → 二次，极值在两端或 u* = -b2/(3·b3)
	       acc(u) = (2·b2 + 6·b3·u) / T²           → 线性，极值必在两端 */
	struct cubic_peak_s
	{
		float vel;
		float acc;
	};

	[[nodiscard]] inline cubic_peak_s CubicPeaks(const cubic_coeff_s& c)
	{
		cubic_peak_s p;
		p.acc = fmaxf(Abs(2.f * c.b2), Abs(2.f * c.b2 + 6.f * c.b3)) * c.inv_T2;
		p.vel = fmaxf(Abs(c.b1), Abs(c.b1 + 2.f * c.b2 + 3.f * c.b3)) * c.inv_T;
		if (c.b3 != 0.f)
		{
			const float u = -c.b2 / (3.f * c.b3);
			if ((u > 0.f) && (u < 1.f))
			{
				const float v = Abs((c.b1 + u * (2.f * c.b2 + 3.f * c.b3 * u)) * c.inv_T);
				if (v > p.vel) p.vel = v;
			}
		}
		return p;
	}

	/* 给定完成时间时，这一段是否满足峰值限幅 */
	[[nodiscard]] inline bool CubicWithinLimits(const float dq, const float vel_start, const float vel_target,
		const float T, const float vel_max, const float acc_max)
	{
		cubic_cfg_s cfg{0.f, vel_start, dq, vel_target, T};
		const cubic_peak_s p = CubicPeaks(CubicCoeff(cfg));
		if ((vel_max > 0.f) && (p.vel > vel_max)) return false;
		if ((acc_max > 0.f) && (p.acc > acc_max)) return false;
		return true;
	}

	/* 按「峰值速度 ≤ vel_max、峰值加速度 ≤ acc_max」反解完成时间 (s)。
	   思路：基准时长 base_time 先试一次，可行就直接用（巡航时滞后 ≈ 巡航速度 × base_time，
	   所以 base_time 要小）；不可行则加倍扩张找可行上界，再二分成「满足限幅的最短时间」。
	   只取满足限幅的最短时间，不做「一步放大到很大」的近似 —— 放大过头会让参考比目标慢，
	   剩余距离变大 → T 又变大，滞后会正反馈发散。
	   不要用「按剩余距离反解 T」的写法，同样会发散。
	   若连 time_max 都不可行（例如边界速度本身已超限），返回 time_max 这段最慢的方案。 */
	[[nodiscard]] inline float CubicTimeByLimits(const float dq, const float vel_start, const float vel_target,
		const float vel_max, const float acc_max, const float base_time = plan_time)
	{
		float t_lo = (base_time > time_min) ? base_time : time_min;
		if (t_lo > time_max) return time_max;
		if ((vel_max <= 0.f) && (acc_max <= 0.f)) return t_lo;
		if (CubicWithinLimits(dq, vel_start, vel_target, t_lo, vel_max, acc_max)) return t_lo;

		// 加倍扩张，先找到一个可行的时间上界
		float t_hi = t_lo;
		for (int i = 0; (i < 24) && (t_hi < time_max); ++i)
		{
			t_hi = fminf(t_hi * 2.f, time_max);
			if (CubicWithinLimits(dq, vel_start, vel_target, t_hi, vel_max, acc_max)) break;
		}
		if (!CubicWithinLimits(dq, vel_start, vel_target, t_hi, vel_max, acc_max)) return t_hi;
		const float t_ok = t_hi;  // 已确认可行，作为二分失败时的兜底

		// 二分：峰值随 T 下降，收敛到最短可行时间
		for (int i = 0; i < 24; ++i)
		{
			const float t_m = 0.5f * (t_lo + t_hi);
			if (CubicWithinLimits(dq, vel_start, vel_target, t_m, vel_max, acc_max)) t_hi = t_m;
			else t_lo = t_m;
		}
		// 非严格单调的退化情形下，二分结果可能不可行，退回已知可行值
		return CubicWithinLimits(dq, vel_start, vel_target, t_hi, vel_max, acc_max) ? t_hi : t_ok;
	}

	/* 有状态封装：用来替代原来的 slope_s。
	   Plan() 重置一段轨迹，Update(dt) 按控制周期推进并返回参考量；
	   段结束后恒返回终点（位置=目标位置，速度=目标速度，加速度=0），不再外推。 */
	struct cubic_plan_s
	{
		/* 对外接口③：显式给定 5 个边界条件，重置为一段新轨迹 */
		void Plan(const float pos_target_, const float time_total_, const float vel_target_,
			const float pos_start_, const float vel_start_)
		{
			cfg.pos_start = pos_start_;
			cfg.vel_start = vel_start_;
			cfg.pos_target = pos_target_;
			cfg.vel_target = vel_target_;
			cfg.time_total = (time_total_ > time_min) ? time_total_ : time_min;
			coeff = CubicCoeff(cfg);
			t = 0.f;
			running = true;
		}

		/* 对外接口④：以当前参考状态为起点重规划到新目标（速度连续、加速度可能跳变） */
		void Replan(const float pos_target_, const float time_total_, const float vel_target_)
		{
			const cubic_ref_s now = Ref();
			Plan(pos_target_, time_total_, vel_target_, now.pos, now.vel);
		}

		/* 推进一个控制周期并返回本周期参考量 */
		cubic_ref_s Update(const float dt)
		{
			if (!running) return End();
			t += (dt > 0.f) ? dt : 0.f;
			if (t >= cfg.time_total)
			{
				running = false;  // 到位：之后恒定输出终点，由控制器维持
				return End();
			}
			return CubicSample(coeff, t);
		}

		/* 当前参考状态（重规划取起点用） */
		[[nodiscard]] cubic_ref_s Ref() const
		{
			return running ? CubicSample(coeff, t) : End();
		}

		[[nodiscard]] bool IsArrived() const { return !running; }

		/* 段终点参考量。注意 vel_target ≠ 0 表示「段结束时仍在运动」，
		   需要由后续段接续；若希望停住，请把 vel_target 给 0。 */
		[[nodiscard]] cubic_ref_s End() const
		{
			cubic_ref_s out;
			out.pos = cfg.pos_target;
			out.vel = cfg.vel_target;
			out.acc = 0.f;
			return out;
		}

		cubic_cfg_s cfg{};
		cubic_coeff_s coeff{};
		float t{0.f};
		bool running{false};
	};
}
