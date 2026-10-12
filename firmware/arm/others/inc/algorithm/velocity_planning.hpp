#pragma once

#include "mathetics.hpp"

namespace Route_Plan_N
{
	struct motion_ref_s { float pos; float vel; float acc; };
	struct position_range_s { float min; float max; };
	struct jerk_segment_s { float jerk; float time; };
	struct velocity_profile_s
	{
		jerk_segment_s segments[3];
		float target;
	};

	/* 参考硬限位优先于连续性：已经触限/越界的外向状态不能再用于积分。
	   返回 -1/0/+1 表示下限/未裁剪/上限。向内运动不被边界锁住。 */
	inline int ClampMotionPosition(motion_ref_s& ref, const float lower, const float upper)
	{
		if (ref.pos > upper || (ref.pos >= upper && (ref.vel > 0.f || ref.acc > 0.f)))
		{
			ref = {upper, 0.f, 0.f};
			return 1;
		}
		if (ref.pos < lower || (ref.pos <= lower && (ref.vel < 0.f || ref.acc < 0.f)))
		{
			ref = {lower, 0.f, 0.f};
			return -1;
		}
		return 0;
	}

	/* 速度域的 S 曲线：从任意 (v,a) 到 (v_target,0)。三段分别为恒 jerk、
	   恒加速度、反向 jerk；小速度变化时中间段为零。位置由各段解析积分得到。
	   输入须有限且 a_max/j_max > 0，初始 |a| <= a_max。 */
	[[nodiscard]] inline velocity_profile_s VelocityProfile(const motion_ref_s& start,
		const float target, const float a_max, const float j_max)
	{
		const float dv = target - start.vel;
		const float unwind = start.acc * Abs(start.acc) / (2.f * j_max);
		const float direction = (dv >= unwind) ? 1.f : -1.f;
		const float peak = sqrtf(fmaxf(0.f, direction * j_max * dv + 0.5f * start.acc * start.acc));
		const float a_peak = direction * fminf(peak, a_max);
		const float t1 = fmaxf(0.f, (a_peak - start.acc) / (direction * j_max));
		const float t3 = Abs(a_peak) / j_max;
		const float dv_ramps = 0.5f * (start.acc + a_peak) * t1 + 0.5f * a_peak * t3;
		const float t2 = (peak > a_max) ? fmaxf(0.f, (dv - dv_ramps) / a_peak) : 0.f;
		return {{{direction * j_max, t1}, {0.f, t2}, {-direction * j_max, t3}}, target};
	}

	inline void IncludePosition(position_range_s& range, const float q)
	{
		range.min = fminf(range.min, q);
		range.max = fmaxf(range.max, q);
	}

	[[nodiscard]] inline float JerkPosition(const motion_ref_s& r, const float j, const float t)
	{
		return r.pos + t * (r.vel + t * (0.5f * r.acc + t * j / 6.f));
	}

	/* 同时检查段内 v(t)=0 的位置极值，不能只检查段终点。 */
	inline void IntegrateJerk(motion_ref_s& r, const float j, const float t, position_range_s& range)
	{
		if (!(t > 0.f)) return;
		if (j != 0.f)
		{
			const float discriminant = r.acc * r.acc - 2.f * j * r.vel;
			if (discriminant >= 0.f)
			{
				const float root = sqrtf(discriminant);
				const float t1 = (-r.acc + root) / j;
				const float t2 = (-r.acc - root) / j;
				if (t1 > 0.f && t1 < t) IncludePosition(range, JerkPosition(r, j, t1));
				if (t2 > 0.f && t2 < t) IncludePosition(range, JerkPosition(r, j, t2));
			}
		}
		else if (r.acc != 0.f)
		{
			const float t0 = -r.vel / r.acc;
			if (t0 > 0.f && t0 < t) IncludePosition(range, JerkPosition(r, j, t0));
		}
		r.pos = JerkPosition(r, j, t);
		r.vel += t * (r.acc + 0.5f * j * t);
		r.acc += j * t;
		IncludePosition(range, r.pos);
	}

	inline void AdvanceVelocity(motion_ref_s& ref, const velocity_profile_s& p, float dt,
		position_range_s& range)
	{
		for (const auto& segment : p.segments)
		{
			const float step = fminf(dt, segment.time);
			IntegrateJerk(ref, segment.jerk, step, range);
			dt -= step;
			if (step < segment.time) return;
		}
		// 到达目标速度后继续匀速积分；目标为零时自然保持停止位置。
		ref.vel = p.target;
		ref.acc = 0.f;
		IntegrateJerk(ref, 0.f, dt, range);
	}

	[[nodiscard]] inline position_range_s BrakingRange(motion_ref_s ref,
		const float a_max, const float j_max)
	{
		position_range_s range{ref.pos, ref.pos};
		const auto p = VelocityProfile(ref, 0.f, a_max, j_max);
		for (const auto& segment : p.segments)
			IntegrateJerk(ref, segment.jerk, segment.time, range);
		return range;
	}

	struct velocity_plan_s
	{
		motion_ref_s ref{};
		bool braking{false};  // 限位/跟踪窗口阻止了本拍的操作指令
		bool saturated{false}; // 已触及硬边界，本拍允许覆盖 S 曲线连续性

		void Reset(const float pos, const float vel = 0.f, const float acc = 0.f)
		{
			ref = {pos, vel, acc};
			braking = false;
			saturated = false;
		}

		/* 先试走本拍并检查完整制动包络；不可行时改走制动曲线。
		   积分前后都执行硬限位，制动已来不及时也不允许参考越界。
		   这是参考状态裁剪，不能让有惯性的实际机械臂瞬间停住。 */
		motion_ref_s Update(const float command, const float dt, const float v_max,
			const float a_max, const float j_max, const float pos_min, const float pos_max)
		{
			if (!(dt > 0.f) || !std::isfinite(dt) || !(a_max > 0.f) || !(j_max > 0.f) || pos_min > pos_max) return ref;
			saturated = ClampMotionPosition(ref, pos_min, pos_max) != 0;
			float target = std::isfinite(command) ? Limit(command, v_max) : 0.f;
			if ((ref.pos >= pos_max && target > 0.f) || (ref.pos <= pos_min && target < 0.f)) target = 0.f;
			motion_ref_s candidate = ref;
			position_range_s travelled{ref.pos, ref.pos};
			AdvanceVelocity(candidate, VelocityProfile(ref, target, a_max, j_max), dt, travelled);
			const auto stop = BrakingRange(candidate, a_max, j_max);
			constexpr float epsilon = 1.e-6f;  // 仅浮点容差，不是可用制动距离
			const float lower = pos_min - epsilon;
			const float upper = pos_max + epsilon;
			braking = travelled.min < lower || travelled.max > upper || stop.min < lower || stop.max > upper;
			if (braking)
			{
				travelled = {ref.pos, ref.pos};
				AdvanceVelocity(ref, VelocityProfile(ref, 0.f, a_max, j_max), dt, travelled);
			}
			else ref = candidate;
			// 包含本拍段内极值，防止先越界再回到范围内而漏检。
			if (travelled.max > pos_max)
			{
				ref = {pos_max, 0.f, 0.f};
				saturated = true;
			}
			else if (travelled.min < pos_min)
			{
				ref = {pos_min, 0.f, 0.f};
				saturated = true;
			}
			saturated = (ClampMotionPosition(ref, pos_min, pos_max) != 0) || saturated;
			return ref;
		}
	};
}
