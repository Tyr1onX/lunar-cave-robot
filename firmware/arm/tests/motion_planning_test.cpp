#include "velocity_planning.hpp"
#include "route_planning.hpp"
#include <cassert>
#include <cstdio>
#include <random>

using namespace Route_Plan_N;
constexpr float dt = 0.002f;
constexpr float vmax = 1.2f;
constexpr float amax = 12.f;
constexpr float jmax = 240.f;

static motion_ref_s Step(velocity_plan_s& p, float command, float lo = -100.f, float hi = 100.f)
{
	const auto before = p.ref;
	const auto r = p.Update(command, dt, vmax, amax, jmax, lo, hi);
	assert(std::isfinite(r.pos) && std::isfinite(r.vel) && std::isfinite(r.acc));
	assert(Abs(r.vel) <= vmax + 2.e-5f);
	assert(Abs(r.acc) <= amax + 2.e-4f);
	// 已触限时允许参考状态硬裁剪；正常段仍验证加速度/jerk 连续约束。
	if (!p.saturated)
	{
		assert(Abs(r.acc - before.acc) <= jmax * dt + 2.e-4f);
		assert(Abs(r.vel - before.vel) <= amax * dt + 2.e-5f);
	}
	assert(Abs(r.pos - before.pos) <= vmax * dt + 1.e-5f);
	return r;
}

int main()
{
	// 恒定 jerk 第一拍应有精确的三次位置积分，而非直接积分原始速度。
	velocity_plan_s p;
	auto r = Step(p, vmax);
	assert(Abs(r.pos - jmax * dt * dt * dt / 6.f) < 1.e-9f);
	assert(Abs(r.vel - 0.5f * jmax * dt * dt) < 1.e-8f);
	assert(Abs(r.acc - jmax * dt) < 1.e-6f);
	for (int i = 1; i < 500; ++i) Step(p, vmax);
	assert(Abs(p.ref.vel - vmax) < 1.e-5f && Abs(p.ref.acc) < 1.e-5f);
	// a=0 时达到 vmax 的总时长为 vmax/amax + amax/jmax = 0.15s。
	assert(Abs(p.ref.pos - (vmax * (1.f - 0.075f))) < 5.e-5f);
	const float release_pos = p.ref.pos;
	for (int i = 0; i < 500; ++i) Step(p, 0.f);
	assert(Abs(p.ref.vel) < 1.e-6f && Abs(p.ref.acc) < 1.e-6f);
	assert(Abs(p.ref.pos - release_pos - 0.09f) < 5.e-5f);
	const float stop_pos = p.ref.pos;
	for (int i = 0; i < 1000; ++i) Step(p, 0.f);
	assert(p.ref.pos == stop_pos);
	std::printf("cruise release: stop distance %.6f rad, expected 0.090000\n", stop_pos - release_pos);

	// 加速尚未结束就松杆/换向，速度和加速度仍连续。
	for (int release_tick : {1, 5, 20, 35, 55})
	{
		p.Reset(0.f);
		for (int i = 0; i < release_tick; ++i) Step(p, vmax);
		for (int i = 0; i < 300; ++i) Step(p, -vmax);
		assert(Abs(p.ref.vel + vmax) < 1.e-5f);
		for (int i = 0; i < 300; ++i) Step(p, 0.f);
		assert(Abs(p.ref.vel) < 1.e-6f && Abs(p.ref.acc) < 1.e-6f);
	}

	// 长时间顶限位、反向离开，不允许累计一份隐藏的原始速度目标。
	for (float sign : {-1.f, 1.f})
	{
		p.Reset(0.f);
		for (int i = 0; i < 5000; ++i)
		{
			r = Step(p, sign * vmax, -0.3f, 0.3f);
			assert(r.pos >= -0.30001f && r.pos <= 0.30001f);
		}
		assert(Abs(p.ref.vel) < 0.001f);
		const float edge = p.ref.pos;
		for (int i = 0; i < 150; ++i) Step(p, -sign * vmax, -0.3f, 0.3f);
		assert(sign * (p.ref.pos - edge) < -0.1f);
	}

	// 反馈完全不动：制动包络被跟踪窗口限制，持杆 20s 仍不继续积累。
	p.Reset(0.f);
	for (int i = 0; i < 10000; ++i) Step(p, vmax, -0.35f, 0.35f);
	assert(p.ref.pos <= 0.35001f && Abs(p.ref.vel) < 0.001f);

	// 回归：带外向速度已经到边界/来不及制动/已经越界，都不能继续积分到界外。
	for (float sign : {-1.f, 1.f})
	{
		for (float start : {0.9975f, 0.998f, 1.088f})
		{
			for (float acceleration : {-6.f, 0.f, 6.f})
			{
				p.Reset(sign * start, sign * vmax, acceleration);
				r = p.Update(sign * vmax, dt, vmax, amax, jmax, -0.998f, 0.998f);
				assert(r.pos == sign * 0.998f && r.vel == 0.f && r.acc == 0.f && p.saturated);
			}
			for (int i = 0; i < 10000; ++i)
			{
				r = Step(p, sign * vmax, -0.998f, 0.998f);
				assert(r.pos == sign * 0.998f && r.vel == 0.f && r.acc == 0.f);
			}
			r = Step(p, -sign * vmax, -0.998f, 0.998f);
			assert(sign * r.vel < 0.f && sign * r.acc < 0.f && sign * r.pos < 0.998f);
		}
	}

	// 随机跳变命令 + 不对称限位，覆盖加速度非零时的连续重规划。
	std::mt19937 random(20260926);
	std::uniform_real_distribution<float> command(-2.f, 2.f);
	p.Reset(0.f);
	float u = 0.f;
	for (int i = 0; i < 100000; ++i)
	{
		if (i % 13 == 0) u = command(random);
		r = Step(p, u, -0.7f, 0.4f);
		assert(r.pos >= -0.70002f && r.pos <= 0.40002f);
	}

	// 零 dt 不推进；NaN 命令进入受限停止，恢复 Reset 不保留旧积分。
	const auto saved = p.ref;
	p.Update(vmax, 0.f, vmax, amax, jmax, -1.f, 1.f);
	assert(p.ref.pos == saved.pos && p.ref.vel == saved.vel);
	for (int i = 0; i < 500; ++i) Step(p, NAN);
	assert(p.ref.vel == 0.f);
	p.Reset(0.12f);
	assert(p.ref.pos == 0.12f && p.ref.vel == 0.f && p.ref.acc == 0.f);

	// 原位置规划保留：固定目标、非零起始速度，最终仍准确到位并静止。
	cubic_plan_s position;
	const float duration = CubicTimeByLimits(0.8f, 0.3f, 0.f, 2.f, amax);
	assert(CubicWithinLimits(0.8f, 0.3f, 0.f, duration, 2.f, amax));
	position.Plan(0.8f, duration, 0.f, 0.f, 0.3f);
	for (int i = 0; i < 5000; ++i) position.Update(dt);
	assert(position.IsArrived() && position.Ref().pos == 0.8f && position.Ref().vel == 0.f);
	std::puts("motion planning tests passed");
}
