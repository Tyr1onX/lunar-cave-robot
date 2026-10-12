#pragma once

#include <cmath>

namespace Math_N
{
	inline constexpr float dig_to_rad = 0.0174533f;
	inline constexpr float pi = 3.14159265f;
	inline constexpr float two_pi = 2.f * pi;
}
[[gnu::always_inline]] inline float Abs(const float value) { return fabsf(value); }

[[gnu::always_inline]] inline float Sign(const float value)
{
	if (value > 0.f) return 1.f;
	if (value < 0.f) return -1.f;
	return 0.f;
}

[[gnu::always_inline]] inline float Limit(const float value, const float max, const float min)
{
	return fmaxf(fminf(value, max), min);
}

[[gnu::always_inline]] inline float Limit(const float value, const float limit_abs)
{
	return fmaxf(fminf(value, limit_abs), -limit_abs);
}

[[gnu::always_inline]] inline void LimitSelf(float& value, const float max, const float min)
{
	value = fmaxf(fminf(value, max), min);
}

[[gnu::always_inline]] inline void LimitSelf(float& value, const float limit_abs)
{
	value = fmaxf(fminf(value, limit_abs), -limit_abs);
}

inline float DeadZone(const float val, const float dead_zone)
{
	if (val > dead_zone) return val - dead_zone;
	if (val < -dead_zone) return val + dead_zone;
	return 0.f;
}

[[gnu::always_inline]] inline float Clamp(float val, const float max, const float min)
{
    const float range = max - min;
    val = fmodf(val - min, range);
    if (val < 0) val += range;
    return val + min;
}

[[gnu::always_inline]] inline float Sqrt(const float value) { return value > 0.f ? sqrtf(value) : 0.f; }  // vsqrt.f32

[[gnu::always_inline]] inline float WrapRad(const float value)
{
	return Clamp(value, Math_N::pi, -Math_N::pi);
}

[[gnu::always_inline]] inline float UnwrapRadNear(const float value, const float reference)
{
	return reference + WrapRad(value - reference);
}

inline float RadDiff(const float _q1, const float _q2) { return WrapRad(_q1 - _q2); }
