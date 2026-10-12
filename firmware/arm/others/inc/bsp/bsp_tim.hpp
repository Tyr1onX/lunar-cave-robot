/**
 * DWT-based microsecond timer.
 * Replaces the TIM23/TIM7 version: those timers are not present in the
 * current CubeMX configuration, so the 32-bit DWT cycle counter is used.
 */

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32h7xx_hal.h"

#ifdef __cplusplus
}
#endif


class Timebase_Timer
{
	public:
	Timebase_Timer() : m_cycles_per_us(SystemCoreClock / 1000000u) {}
	void Init() const
	{
		CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;  // enable DWT
		DWT->CYCCNT = 0;  // reset the counter
		DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;  // enable the counter
	}
	static uint32_t GetMilliSecond() { return HAL_GetTick(); }
	uint64_t GetMicroSecond() const
	{
		const uint32_t tick_old = HAL_GetTick();
		const uint32_t cnt_old = DWT->CYCCNT / m_cycles_per_us;
		const uint32_t tick_new = HAL_GetTick();
		const uint32_t cnt_new = DWT->CYCCNT / m_cycles_per_us;
		const uint32_t delta_ms = tick_new - tick_old;
		do
		{
			switch (delta_ms)
			{
			case 0:
				return static_cast<uint64_t>(tick_new) * 1000 + static_cast<uint64_t>(cnt_old);
			case 1:  // interrupt happened between two reads
				return static_cast<uint64_t>(tick_new) * 1000 + static_cast<uint64_t>(cnt_new);
			default:  // indicates that interrupt took more than 1ms, an error case
				continue;
			}
		}
		while (true);
	}
	void MicroDelayTimer(const uint32_t us_) const
	{
		if (__builtin_expect(us_ == 0, 0)) return;
		const uint64_t start = GetMicroSecond();
		while (GetMicroSecond() - start < us_) {}
	}
	private:
	const uint32_t m_cycles_per_us;
};
