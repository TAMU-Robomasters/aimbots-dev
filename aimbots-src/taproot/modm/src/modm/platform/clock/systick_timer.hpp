/*
 * Copyright (c) 2013, Kevin Läufer
 * Copyright (c) 2014-2017, Niklas Hauser
 *
 * This file is part of the modm project.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
// ----------------------------------------------------------------------------

#ifndef MODM_CORTEX_SYSTICK_TIMER_HPP
#define MODM_CORTEX_SYSTICK_TIMER_HPP

#include <modm/architecture/interface/peripheral.hpp>
#include <modm/architecture/interface/clock.hpp>
#include <modm/math/algorithm/prescaler.hpp>

namespace modm::platform
{

typedef void (*InterruptHandler)(void);

/**
 * @brief		SysTick Timer
 * @ingroup		modm_platform_clock_cortex
 */
class SysTickTimer
{
public:
	/**
	 * Initializes the SysTick Timer to generate periodic events.
	 *
	 *
	 * @warning	The SysTick Timer is used by default to increment
	 * 			modm::Clock, which is used by modm::Timeout and other
	 * 			similar processing classes.
	 * 			You must not increment the modm::Clock
	 * 			additionally somewhere else.
	 *
	 * @tparam	SystemClock
	 * 		the currently active system clock
	 * @tparam	rate
	 * 		the desired update rate of the modm::Clock
	 * @tparam	tolerance
	 * 		the allowed absolute tolerance for the resulting clock rate
	 */
	template< class SystemClock, percent_t tolerance=pct(0) >
	static void
	initialize()
	{
		static_assert(SystemClock::Frequency < (1ull << 24)*8*4,
		              "HLCK is too fast for the SysTick to run at 4Hz!");
		if constexpr (SystemClock::Frequency < 8'000'000)
		{
			constexpr auto result = Prescaler::from_range(
					SystemClock::Frequency, 4, 1, (1ul << 24)-1);
			PeripheralDriver::assertBaudrateInTolerance< result.frequency, 4, tolerance >();

			us_per_Ncycles = ((1ull << Ncycles) * 1'000'000ull) / SystemClock::Frequency;
			ms_per_Ncycles = ((1ull << Ncycles) * 1'000ull) / SystemClock::Frequency;
			enable(result.index, false);
		}
		else
		{
			constexpr auto result = Prescaler::from_range(
					SystemClock::Frequency/8, 4, 1, (1ul << 24)-1);
			PeripheralDriver::assertBaudrateInTolerance< result.frequency, 4, tolerance >();

			us_per_Ncycles = ((1ull << Ncycles) * 8'000'000ull) / SystemClock::Frequency;
			ms_per_Ncycles = ((1ull << Ncycles) * 8'000ull) / SystemClock::Frequency;
			enable(result.index, true);
		}
	}

	/**
	 * Disables SysTick Timer.
	 *
	 * @warning	If the SysTick Timer is disabled modm::Clock is not
	 * 			incremented automatically. Workflow classes which
	 * 			rely on modm::Clock will not work if modm::Clock
	 * 			is not incremented.
	 */
	static void
	disable();

private:
	static void
	enable(uint32_t reload, bool prescaler8);

	// FCPU < 8MHz
	// 536e6/4 < 27-bit, 8e6/4 < 21-bit
	// 2^32*1e6/536e6 < 23-bit, 2^32*1e6/8e6 = 29-bit
	// FCPU >= 8MHz
	// 536e6/8/4 < 24-bit, 8e6/8/4 < 18-bit
	// 2^32*8e6/536e6 < 26-bit, 2^32*8e6/8e6 = 32-bit
	static constexpr uint8_t Ncycles{32};
	static inline uint32_t ms_per_Ncycles{0};
	static inline uint32_t us_per_Ncycles{0};
	friend class modm::chrono::milli_clock;
	friend class modm::chrono::micro_clock;
};

}

namespace modm::cortex {
	using SysTickTimer [[deprecated("Use `modm::platform:SysTickTimer` instead!")]] =
		::modm::platform::SysTickTimer;
}

#endif	//  MODM_STM32_CORTEX_TIMER_HPP