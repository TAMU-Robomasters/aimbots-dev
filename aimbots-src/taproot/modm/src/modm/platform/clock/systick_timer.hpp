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

/// @ingroup modm_platform_clock
class SysTickTimer
{
public:
	/**
	 * Initializes the SysTick to provide a timer to `modm::Clock` and
	 * `modm::PreciseClock`.
	 *
	 * @tparam	SystemClock
	 * 		the currently active system clock
	 * @tparam	tolerance
	 * 		the allowed tolerance for the resulting clock rate
	 */
	template< class SystemClock, percent_t tolerance=pct(0) >
	static void
	initialize()
	{
		static_assert(SystemClock::Frequency < (1ull << 24)*8*100,
		              "HLCK is too fast for the SysTick to run at 100Hz!");
		if constexpr (SystemClock::Frequency < 8'000'000)
		{
			constexpr auto result = Prescaler::from_linear(
					SystemClock::Frequency, 100, 1, (1ul << 24)-1);
			PeripheralDriver::assertBaudrateInTolerance< result.frequency, 100, tolerance >();

			us_per_Ncycles = ((1ull << Ncycles) * 1'000'000ull) / SystemClock::Frequency;
			ms_per_Ncycles = ((1ull << Ncycles) * 1'000ull) / SystemClock::Frequency;
			enable(result.index, true);
		}
		else
		{
			constexpr auto result = Prescaler::from_linear(
					SystemClock::Frequency/8, 100, 1, (1ul << 24)-1);
			PeripheralDriver::assertBaudrateInTolerance< result.frequency, 100, tolerance >();

			us_per_Ncycles = ((1ull << Ncycles) * 8'000'000ull) / SystemClock::Frequency;
			ms_per_Ncycles = ((1ull << Ncycles) * 8'000ull) / SystemClock::Frequency;
			enable(result.index, false);
		}
	}

	/**
	 * Disables SysTick Timer.
	 *
	 * @warning	If the SysTick Timer is disabled, `modm::Clock` and
	 *			`modm::PreciseClock` will stop incrementing.
	 */
	static void
	disable();

private:
	static void
	enable(uint32_t reload, bool use_processor_clock);

public:
	/// @cond
	// FCPU < 8MHz
	// 536e6/4 < 27-bit, 8e6/4 < 21-bit
	// 2^32*1e6/536e6 < 23-bit, 2^32*1e6/8e6 = 29-bit
	// FCPU >= 8MHz
	// 536e6/8/4 < 24-bit, 8e6/8/4 < 18-bit
	// 2^32*8e6/536e6 < 26-bit, 2^32*8e6/8e6 = 32-bit
	static constexpr uint8_t Ncycles{32};
	static inline uint32_t ms_per_Ncycles{0};
	static inline uint32_t us_per_Ncycles{0};
	/// @endcond
};

}

#endif	//  MODM_STM32_CORTEX_TIMER_HPP