/*
 * Copyright (c) 2009, 2011-2012, Georgi Grinshpun
 * Copyright (c) 2009-2012, 2016-2017, Fabian Greif
 * Copyright (c) 2010, Martin Rosekeit
 * Copyright (c) 2011, 2013-2017, Niklas Hauser
 * Copyright (c) 2013-2014, 2016, Kevin Läufer
 * Copyright (c) 2014, Sascha Schade
 *
 * This file is part of the modm project.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
// ----------------------------------------------------------------------------

#ifndef MODM_STM32_TIMER_12_HPP
#define MODM_STM32_TIMER_12_HPP

#include "general_purpose_base.hpp"
#include <modm/platform/gpio/connector.hpp>

namespace modm
{
namespace platform
{
/**
 * General Purpose Timer 12
 *
 * Interrupt handler:
 * @code
 * MODM_ISR(TIM12)
 * {
 *     Timer12::resetInterruptFlags(Timer12::...);
 *
 *     ...
 * }
 * @endcode
 *
 * @warning	The Timer has much more possibilities than presented by this
 * 			interface (e.g. Input Capture, Trigger for other Timers, DMA).
 * 			It might be expanded in the future.
 *
 * @author		Fabian Greif
 * @ingroup		modm_platform_timer
 */
class Timer12 : public GeneralPurposeTimer
{
public:
	enum class MasterMode : uint32_t
	{
		Reset 			= 0,							// 0b000
		Enable 			= TIM_CR2_MMS_0,				// 0b001
		Update 			= TIM_CR2_MMS_1,				// 0b010
		Pulse 			= TIM_CR2_MMS_1 | TIM_CR2_MMS_0,// 0b011
		CompareOc1Ref 	= TIM_CR2_MMS_2,				// 0b100
		CompareOc2Ref 	= TIM_CR2_MMS_2 | TIM_CR2_MMS_0,// 0b101
	};

	enum class SlaveModeTrigger : uint32_t
	{
		Internal0 = 0,
		Internal1 = TIM_SMCR_TS_0,
		Internal2 = TIM_SMCR_TS_1,
		TimerInput1EdgeDetector = TIM_SMCR_TS_2,
		TimerInput1Filtered = TIM_SMCR_TS_2 | TIM_SMCR_TS_0,
		TimerInput2Filtered = TIM_SMCR_TS_2 | TIM_SMCR_TS_1,
	};

	enum class SlaveMode : uint32_t
	{
		/// Slave mode disabled - if CEN = '1' then the prescaler is clocked directly by the internal clock.
		Disabled	= 0,
		/// Rising edge of the selected trigger input (TRGI) reinitializes the counter and generates an update of the registers.
		Reset		= TIM_SMCR_SMS_2,
		/// The counter clock is enabled when the trigger input (TRGI) is high. The counter stops (but is not reset) as soon as the trigger becomes low. Both start and stop of the counter are controlled.
		Gated		= TIM_SMCR_SMS_2 | TIM_SMCR_SMS_0,
		/// The counter starts at a rising edge of the trigger TRGI (but it is not reset). Only the start of the counter is controlled.
		Trigger	= TIM_SMCR_SMS_2 | TIM_SMCR_SMS_1,
		/// Rising edges of the selected trigger (TRGI) clock the counter.
		ExternalClock = TIM_SMCR_SMS_2 | TIM_SMCR_SMS_1 | TIM_SMCR_SMS_0,
	};

	// This type is the internal size of the counter.
	typedef uint16_t Value;

	template< template<Peripheral _> class... Signals >
	static void
	connect()
	{
		using Connector = GpioConnector<Peripheral::Tim12, Signals...>;
		Connector::connect();
	}

	// Just enable the clock of the peripheral
	static void
	clockEnable();

	// Enables the clock and resets the timer
	static void
	enable();

	static void
	disable();

	static inline void
	pause()
	{
		TIM12->CR1 &= ~TIM_CR1_CEN;
	}

	static inline void
	start()
	{
		TIM12->CR1 |= TIM_CR1_CEN;
	}

	static void
	setMode(Mode mode,
			SlaveMode slaveMode = SlaveMode::Disabled,
			SlaveModeTrigger slaveModeTrigger = static_cast<SlaveModeTrigger>(0),
			MasterMode masterMode = MasterMode::Reset,
			bool enableOnePulseMode = false,
			bool bufferAutoReloadRegister = true,
			bool limitUpdateEventRequestSource = true);

	static inline void
	setPrescaler(uint16_t prescaler)
	{
		// Because a prescaler of zero is not possible the actual
		// prescaler value is \p prescaler - 1 (see Datasheet)
		TIM12->PSC = prescaler - 1;
	}

	static uint16_t
	getPrescaler()
	{
		return (TIM12->PSC + 1);
	}

	static inline void
	setOverflow(Value overflow)
	{
		TIM12->ARR = overflow;
	}

	template<class SystemClock>
	static Value
	setPeriod(uint32_t microseconds, bool autoApply = true)
	{
		// This will be inaccurate for non-smooth frequencies (last six digits
		// unequal to zero)
		uint32_t cycles = microseconds * (SystemClock::Timer12 / 1'000'000UL);
		uint16_t prescaler = (cycles + 65'535) / 65'536;	// always round up
		Value overflow = cycles / prescaler;

		overflow = overflow - 1;	// e.g. 36'000 cycles are from 0 to 35'999

		setPrescaler(prescaler);
		setOverflow(overflow);

		if (autoApply) {
			// Generate Update Event to apply the new settings for ARR
			TIM12->EGR |= TIM_EGR_UG;
		}

		return overflow;
	}

	/* Returns the frequency of the timer */
	template<class SystemClock>
	static uint32_t
	getTickFrequency()
	{
		return SystemClock::Timer12 / (TIM12->PSC + 1);
	}

	static inline void
	applyAndReset()
	{
		// Generate Update Event to apply the new settings for ARR
		TIM12->EGR |= TIM_EGR_UG;
	}

	static inline Value
	getValue()
	{
		return TIM12->CNT;
	}

	static inline void
	setValue(Value value)
	{
		TIM12->CNT = value;
	}


	static inline void
	enableOutput()
	{
		TIM12->BDTR |= TIM_BDTR_MOE;
	}

	static inline void
	disableOutput()
	{
		TIM12->BDTR &= ~(TIM_BDTR_MOE);
	}

	/*
	 * Enable/Disable automatic set of MOE bit at the next update event
	 */
	static inline void
	setAutomaticUpdate(bool enable)
	{
		if(enable)
			TIM12->BDTR |= TIM_BDTR_AOE;
		else
			TIM12->BDTR &= ~TIM_BDTR_AOE;
	}

	static inline void
	setOffState(OffStateForRunMode runMode, OffStateForIdleMode idleMode)
	{
		uint32_t flags = TIM12->BDTR;
		flags &= ~(TIM_BDTR_OSSR | TIM_BDTR_OSSI);
		flags |= static_cast<uint32_t>(runMode);
		flags |= static_cast<uint32_t>(idleMode);
		TIM12->BDTR = flags;
	}

	/*
	 * Set Dead Time Value
	 *
	 * Different Resolution Depending on DeadTime[7:5]:
	 *     0xx =>  DeadTime[6:0]            * T(DTS)
	 *     10x => (DeadTime[5:0] + 32) *  2 * T(DTS)
	 *     110 => (DeadTime[4:0] + 4)  *  8 * T(DTS)
	 *     111 => (DeadTime[4:0] + 2)  * 16 * T(DTS)
	 */
	static inline void
	setDeadTime(uint8_t deadTime)
	{
		uint32_t flags = TIM12->BDTR;
		flags &= ~TIM_BDTR_DTG;
		flags |= deadTime;
		TIM12->BDTR = flags;
	}

	/*
	 * Set Dead Time Value
	 *
	 * Different Resolution Depending on DeadTime[7:5]:
	 *     0xx =>  DeadTime[6:0]            * T(DTS)
	 *     10x => (DeadTime[5:0] + 32) *  2 * T(DTS)
	 *     110 => (DeadTime[4:0] + 4)  *  8 * T(DTS)
	 *     111 => (DeadTime[4:0] + 2)  * 16 * T(DTS)
	 */
	static inline void
	setDeadTime(DeadTimeResolution resolution, uint8_t deadTime)
	{
		uint8_t bitmask;
		switch(resolution){
			case DeadTimeResolution::From0With125nsStep:
				bitmask = 0b01111111;
				break;
			case DeadTimeResolution::From16usWith250nsStep:
				bitmask = 0b00111111;
				break;
			case DeadTimeResolution::From32usWith1usStep:
			case DeadTimeResolution::From64usWith2usStep:
				bitmask = 0b00011111;
				break;
			default:
				bitmask = 0x00;
				break;
		}
		uint32_t flags = TIM12->BDTR;
		flags &= ~TIM_BDTR_DTG;
		flags |= (deadTime & bitmask) | static_cast<uint32_t>(resolution);
		TIM12->BDTR = flags;
	}
public:
	static void
	configureInputChannel(uint32_t channel, InputCaptureMapping input,
			InputCapturePrescaler prescaler,
			InputCapturePolarity polarity, uint8_t filter,
			bool xor_ch1_3=false);

	static void
	configureOutputChannel(uint32_t channel, OutputCompareMode_t mode,
			Value compareValue, PinState out = PinState::Enable,
			bool enableComparePreload = true);

	/// Switch to Pwm Mode 2
	///
	/// While upcounting channel will be active as long as the time value is
	/// smaller than the compare value, else inactive.
	/// Timer will not be disabled while switching modes.
	static void
	setInvertedPwm(uint32_t channel)
	{
		channel -= 1;	// 1..2 -> 0..1

		{
			uint32_t flags = static_cast<uint32_t>(OutputCompareMode::Pwm2);

			if (channel <= 1)
			{
				uint32_t offset = 8 * channel;

				flags <<= offset;
				flags |= TIM12->CCMR1 & ~(TIM_CCMR1_OC1M << offset);
				TIM12->CCMR1 = flags;
			}
			else {
				uint32_t offset = 8 * (channel - 2);

				flags <<= offset;
				flags |= TIM12->CCMR2 & ~(TIM_CCMR1_OC1M << offset);

				TIM12->CCMR2 = flags;
			}
		}
	}

	/// Switch to Pwm Mode 1
	///
	/// While upcounting channel will be inactive as long as the time value is
	/// smaller than the compare value, else active.
	/// **Please note**: Timer will not be disabled while switching modes.
	static void
	setNormalPwm(uint32_t channel)
	{
		channel -= 1;	// 1..2 -> 0..1

		{
			uint32_t flags = static_cast<uint32_t>(OutputCompareMode::Pwm);

			if (channel <= 1)
			{
				uint32_t offset = 8 * channel;

				flags <<= offset;
				flags |= TIM12->CCMR1 & ~(TIM_CCMR1_OC1M << offset);
				TIM12->CCMR1 = flags;
			}
			else {
				uint32_t offset = 8 * (channel - 2);

				flags <<= offset;
				flags |= TIM12->CCMR2 & ~(TIM_CCMR1_OC1M << offset);

				TIM12->CCMR2 = flags;
			}
		}
	}

	/// Switch to Inactive Mode
	///
	/// The channel output will be forced to the inactive level.
	/// **Please note**: Timer will not be disabled while switching modes.
	static void
	forceInactive(uint32_t channel)
	{
		channel -= 1;	// 1..2 -> 0..1

		{
			uint32_t flags = static_cast<uint32_t>(OutputCompareMode::ForceInactive);

			if (channel <= 1)
			{
				uint32_t offset = 8 * channel;

				flags <<= offset;
				flags |= TIM12->CCMR1 & ~(TIM_CCMR1_OC1M << offset);
				TIM12->CCMR1 = flags;
			}
			else {
				uint32_t offset = 8 * (channel - 2);

				flags <<= offset;
				flags |= TIM12->CCMR2 & ~(TIM_CCMR1_OC1M << offset);

				TIM12->CCMR2 = flags;
			}
		}
	}

	/// Switch to Active Mode
	///
	/// The channel output will be forced to the active level.
	/// **Please note**: Timer will not be disabled while switching modes.
	static void
	forceActive(uint32_t channel)
	{
		channel -= 1;	// 1..2 -> 0..1

		{
			uint32_t flags = static_cast<uint32_t>(OutputCompareMode::ForceActive);

			if (channel <= 1)
			{
				uint32_t offset = 8 * channel;

				flags <<= offset;
				flags |= TIM12->CCMR1 & ~(TIM_CCMR1_OC1M << offset);
				TIM12->CCMR1 = flags;
			}
			else {
				uint32_t offset = 8 * (channel - 2);

				flags <<= offset;
				flags |= TIM12->CCMR2 & ~(TIM_CCMR1_OC1M << offset);

				TIM12->CCMR2 = flags;
			}
		}
	}

	/// Returns if the capture/compare channel of the timer is configured as input.
	///
	/// @param channel may be [1..2]
	/// @return `false` if configured as *output*; `true` if configured as *input*
	static bool
	isChannelConfiguredAsInput(uint32_t channel);

	static inline void
	setCompareValue(uint32_t channel, Value value)
	{
		*(&TIM12->CCR1 + (channel - 1)) = value;
	}

	static inline Value
	getCompareValue(uint32_t channel)
	{
		return *(&TIM12->CCR1 + (channel - 1));
	}

public:
	static void
	enableInterruptVector(bool enable, uint32_t priority);

	static inline void
	enableInterrupt(Interrupt_t interrupt)
	{
		TIM12->DIER |= interrupt.value;
	}

	static inline void
	disableInterrupt(Interrupt_t interrupt)
	{
		TIM12->DIER &= ~interrupt.value;
	}

	static inline InterruptFlag_t
	getEnabledInterrupts()
	{
		return InterruptFlag_t(TIM12->DIER);
	}

	static inline void
	enableDmaRequest(DmaRequestEnable dmaRequests)
	{
		TIM12->DIER |= static_cast<uint32_t>(dmaRequests);
	}

	static inline void
	disableDmaRequest(DmaRequestEnable dmaRequests)
	{
		TIM12->DIER &= ~static_cast<uint32_t>(dmaRequests);
	}

	static inline InterruptFlag_t
	getInterruptFlags()
	{
		return InterruptFlag_t(TIM12->SR);
	}

	static inline void
	acknowledgeInterruptFlags(InterruptFlag_t flags)
	{
		// Flags are cleared by writing a zero to the flag position.
		// Writing a one is ignored.
		TIM12->SR = ~flags.value;
	}
};

}	// namespace platform

}	// namespace modm

#endif // MODM_STM32_TIMER_12_HPP