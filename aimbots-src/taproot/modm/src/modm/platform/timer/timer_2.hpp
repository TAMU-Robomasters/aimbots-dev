/*
 * Copyright (c) 2009, 2011-2012, Georgi Grinshpun
 * Copyright (c) 2009-2012, 2016-2017, Fabian Greif
 * Copyright (c) 2010, Martin Rosekeit
 * Copyright (c) 2011, 2013-2017, Niklas Hauser
 * Copyright (c) 2013-2014, 2016, Kevin Läufer
 * Copyright (c) 2014, 2022, Sascha Schade
 * Copyright (c) 2022, Christopher Durand
 * Copyright (c) 2023, Sergey Pluzhnikov
 *
 * This file is part of the modm project.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
// ----------------------------------------------------------------------------

#ifndef MODM_STM32_TIMER_2_HPP
#define MODM_STM32_TIMER_2_HPP

#include <chrono>
#include <limits>
#include "general_purpose_base.hpp"
#include <modm/platform/gpio/connector.hpp>

namespace modm::platform
{
/**
 * General Purpose Timer 2
 *
 * Interrupt handler:
 * @code
 * MODM_ISR(TIM2)
 * {
 *     Timer2::resetInterruptFlags(Timer2::...);
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
class Timer2 : public GeneralPurposeTimer
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
		// Only available on TIM2/3/4
		CompareOc3Ref 	= TIM_CR2_MMS_2 | TIM_CR2_MMS_1,// 0b110
		CompareOc4Ref 	= TIM_CR2_MMS_2 | TIM_CR2_MMS_1	// 0b111
										| TIM_CR2_MMS_0,
	};

	enum class SlaveModeTrigger : uint32_t
	{
		Internal1 = TIM_SMCR_TS_0,
		Internal2 = TIM_SMCR_TS_1,
		TimerInput1EdgeDetector = TIM_SMCR_TS_2,
		TimerInput1Filtered = TIM_SMCR_TS_2 | TIM_SMCR_TS_0,
		TimerInput2Filtered = TIM_SMCR_TS_2 | TIM_SMCR_TS_1,
		External = TIM_SMCR_TS_2 | TIM_SMCR_TS_1 | TIM_SMCR_TS_0,
	};

	enum class SlaveMode : uint32_t
	{
		/// Slave mode disabled - if CEN = '1' then the prescaler is clocked directly by the internal clock.
		Disabled	= 0,
		/// Counter counts up/down on TI2FP2 edge depending on TI1FP1 level.
		Encoder1	= TIM_SMCR_SMS_0,
		/// Counter counts up/down on TI1FP1 edge depending on TI2FP2 level.
		Encoder2	= TIM_SMCR_SMS_1,
		/// Counter counts up/down on both TI1FP1 and TI2FP2 edges depending on the level of the other input.
		Encoder3	= TIM_SMCR_SMS_1 | TIM_SMCR_SMS_0,
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
	// Timer 2, 5, 23 and 24 are the only ones which have a 32 bit counter
	using Value = uint32_t;

	template< class... Signals >
	static void
	connect()
	{
		using Connector = GpioConnector<Peripheral::Tim2, Signals...>;
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

	static bool
	isEnabled();

	static inline void
	pause()
	{
		TIM2->CR1 &= ~TIM_CR1_CEN;
	}

	static inline void
	start()
	{
		TIM2->CR1 |= TIM_CR1_CEN;
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
		TIM2->PSC = prescaler - 1;
	}

	static uint16_t
	getPrescaler()
	{
		return TIM2->PSC + 1;
	}

	static inline void
	setOverflow(Value overflow)
	{
		TIM2->ARR = overflow;
	}

	static inline Value
	getOverflow()
	{
		return TIM2->ARR;
	}

	template<class SystemClock>
	static constexpr uint32_t
	getClockFrequency()
	{
		return SystemClock::Timer2;
	}

	template<class SystemClock, class Rep, class Period>
	static Value
	setPeriod(std::chrono::duration<Rep, Period> duration, bool autoApply = true)
	{
		// This will be inaccurate for non-smooth frequencies (last six digits unequal to zero)
		const uint32_t cycles = duration.count() * SystemClock::Timer2 * Period::num / Period::den;
		uint16_t prescaler;
		if constexpr (sizeof(Value) > sizeof(uint16_t)) {
			// always round-up
			prescaler = (cycles + static_cast<uint64_t>(std::numeric_limits<Value>::max()) - 1) /
						std::numeric_limits<Value>::max();
		} else {
			// always round-up
			prescaler =
				(cycles + std::numeric_limits<Value>::max() - 1) / std::numeric_limits<Value>::max();
		}
		const Value overflow = cycles / prescaler - 1;

		setPrescaler(prescaler);
		setOverflow(overflow);

		// Generate Update Event to apply the new settings for ARR
		if (autoApply) {
			applyAndReset();
		}

		return overflow;
	}

	/* Returns the frequency of the timer */
	template<class SystemClock>
	static uint32_t
	getTickFrequency()
	{
		return SystemClock::Timer2 / (TIM2->PSC + 1);
	}

	static inline void
	generateEvent(Event ev)
	{
		TIM2->EGR = static_cast<uint32_t>(ev);
	}

	static inline void
	applyAndReset()
	{
		// Generate Update Event to apply the new settings for ARR
		generateEvent(Event::Update);
	}

	static inline Value
	getValue()
	{
		return TIM2->CNT;
	}

	static inline void
	setValue(Value value)
	{
		TIM2->CNT = value;
	}

	static constexpr bool
	hasAdvancedPwmControl()
	{
		return false;
	}

	static inline bool
	isCountingUp()
	{
		return (TIM2->CR1 & TIM_CR1_DIR) == 0;
	}

	static inline bool
	isCountingDown()
	{
		return !isCountingUp();
	}
public:
	static void
	configureInputChannel(uint32_t channel, uint8_t filter);

	template<typename Signal>
	static void
	configureInputChannel(uint8_t filter)
	{
		constexpr auto channel = signalToChannel<Peripheral::Tim2, Signal>();
		configureInputChannel(channel, filter);
	}

	static void
	configureInputChannel(uint32_t channel, InputCaptureMapping input,
			InputCapturePrescaler prescaler,
			InputCapturePolarity polarity, uint8_t filter,
			bool xor_ch1_3=false);

	template<typename Signal>
	static void
	configureInputChannel(InputCaptureMapping input,
			InputCapturePrescaler prescaler,
			InputCapturePolarity polarity, uint8_t filter,
			bool xor_ch1_3=false)
	{
		constexpr auto channel = signalToChannel<Peripheral::Tim2, Signal>();
		configureInputChannel(channel, input, prescaler, polarity, filter, xor_ch1_3);
	}

	static void
	configureOutputChannel(uint32_t channel, OutputCompareMode_t mode,
			Value compareValue, PinState out = PinState::Enable,
			bool enableComparePreload = true);

	template<typename Signal>
	static void
	configureOutputChannel(OutputCompareMode_t mode,
			Value compareValue, PinState out = PinState::Enable,
			bool enableComparePreload = true)
	{
		constexpr auto channel = signalToChannel<Peripheral::Tim2, Signal>();
		configureOutputChannel(channel, mode, compareValue, out, enableComparePreload);
	}

	static void
	configureOutputChannel(uint32_t channel, OutputCompareMode mode,
			Value compareValue, PinState out,
			OutputComparePolarity polarity,
			OutputComparePreload preload = OutputComparePreload::Disable);

	template<typename Signal>
	static void
	configureOutputChannel(OutputCompareMode mode,
			Value compareValue, PinState out,
			OutputComparePolarity polarity,
			OutputComparePreload preload = OutputComparePreload::Disable)
	{
		constexpr auto channel = signalToChannel<Peripheral::Tim2, Signal>();
		configureOutputChannel(channel, mode, compareValue, out, polarity, PinState::Disable, OutputComparePolarity::ActiveHigh, preload);
	}

	/*
	 * Configure Output Channel without changing the Compare Value
	 *
	 * Normally used to reconfigure the Output channel without touching
	 * the compare value. This can e.g. be useful for commutation of a
	 * bldc motor.
	 *
	 * This function probably won't be used for a one time setup but
	 * rather for adjusting the output setting periodically.
	 * Therefore it aims to provide the best performance possible
	 * without sacrificing code readability.
	 */
	static void
	configureOutputChannel(uint32_t channel, OutputCompareMode mode,
			PinState out, OutputComparePolarity polarity,
			PinState out_n,
			OutputComparePolarity polarity_n = OutputComparePolarity::ActiveHigh,
			OutputComparePreload preload = OutputComparePreload::Disable);

	template<typename Signal>
	static void
	configureOutputChannel(OutputCompareMode mode,
			PinState out, OutputComparePolarity polarity,
			PinState out_n,
			OutputComparePolarity polarity_n = OutputComparePolarity::ActiveHigh,
			OutputComparePreload preload = OutputComparePreload::Disable)
	{
		constexpr auto channel = signalToChannel<Peripheral::Tim2, Signal>();
		configureOutputChannel(channel, mode, out, polarity, out_n, polarity_n, preload);
	}

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
				flags |= TIM2->CCMR1 & ~(TIM_CCMR1_OC1M << offset);
				TIM2->CCMR1 = flags;
			}
			else {
				uint32_t offset = 8 * (channel - 2);

				flags <<= offset;
				flags |= TIM2->CCMR2 & ~(TIM_CCMR1_OC1M << offset);

				TIM2->CCMR2 = flags;
			}
		}
	}

	template<typename Signal>
	static void
	setInvertedPwm()
	{
		constexpr auto channel = signalToChannel<Peripheral::Tim2, Signal>();
		setInvertedPwm(channel);
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
				flags |= TIM2->CCMR1 & ~(TIM_CCMR1_OC1M << offset);
				TIM2->CCMR1 = flags;
			}
			else {
				uint32_t offset = 8 * (channel - 2);

				flags <<= offset;
				flags |= TIM2->CCMR2 & ~(TIM_CCMR1_OC1M << offset);

				TIM2->CCMR2 = flags;
			}
		}
	}

	template<typename Signal>
	static void
	setNormalPwm()
	{
		constexpr auto channel = signalToChannel<Peripheral::Tim2, Signal>();
		setNormalPwm(channel);
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
				flags |= TIM2->CCMR1 & ~(TIM_CCMR1_OC1M << offset);
				TIM2->CCMR1 = flags;
			}
			else {
				uint32_t offset = 8 * (channel - 2);

				flags <<= offset;
				flags |= TIM2->CCMR2 & ~(TIM_CCMR1_OC1M << offset);

				TIM2->CCMR2 = flags;
			}
		}
	}

	template<typename Signal>
	static void
	forceInactive()
	{
		constexpr auto channel = signalToChannel<Peripheral::Tim2, Signal>();
		forceInactive(channel);
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
				flags |= TIM2->CCMR1 & ~(TIM_CCMR1_OC1M << offset);
				TIM2->CCMR1 = flags;
			}
			else {
				uint32_t offset = 8 * (channel - 2);

				flags <<= offset;
				flags |= TIM2->CCMR2 & ~(TIM_CCMR1_OC1M << offset);

				TIM2->CCMR2 = flags;
			}
		}
	}

	template<typename Signal>
	static void
	forceActive()
	{
		constexpr auto channel = signalToChannel<Peripheral::Tim2, Signal>();
		forceActive(channel);
	}

	/// Returns if the capture/compare channel of the timer is configured as input.
	///
	/// @param channel may be [1..4]
	/// @return `false` if configured as *output*; `true` if configured as *input*
	static bool
	isChannelConfiguredAsInput(uint32_t channel);

	static inline void
	setCompareValue(uint32_t channel, Value value)
	{
		*(&TIM2->CCR1 + (channel - 1)) = value;
	}


	template<typename Signal>
	static void
	setCompareValue(Value value)
	{
		constexpr auto channel = signalToChannel<Peripheral::Tim2, Signal>();
		setCompareValue(channel, value);
	}

	static inline Value
	getCompareValue(uint32_t channel)
	{
		return *(&TIM2->CCR1 + (channel - 1));
	}

	template<typename Signal>
	static inline Value
	getCompareValue()
	{
		constexpr auto channel = signalToChannel<Peripheral::Tim2, Signal>();
		return getCompareValue(channel);
	}
public:
	static void
	enableInterruptVector(bool enable, uint32_t priority);

	static inline void
	enableInterrupt(Interrupt_t interrupt)
	{
		TIM2->DIER |= interrupt.value;
	}

	static inline void
	disableInterrupt(Interrupt_t interrupt)
	{
		TIM2->DIER &= ~interrupt.value;
	}

	static inline InterruptFlag_t
	getEnabledInterrupts()
	{
		return InterruptFlag_t(TIM2->DIER);
	}

	static inline void
	enableDmaRequest(DmaRequestEnable dmaRequests)
	{
		TIM2->DIER |= static_cast<uint32_t>(dmaRequests);
	}

	static inline void
	disableDmaRequest(DmaRequestEnable dmaRequests)
	{
		TIM2->DIER &= ~static_cast<uint32_t>(dmaRequests);
	}

	static inline InterruptFlag_t
	getInterruptFlags()
	{
		return InterruptFlag_t(TIM2->SR);
	}

	static inline void
	acknowledgeInterruptFlags(InterruptFlag_t flags)
	{
		// Flags are cleared by writing a zero to the flag position.
		// Writing a one is ignored.
		TIM2->SR = ~flags.value;
	}
};

}	// namespace modm::platform

#endif // MODM_STM32_TIMER_2_HPP