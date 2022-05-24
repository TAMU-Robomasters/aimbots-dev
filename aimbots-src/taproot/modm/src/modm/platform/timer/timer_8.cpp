/*
 * Copyright (c) 2009, Martin Rosekeit
 * Copyright (c) 2009-2012, 2017, Fabian Greif
 * Copyright (c) 2011, 2014, Georgi Grinshpun
 * Copyright (c) 2013, 2016, Kevin Läufer
 * Copyright (c) 2014-2017, Niklas Hauser
 *
 * This file is part of the modm project.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
// ----------------------------------------------------------------------------

#include "timer_8.hpp"
#include <modm/platform/clock/rcc.hpp>

// ----------------------------------------------------------------------------
void
modm::platform::Timer8::enable()
{
	Rcc::enable<Peripheral::Tim8>();
}

void
modm::platform::Timer8::disable()
{
	TIM8->CR1 = 0;
	TIM8->DIER = 0;
	TIM8->CCER = 0;

	Rcc::disable<Peripheral::Tim8>();
}

// ----------------------------------------------------------------------------
void
modm::platform::Timer8::setMode(Mode mode, SlaveMode slaveMode,
		SlaveModeTrigger slaveModeTrigger, MasterMode masterMode
		)
{
	// disable timer
	TIM8->CR1 = 0;
	TIM8->CR2 = 0;

	if (slaveMode == SlaveMode::Encoder1 ||
		slaveMode == SlaveMode::Encoder2 ||
		slaveMode == SlaveMode::Encoder3)
	{
		setPrescaler(1);
	}

	// ARR Register is buffered, only Under/Overflow generates update interrupt
	TIM8->CR1 = TIM_CR1_ARPE | TIM_CR1_URS | static_cast<uint32_t>(mode);
	TIM8->CR2 = static_cast<uint32_t>(masterMode);
	TIM8->SMCR = static_cast<uint32_t>(slaveMode) |
						static_cast<uint32_t>(slaveModeTrigger);
}

// ----------------------------------------------------------------------------
void
modm::platform::Timer8::configureInputChannel(uint32_t channel,
		InputCaptureMapping input, InputCapturePrescaler prescaler,
		InputCapturePolarity polarity, uint8_t filter,
		bool xor_ch1_3)
{
	channel -= 1;	// 1..4 -> 0..3

	// disable channel
	TIM8->CCER &= ~((TIM_CCER_CC1NP | TIM_CCER_CC1P | TIM_CCER_CC1E) << (channel * 4));

	uint32_t flags = static_cast<uint32_t>(input);
	flags |= static_cast<uint32_t>(prescaler) << 2;
	flags |= static_cast<uint32_t>(filter&0xf) << 4;

	if (channel <= 1)
	{
		uint32_t offset = 8 * channel;

		flags <<= offset;
		flags |= TIM8->CCMR1 & ~(0xff << offset);

		TIM8->CCMR1 = flags;

		if(channel == 0) {
			if(xor_ch1_3)
				TIM8->CR2 |= TIM_CR2_TI1S;
			else
				TIM8->CR2 &= ~TIM_CR2_TI1S;
		}
	}
	else {
		uint32_t offset = 8 * (channel - 2);

		flags <<= offset;
		flags |= TIM8->CCMR2 & ~(0xff << offset);

		TIM8->CCMR2 = flags;
	}

	TIM8->CCER |= (TIM_CCER_CC1E | static_cast<uint32_t>(polarity)) << (channel * 4);
}

// ----------------------------------------------------------------------------
void
modm::platform::Timer8::configureOutputChannel(uint32_t channel,
		OutputCompareMode mode, uint16_t compareValue)
{
	channel -= 1;	// 1..4 -> 0..3

	// disable output
	TIM8->CCER &= ~(0xf << (channel * 4));

	setCompareValue(channel + 1, compareValue);

	// enable preload (the compare value is loaded at each update event)
	uint32_t flags = static_cast<uint32_t>(mode) | TIM_CCMR1_OC1PE;

	if (channel <= 1)
	{
		uint32_t offset = 8 * channel;

		flags <<= offset;
		flags |= TIM8->CCMR1 & ~(0xff << offset);

		TIM8->CCMR1 = flags;
	}
	else {
		uint32_t offset = 8 * (channel - 2);

		flags <<= offset;
		flags |= TIM8->CCMR2 & ~(0xff << offset);

		TIM8->CCMR2 = flags;
	}

	// Disable Repetition Counter (FIXME has to be done here for some unknown reason)
	TIM8->RCR = 0;

	if (mode != OutputCompareMode::Inactive) {
		TIM8->CCER |= (TIM_CCER_CC1E) << (channel * 4);
	}
}

void
modm::platform::Timer8::configureOutputChannel(uint32_t channel,
OutputCompareMode mode,
PinState out, OutputComparePolarity polarity,
PinState out_n, OutputComparePolarity polarity_n,
OutputComparePreload preload)
{
	channel -= 1;	// 1..4 -> 0..3

	// disable output
	TIM8->CCER &= ~(0xf << (channel * 4));

	uint32_t flags = static_cast<uint32_t>(mode) | static_cast<uint32_t>(preload);

	if (channel <= 1)
	{
		uint32_t offset = 8 * channel;

		flags <<= offset;
		flags |= TIM8->CCMR1 & ~(0xff << offset);

		TIM8->CCMR1 = flags;
	}
	else {
		uint32_t offset = 8 * (channel - 2);

		flags <<= offset;
		flags |= TIM8->CCMR2 & ~(0xff << offset);

		TIM8->CCMR2 = flags;
	}

	// Disable Repetition Counter (FIXME has to be done here for some unknown reason)
	TIM8->RCR = 0;

	// CCER Flags (Enable/Polarity)
	flags = (static_cast<uint32_t>(polarity_n) << 2) |
			(static_cast<uint32_t>(out_n)      << 2) |
			 static_cast<uint32_t>(polarity) | static_cast<uint32_t>(out);

	TIM8->CCER |= flags << (channel * 4);
}

void
modm::platform::Timer8::configureOutputChannel(uint32_t channel,
uint32_t modeOutputPorts)
{
	channel -= 1;	// 1..4 -> 0..3

	{
		uint32_t flags = modeOutputPorts & (0x70);

		if (channel <= 1)
		{
			uint32_t offset = 8 * channel;

			flags <<= offset;
			flags |= TIM8->CCMR1 & ~(TIM_CCMR1_OC1M << offset);
			TIM8->CCMR1 = flags;
		}
		else {
			uint32_t offset = 8 * (channel - 2);

			flags <<= offset;
			flags |= TIM8->CCMR2 & ~(TIM_CCMR1_OC1M << offset);

			TIM8->CCMR2 = flags;
		}
	}

	// Disable Repetition Counter (FIXME has to be done here for some unknown reason)
	TIM8->RCR = 0;

	uint32_t flags = (modeOutputPorts & (0xf)) << (channel * 4);
	flags |= TIM8->CCER & ~(0xf << (channel * 4));
	TIM8->CCER = flags;
}

// ----------------------------------------------------------------------------
void
modm::platform::Timer8::enableInterruptVector(Interrupt interrupt, bool enable, uint32_t priority)
{
	if(interrupt & (Interrupt::Break))
	{
		if (enable)
		{
			NVIC_SetPriority(TIM8_BRK_TIM12_IRQn, priority);
			NVIC_EnableIRQ(TIM8_BRK_TIM12_IRQn);
		}
		else
		{
			NVIC_DisableIRQ(TIM8_BRK_TIM12_IRQn);
		}
	}
	if(interrupt & (Interrupt::Update))
	{
		if (enable)
		{
			NVIC_SetPriority(TIM8_UP_TIM13_IRQn, priority);
			NVIC_EnableIRQ(TIM8_UP_TIM13_IRQn);
		}
		else
		{
			NVIC_DisableIRQ(TIM8_UP_TIM13_IRQn);
		}
	}
	if(interrupt & (Interrupt::Trigger | Interrupt::COM))
	{
		if (enable)
		{
			NVIC_SetPriority(TIM8_TRG_COM_TIM14_IRQn, priority);
			NVIC_EnableIRQ(TIM8_TRG_COM_TIM14_IRQn);
		}
		else
		{
			NVIC_DisableIRQ(TIM8_TRG_COM_TIM14_IRQn);
		}
	}
	if(interrupt & (Interrupt::CaptureCompare1 | Interrupt::CaptureCompare2 | Interrupt::CaptureCompare3 | Interrupt::CaptureCompare4))
	{
		if (enable)
		{
			NVIC_SetPriority(TIM8_CC_IRQn, priority);
			NVIC_EnableIRQ(TIM8_CC_IRQn);
		}
		else
		{
			NVIC_DisableIRQ(TIM8_CC_IRQn);
		}
	}
}