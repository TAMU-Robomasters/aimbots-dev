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

#include "timer_1.hpp"
#include <modm/platform/clock/rcc.hpp>

// ----------------------------------------------------------------------------
void
modm::platform::Timer1::enable()
{
	Rcc::enable<Peripheral::Tim1>();
}

void
modm::platform::Timer1::disable()
{
	TIM1->CR1 = 0;
	TIM1->DIER = 0;
	TIM1->CCER = 0;

	Rcc::disable<Peripheral::Tim1>();
}

bool
modm::platform::Timer1::isEnabled()
{
	return Rcc::isEnabled<Peripheral::Tim1>();
}

// ----------------------------------------------------------------------------
void
modm::platform::Timer1::setMode(Mode mode, SlaveMode slaveMode,
		SlaveModeTrigger slaveModeTrigger, MasterMode masterMode,
		bool enableOnePulseMode
		, MasterMode2 masterMode2
		)
{
	// disable timer
	TIM1->CR1 = 0;
	TIM1->CR2 = 0;

	if (slaveMode == SlaveMode::Encoder1 ||
		slaveMode == SlaveMode::Encoder2 ||
		slaveMode == SlaveMode::Encoder3)
	{
		setPrescaler(1);
	}

	// ARR Register is buffered, only Under/Overflow generates update interrupt
	uint32_t cr1 = TIM_CR1_ARPE | TIM_CR1_URS | static_cast<uint32_t>(mode);
	if (enableOnePulseMode) {
		TIM1->CR1 = cr1 | TIM_CR1_OPM;
	} else {
		TIM1->CR1 = cr1;
	}
	TIM1->CR2 = 	static_cast<uint32_t>(masterMode) |
						static_cast<uint32_t>(masterMode2);
	TIM1->SMCR = static_cast<uint32_t>(slaveMode) |
						static_cast<uint32_t>(slaveModeTrigger);
}

// ----------------------------------------------------------------------------
void
modm::platform::Timer1::configureInputChannel(uint32_t channel, uint8_t filter) {
		channel -= 1;	// 1..4 -> 0..3

	// disable channel
	TIM1->CCER &= ~(TIM_CCER_CC1E << (channel * 4));

	uint32_t flags = static_cast<uint32_t>(filter&0xf) << 4;

	if (channel <= 1)
	{
		const uint32_t offset = 8 * channel;

		flags <<= offset;
		flags |= TIM1->CCMR1 & ~(0xf0 << offset);

		TIM1->CCMR1 = flags;
	}
	else {
		const uint32_t offset = 8 * (channel - 2);

		flags <<= offset;
		flags |= TIM1->CCMR2 & ~(0xf0 << offset);

		TIM1->CCMR2 = flags;
	}
	TIM1->CCER |= TIM_CCER_CC1E << (channel * 4);
}

void
modm::platform::Timer1::configureInputChannel(uint32_t channel,
		InputCaptureMapping input, InputCapturePrescaler prescaler,
		InputCapturePolarity polarity, uint8_t filter,
		bool xor_ch1_3)
{
	channel -= 1;	// 1..4 -> 0..3

	// disable channel
	TIM1->CCER &= ~((TIM_CCER_CC1NP | TIM_CCER_CC1P | TIM_CCER_CC1E) << (channel * 4));

	uint32_t flags = static_cast<uint32_t>(input);
	flags |= static_cast<uint32_t>(prescaler) << 2;
	flags |= static_cast<uint32_t>(filter&0xf) << 4;

	if (channel <= 1)
	{
		const uint32_t offset = 8 * channel;

		flags <<= offset;
		flags |= TIM1->CCMR1 & ~(0xff << offset);

		TIM1->CCMR1 = flags;

		if(channel == 0) {
			if(xor_ch1_3)
				TIM1->CR2 |= TIM_CR2_TI1S;
			else
				TIM1->CR2 &= ~TIM_CR2_TI1S;
		}
	}
	else if (channel <= 3) {
		const uint32_t offset = 8 * (channel - 2);

		flags <<= offset;
		flags |= TIM1->CCMR2 & ~(0xff << offset);

		TIM1->CCMR2 = flags;
	}
	else {
		const uint32_t offset = 8 * (channel - 4);

		flags <<= offset;
		flags |= TIM1->CCMR3 & ~(0xff << offset);

		TIM1->CCMR3 = flags;
	}
	TIM1->CCER |= (TIM_CCER_CC1E | static_cast<uint32_t>(polarity)) << (channel * 4);
}

// ----------------------------------------------------------------------------
void
modm::platform::Timer1::configureOutputChannel(uint32_t channel,
		OutputCompareMode mode, Value compareValue, PinState out)
{
	channel -= 1;	// 1..4 -> 0..3

	// disable output
	TIM1->CCER &= ~(0xf << (channel * 4));

	setCompareValue(channel + 1, compareValue);

	// enable preload (the compare value is loaded at each update event)
	uint32_t flags = static_cast<uint32_t>(mode) | TIM_CCMR1_OC1PE;

	if (channel <= 1)
	{
		const uint32_t offset = 8 * channel;

		flags <<= offset;
		flags |= TIM1->CCMR1 & ~(0xff << offset);

		TIM1->CCMR1 = flags;
	}
	else if (channel <= 3) {
		const uint32_t offset = 8 * (channel - 2);

		flags <<= offset;
		flags |= TIM1->CCMR2 & ~(0xff << offset);

		TIM1->CCMR2 = flags;
	}
	else {
		const uint32_t offset = 8 * (channel - 4);

		flags <<= offset;
		flags |= TIM1->CCMR3 & ~(0xff << offset);

		TIM1->CCMR3 = flags;
	}
	if ((mode != OutputCompareMode::Inactive) and (out == PinState::Enable)) {
		TIM1->CCER |= (TIM_CCER_CC1E) << (channel * 4);
	}
}

void
modm::platform::Timer1::configureOutputChannel(uint32_t channel,
OutputCompareMode mode, Value compareValue,
PinState out, OutputComparePolarity polarity,
PinState out_n, OutputComparePolarity polarity_n,
OutputComparePreload preload)
{
	// disable output
	TIM1->CCER &= ~(0xf << ((channel-1) * 4));
	setCompareValue(channel, compareValue);
	configureOutputChannel(channel, mode, out, polarity, out_n, polarity_n, preload);
}

void
modm::platform::Timer1::configureOutputChannel(uint32_t channel,
OutputCompareMode mode,
PinState out, OutputComparePolarity polarity,
PinState out_n, OutputComparePolarity polarity_n,
OutputComparePreload preload)
{
	channel -= 1;	// 1..4 -> 0..3

	// disable output
	TIM1->CCER &= ~(0xf << (channel * 4));

	uint32_t flags = static_cast<uint32_t>(mode) | static_cast<uint32_t>(preload);

	if (channel <= 1)
	{
		const uint32_t offset = 8 * channel;

		flags <<= offset;
		flags |= TIM1->CCMR1 & ~(0xff << offset);

		TIM1->CCMR1 = flags;
	}
	else if (channel <= 3) {
		const uint32_t offset = 8 * (channel - 2);

		flags <<= offset;
		flags |= TIM1->CCMR2 & ~(0xff << offset);

		TIM1->CCMR2 = flags;
	}
	else {
		const uint32_t offset = 8 * (channel - 4);

		flags <<= offset;
		flags |= TIM1->CCMR3 & ~(0xff << offset);

		TIM1->CCMR3 = flags;
	}
	// CCER Flags (Enable/Polarity)
	flags = (static_cast<uint32_t>(polarity_n) << 2) |
			(static_cast<uint32_t>(out_n)      << 2) |
			 static_cast<uint32_t>(polarity) | static_cast<uint32_t>(out);

	TIM1->CCER |= flags << (channel * 4);
}

void
modm::platform::Timer1::configureOutputChannel(uint32_t channel,
uint32_t modeOutputPorts)
{
	channel -= 1;	// 1..4 -> 0..3

	{
		uint32_t flags = modeOutputPorts & (0x70);

		if (channel <= 1)
		{
			uint32_t offset = 8 * channel;

			flags <<= offset;
			flags |= TIM1->CCMR1 & ~(TIM_CCMR1_OC1M << offset);
			TIM1->CCMR1 = flags;
		}
		else if (channel <= 3) {
			uint32_t offset = 8 * (channel - 2);

			flags <<= offset;
			flags |= TIM1->CCMR2 & ~(TIM_CCMR1_OC1M << offset);

			TIM1->CCMR2 = flags;
		}
		else {
			uint32_t offset = 8 * (channel - 4);

			flags <<= offset;
			flags |= TIM1->CCMR3 & ~(TIM_CCMR1_OC1M << offset);

			TIM1->CCMR3 = flags;
		}
	}

	uint32_t flags = (modeOutputPorts & (0xf)) << (channel * 4);
	flags |= TIM1->CCER & ~(0xf << (channel * 4));
	TIM1->CCER = flags;
}

// ----------------------------------------------------------------------------
void
modm::platform::Timer1::enableInterruptVector(Interrupt interrupt, bool enable, uint32_t priority)
{
	if(interrupt & (Interrupt::Break))
	{
		if (enable)
		{
			NVIC_SetPriority(TIM1_BRK_IRQn, priority);
			NVIC_EnableIRQ(TIM1_BRK_IRQn);
		}
		else
		{
			NVIC_DisableIRQ(TIM1_BRK_IRQn);
		}
	}
	if(interrupt & (Interrupt::Update))
	{
		if (enable)
		{
			NVIC_SetPriority(TIM1_UP_IRQn, priority);
			NVIC_EnableIRQ(TIM1_UP_IRQn);
		}
		else
		{
			NVIC_DisableIRQ(TIM1_UP_IRQn);
		}
	}
	if(interrupt & (Interrupt::Trigger | Interrupt::COM))
	{
		if (enable)
		{
			NVIC_SetPriority(TIM1_TRG_COM_IRQn, priority);
			NVIC_EnableIRQ(TIM1_TRG_COM_IRQn);
		}
		else
		{
			NVIC_DisableIRQ(TIM1_TRG_COM_IRQn);
		}
	}
	if(interrupt & (Interrupt::CaptureCompare1 | Interrupt::CaptureCompare2 | Interrupt::CaptureCompare3 | Interrupt::CaptureCompare4))
	{
		if (enable)
		{
			NVIC_SetPriority(TIM1_CC_IRQn, priority);
			NVIC_EnableIRQ(TIM1_CC_IRQn);
		}
		else
		{
			NVIC_DisableIRQ(TIM1_CC_IRQn);
		}
	}
}