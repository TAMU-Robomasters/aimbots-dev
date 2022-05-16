/*
 * Copyright (c) 2013-2014, Kevin Läufer
 * Copyright (c) 2014-2017, Niklas Hauser
 * Copyright (c) 2016, Fabian Greif
 *
 * This file is part of the modm project.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
// ----------------------------------------------------------------------------

#ifndef MODM_STM32_ADC1_HPP
#	error 	"Don't include this file directly, use 'adc_1.hpp' instead!"
#endif

#include <modm/platform/clock/rcc.hpp>
#include <modm/math/algorithm/prescaler.hpp>

template< class SystemClock, modm::frequency_t frequency, modm::percent_t tolerance >
void
modm::platform::Adc1::initialize()
{
	constexpr auto result = modm::Prescaler::from_list(SystemClock::Adc, frequency, {2,4,6,8});
	static_assert(result.frequency <= 36000000, "Generated ADC frequency is above maximum frequency!");
	assertBaudrateInTolerance<result.frequency, frequency, tolerance >();

	Rcc::enable<Peripheral::Adc1>();
	ADC1->CR2 |= ADC_CR2_ADON;			// switch on ADC

	setPrescaler(Prescaler{result.index});
}

void
modm::platform::Adc1::setPrescaler(const Prescaler prescaler)
{
	ADC->CCR = (ADC->CCR & ~ADC_CCR_ADCPRE) | (uint32_t(prescaler) << ADC_CCR_ADCPRE_Pos);
}

void
modm::platform::Adc1::enableTemperatureRefVMeasurement()
{
	ADC->CCR |= ADC_CCR_TSVREFE;
}

void
modm::platform::Adc1::disableTemperatureRefVMeasurement()
{
	ADC->CCR &= ~ADC_CCR_TSVREFE;
}
void
modm::platform::Adc1::setLeftAdjustResult()
{
	ADC1->CR2 |= ADC_CR2_ALIGN;
}

void
modm::platform::Adc1::setRightAdjustResult()
{
	ADC1->CR2 &= ~ADC_CR2_ALIGN;
}

bool
modm::platform::Adc1::setChannel(const Channel channel,
									 const SampleTime sampleTime)
{
	if (uint32_t(channel) > 18) return false;
	// clear number of conversions in the sequence
	// and set number of conversions to 1
	ADC1->SQR1 = 0;
	ADC1->SQR2 = 0;
	ADC1->SQR3 = uint32_t(channel) & 0x1f;

	setSampleTime(channel, sampleTime);
	return true;
}

modm::platform::Adc1::Channel
modm::platform::Adc1::getChannel()
{
	return Channel(ADC1->SQR3 & 0x1f);
}

bool
modm::platform::Adc1::addChannel(const Channel channel,
									const SampleTime sampleTime)
{
	// read channel count
	uint8_t channel_count = (ADC1->SQR1 & ADC_SQR1_L) >> 20;
	++channel_count;
	if(channel_count > 0x0f) return false; // emergency exit
	// write channel number
	if(channel_count < 6) {
		ADC1->SQR3 |=
			(uint32_t(channel) & 0x1f) << (channel_count*5);
	} else 	if(channel_count < 12) {
		ADC1->SQR2 |=
			(uint32_t(channel) & 0x1f) << ((channel_count-6)*5);
	} else {
		ADC1->SQR1 |=
			(uint32_t(channel) & 0x1f) << ((channel_count-12)*5);
	}
	// update channel count
	ADC1->SQR1 = (ADC1->SQR1 & ~ADC_SQR1_L) | (channel_count << 20);

	setSampleTime(channel, sampleTime);
	return true;
}

void
modm::platform::Adc1::setSampleTime(const Channel channel,
										const SampleTime sampleTime)
{
	if (uint32_t(channel) < 10) {
		ADC1->SMPR2 |= uint32_t(sampleTime)
								<< (uint32_t(channel) * 3);
	}
	else {
		ADC1->SMPR1 |= uint32_t(sampleTime)
								<< ((uint32_t(channel)-10) * 3);
	}
}

void
modm::platform::Adc1::enableFreeRunningMode()
{
	ADC1->CR2 |= ADC_CR2_CONT;	// set to continuous mode
}

void
modm::platform::Adc1::disableFreeRunningMode()
{
	ADC1->CR2 &= ~ADC_CR2_CONT;		// set to single mode
}

void
modm::platform::Adc1::disable()
{
	ADC1->CR2 &= ~(ADC_CR2_ADON);		// switch off ADC
	RCC->APB2ENR &= ~RCC_APB2ENR_ADC1EN; // stop ADC Clock
}

void
modm::platform::Adc1::startConversion()
{
	acknowledgeInterruptFlags(InterruptFlag::All);
	// starts single conversion for the regular group
	ADC1->CR2 |= ADC_CR2_SWSTART;
}

bool
modm::platform::Adc1::isConversionFinished()
{
	return (ADC1->SR & ADC_SR_EOC);
}

uint16_t
modm::platform::Adc1::getValue()
{
	return ADC1->DR;
}


uint16_t
modm::platform::Adc1::readChannel(Channel channel)
{
	if (!setChannel(channel)) return 0;

	startConversion();
	// wait until the conversion is finished
	while (!isConversionFinished())
		;

	return getValue();
}

// ----------------------------------------------------------------------------
void
modm::platform::Adc1::enableInterruptVector(const uint32_t priority,
												   const bool enable)
{
	const IRQn_Type InterruptVector = ADC_IRQn;
	if (enable) {
		NVIC_SetPriority(InterruptVector, priority);
		NVIC_EnableIRQ(InterruptVector);
	} else {
		NVIC_DisableIRQ(InterruptVector);
	}
}

void
modm::platform::Adc1::enableInterrupt(const Interrupt_t interrupt)
{
	ADC1->CR1 |= interrupt.value;
}

void
modm::platform::Adc1::disableInterrupt(const Interrupt_t interrupt)
{
	ADC1->CR1 &= ~interrupt.value;
}

modm::platform::Adc1::InterruptFlag_t
modm::platform::Adc1::getInterruptFlags()
{
	return InterruptFlag_t(ADC1->SR);
}

void
modm::platform::Adc1::acknowledgeInterruptFlags(const InterruptFlag_t flags)
{
	// Flags are cleared by writing a one to the flag position.
	// Writing a zero is ignored.
	ADC1->SR = flags.value;
}