/*
 * Copyright (c) 2013-2014, Kevin Läufer
 * Copyright (c) 2014, Sascha Schade
 * Copyright (c) 2014, 2016-2017, Niklas Hauser
 * Copyright (c) 2023, Christopher Durand
 *
 * This file is part of the modm project.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
// ----------------------------------------------------------------------------

#ifndef MODM_STM32F3_ADC1_HPP
#	error 	"Don't include this file directly, use 'adc_1.hpp' instead!"
#endif

#include <modm/architecture/interface/delay.hpp>	// modm::delayMicroseconds
#include <modm/platform/clock/rcc.hpp>

void
modm::platform::Adc1::initialize(const ClockMode clk,
		const ClockSource clk_src,
		const Prescaler pre,
		const CalibrationMode cal, const bool blocking)
{
	uint32_t tmp = 0;

	// enable clock
	RCC->AHB1ENR |= RCC_AHB1ENR_ADC12EN;
	// select clock source
	RCC->D3CCIPR |= static_cast<uint32_t>(clk_src);
	// Disable deep power down
	ADC1->CR &= ~ADC_CR_DEEPPWD;
	// reset ADC
	// FIXME: not a good idea since you can only reset both
	// ADC1/ADC2 or ADC3/ADC4 at once ....

	// set ADC "analog" clock source
	if (clk != ClockMode::DoNotChange) {
		if (clk == ClockMode::Asynchronous) {
			setPrescaler(pre);
		}
		tmp  =  ADC12_COMMON->CCR;
		tmp &= ~ADC_CCR_CKMODE;
		tmp |=  static_cast<uint32_t>(clk);
		ADC12_COMMON->CCR = tmp;
	}

	// Always set highest boost mode for maximum clock for now.
	// This will increase the ADC power consumption on lower ADC clocks.
	// TODO: compute ADC input clock and choose correct setting
	ADC1->CR |= ADC_CR_BOOST_Msk;
	// enable regulator
	ADC1->CR &= ~ADC_CR_ADVREGEN;
	ADC1->CR |= static_cast<uint32_t>(VoltageRegulatorState::Enabled);
	modm::delay_us(10);	// FIXME: this is ugly -> find better solution

	while(!(ADC1->ISR & ADC_ISR_LDORDY));
	acknowledgeInterruptFlags(InterruptFlag::Ready);

	calibrate(cal, true);	// blocking calibration

	ADC1->CR |= ADC_CR_ADEN;
	if (blocking) {
		// ADEN can only be set 4 ADC clock cycles after ADC_CR_ADCAL gets
		// cleared. Setting it in a loop ensures the flag gets set for ADC
		// clocks slower than the CPU clock.
		while (not isReady()) {
			ADC1->CR |= ADC_CR_ADEN;
		}
		acknowledgeInterruptFlags(InterruptFlag::Ready);
	}
}

void
modm::platform::Adc1::disable(const bool blocking)
{
	ADC1->CR |= ADC_CR_ADDIS;
	if (blocking) {
		// wait for ADC_CR_ADDIS to be cleared by hw
		while(ADC1->CR & ADC_CR_ADDIS);
	}
	// disable clock
	RCC->AHB1ENR &= ~RCC_AHB1ENR_ADC12EN;
}

void
modm::platform::Adc1::setPrescaler(const Prescaler pre)
{
	uint32_t tmp;
	tmp  = ADC12_COMMON->CCR;
	tmp &= ~static_cast<uint32_t>(Prescaler::Div256AllBits);
	tmp |=  static_cast<uint32_t>(pre);
	ADC12_COMMON->CCR = tmp;
}

bool
modm::platform::Adc1::isReady()
{
	return static_cast<bool>(getInterruptFlags() & InterruptFlag::Ready);
}

void
modm::platform::Adc1::calibrate(const CalibrationMode mode,
									const bool blocking)
{
	if (mode != CalibrationMode::DoNotCalibrate) {
		ADC1->CR |= ADC_CR_ADCAL | ADC_CR_ADCALLIN |
										static_cast<uint32_t>(mode);
		if(blocking) {
			// wait for ADC_CR_ADCAL to be cleared by hw
			while(ADC1->CR & ADC_CR_ADCAL);
		}
	}
}

bool
modm::platform::Adc1::configureChannel(Channel channel,
											  SampleTime sampleTime)
{
	if (static_cast<uint8_t>(channel) > 19) {
		return false;
	}
	ADC1->PCSEL_RES0 |= (1u << static_cast<uint8_t>(channel));
	uint32_t tmpreg = 0;
	if (static_cast<uint8_t>(channel) < 10) {
		tmpreg = ADC1->SMPR1
			& ~((ADC_SMPR1_SMP0) << (static_cast<uint8_t>(channel) * 3));
		tmpreg |= static_cast<uint32_t>(sampleTime) <<
						(static_cast<uint8_t>(channel) * 3);
		ADC1->SMPR1 = tmpreg;
	}
	else {
		tmpreg = ADC1->SMPR2
			& ~((ADC_SMPR2_SMP10) << ((static_cast<uint8_t>(channel)-10) * 3));
		tmpreg |= static_cast<uint32_t>(sampleTime) <<
						((static_cast<uint8_t>(channel)-10) * 3);
		ADC1->SMPR2 = tmpreg;
	}
	return true;

}

bool
modm::platform::Adc1::setChannel(Channel channel,
										SampleTime sampleTime)
{
	if (!configureChannel(channel, sampleTime)) {
		return false;
	}

	ADC1->SQR1 = (static_cast<uint8_t>(channel) << ADC_SQR1_SQ1_Pos) & ADC_SQR1_SQ1_Msk;
	return true;
}

bool
modm::platform::Adc1::setChannelSequence(std::span<const SequenceChannel> sequence)
{
	if (sequence.size() > 16 || sequence.size() == 0) {
		return false;
	}

	ADC1->SQR1 = sequence.size() - 1;

	for (const auto [i, config] : modm::enumerate(sequence)) {
		const auto [channel, sampleTime] = config;
		if (!configureChannel(channel, sampleTime)) {
			return false;
		}

		if (i < 4) {
			const auto shift = (i + 1) * 6;
			const auto mask = 0b111111u << shift;
			ADC1->SQR1 = (ADC1->SQR1 & ~mask) | (std::to_underlying(channel) << shift);
		} else if (i < 9) {
			const auto shift = (i - 4) * 6;
			const auto mask = 0b111111u << shift;
			ADC1->SQR2 = (ADC1->SQR2 & ~mask) | (std::to_underlying(channel) << shift);
		} else if (i < 14) {
			const auto shift = (i - 9) * 6;
			const auto mask = 0b111111u << shift;
			ADC1->SQR3 = (ADC1->SQR3 & ~mask) | (std::to_underlying(channel) << shift);
		} else {
			const auto shift = (i - 14) * 6;
			const auto mask = 0b111111u << shift;
			ADC1->SQR4 = (ADC1->SQR4 & ~mask) | (std::to_underlying(channel) << shift);
		}
	}
	return true;
}

void
modm::platform::Adc1::setFreeRunningMode(const bool enable)
{
	if (enable) {
		ADC1->CFGR |=  ADC_CFGR_CONT; // set to continuous mode
	} else {
		ADC1->CFGR &= ~ADC_CFGR_CONT; // set to single mode
	}
}

void
modm::platform::Adc1::startConversion()
{
	// TODO: maybe add more interrupt flags
	acknowledgeInterruptFlags(InterruptFlag::EndOfRegularConversion |
			InterruptFlag::EndOfSampling | InterruptFlag::Overrun |
			InterruptFlag::EndOfRegularSequenceOfConversions);
	// starts single conversion for the regular group
	ADC1->CR |= ADC_CR_ADSTART;
}

void
modm::platform::Adc1::stopConversion()
{
	ADC1->CR |= ADC_CR_ADSTP | ADC_CR_JADSTP;
	while ((ADC1->CR & (ADC_CR_ADSTP | ADC_CR_JADSTP)) != 0);

	acknowledgeInterruptFlags(InterruptFlag::EndOfRegularConversion |
			InterruptFlag::EndOfSampling | InterruptFlag::Overrun);
}

bool
modm::platform::Adc1::isConversionFinished()
{
	return static_cast<bool>(getInterruptFlags() & InterruptFlag::EndOfRegularConversion);
}

bool
modm::platform::Adc1::isConversionSequenceFinished()
{
	return static_cast<bool>(getInterruptFlags() & InterruptFlag::EndOfRegularSequenceOfConversions);
}

void
modm::platform::Adc1::enableRegularConversionExternalTrigger(
	ExternalTriggerPolarity externalTriggerPolarity,
	ExternalTriggerEvent externalTriggerEvent)
{
	const auto polarity = (static_cast<uint32_t>(externalTriggerPolarity) << ADC_CFGR_EXTEN_Pos);
	const auto externalTrigger = (static_cast<uint32_t>(externalTriggerEvent) << ADC_CFGR_EXTSEL_Pos);
	const auto mask = ADC_CFGR_EXTEN_Msk | ADC_CFGR_EXTSEL_Msk;
	ADC1->CFGR = (ADC1->CFGR & ~mask) | polarity | externalTrigger;
}

void
modm::platform::Adc1::disableRegularConversionExternalTrigger()
{
	// Disable regular conversions external trigger by clearing the bits
	// for polarity and external trigger source.
	ADC1->CFGR &= ~(ADC_CFGR_EXTEN_Msk | ADC_CFGR_EXTSEL_Msk);
}

void
modm::platform::Adc1::startInjectedConversionSequence()
{
	acknowledgeInterruptFlags(InterruptFlag::EndOfInjectedConversion |
			InterruptFlag::EndOfInjectedSequenceOfConversions);

	ADC1->CR |= ADC_CR_JADSTART;
}

bool
modm::platform::Adc1::setInjectedConversionChannel(uint8_t index, Channel channel,
														  SampleTime sampleTime)
{
	if (index >= 4) {
		return false;
	}
	if (!configureChannel(channel, sampleTime)) {
		return false;
	}

	static_assert(ADC_JSQR_JSQ2_Pos == ADC_JSQR_JSQ1_Pos + 6);
	static_assert(ADC_JSQR_JSQ3_Pos == ADC_JSQR_JSQ2_Pos + 6);
	static_assert(ADC_JSQR_JSQ4_Pos == ADC_JSQR_JSQ3_Pos + 6);

	const uint32_t pos = ADC_JSQR_JSQ1_Pos + 6 * index;
	const uint32_t mask = ADC_JSQR_JSQ1_Msk << (6 * index);
	ADC1->JSQR = (ADC1->JSQR & ~mask) | (static_cast<uint32_t>(channel) << pos);
	return true;
}

template<class Gpio>
bool
modm::platform::Adc1::setInjectedConversionChannel(uint8_t index,
														  SampleTime sampleTime)
{
	return setInjectedConversionChannel(index, getPinChannel<Gpio>(), sampleTime);
}

bool
modm::platform::Adc1::setInjectedConversionSequenceLength(uint8_t length)
{
	if (length < 1 or length > 4) {
		return false;
	}
	ADC1->JSQR = (ADC1->JSQR & ~ADC_JSQR_JL)
		| ((length - 1) << ADC_JSQR_JL_Pos);
	return true;
}

void
modm::platform::Adc1::enableInjectedConversionExternalTrigger(
	ExternalTriggerPolarity externalTriggerPolarity,
	ExternalTriggerEvent externalTriggerEvent)
{
	const auto polarity = (static_cast<uint32_t>(externalTriggerPolarity) << ADC_JSQR_JEXTEN_Pos);
	const auto externalTrigger = (static_cast<uint32_t>(externalTriggerEvent) << ADC_JSQR_JEXTSEL_Pos);
	const auto mask = ADC_JSQR_JEXTEN_Msk | ADC_JSQR_JEXTSEL_Msk;
	ADC1->JSQR = (ADC1->JSQR & ~mask) | polarity | externalTrigger;
}

void
modm::platform::Adc1::disableInjectedConversionExternalTrigger()
{
	// Disable injected conversions external trigger by clearing the bits
	// for polarity and external trigger source.
	ADC1->JSQR &= ~(ADC_JSQR_JEXTEN_Msk | ADC_JSQR_JEXTSEL_Msk);
}

bool
modm::platform::Adc1::isInjectedConversionFinished()
{
	return static_cast<bool>(getInterruptFlags() & InterruptFlag::EndOfInjectedSequenceOfConversions);
}

void
modm::platform::Adc1::setDmaMode(DmaMode mode)
{
	constexpr uint32_t mask = std::to_underlying(DmaMode::Mask);
	ADC1->CFGR = (ADC1->CFGR & ~mask) | std::to_underlying(mode);
}

// ----------------------------------------------------------------------------
void
modm::platform::Adc1::enableInterruptVector(const uint32_t priority,
												const bool enable)
{
	const IRQn_Type INTERRUPT_VECTOR = ADC_IRQn;
	if (enable) {
		NVIC_SetPriority(INTERRUPT_VECTOR, priority);
		NVIC_EnableIRQ(INTERRUPT_VECTOR);
	} else {
		NVIC_DisableIRQ(INTERRUPT_VECTOR);
	}
}

void
modm::platform::Adc1::enableInterrupt(const Interrupt_t interrupt)
{
	ADC1->IER |= interrupt.value;
}

void
modm::platform::Adc1::disableInterrupt(const Interrupt_t interrupt)
{
	ADC1->IER &= ~interrupt.value;
}

modm::platform::Adc1::InterruptFlag_t
modm::platform::Adc1::getInterruptFlags()
{
	return InterruptFlag_t(ADC1->ISR);
}

void
modm::platform::Adc1::acknowledgeInterruptFlags(const InterruptFlag_t flags)
{
	// Flags are cleared by writing a one to the flag position.
	// Writing a zero is ignored.
	ADC1->ISR = flags.value;
}

bool
modm::platform::Adc1::enableChannelOffset(const OffsetSlot slot, const Channel channel, const uint16_t offset)
{
	const uint32_t offsetMask = (offset << ADC_OFR1_OFFSET1_Pos) & ADC_OFR1_OFFSET1_Msk;
	const uint32_t signMask = 0u;
	const uint32_t saturateMask = 0u;

	const uint32_t enableMask = 0u;
	const uint32_t channelMask = (static_cast<uint32_t>(channel) << ADC_OFR1_OFFSET1_CH_Pos) & ADC_OFR1_OFFSET1_CH_Msk;
	const uint32_t offsetValue = channelMask | offsetMask | enableMask | saturateMask | signMask;

	if (ADC1->CR & (ADC_CR_ADSTART | ADC_CR_JADSTART))
	{
		// ADC is currently converting, cannot set offset
		return false;
	}

	switch (slot)
	{
		case OffsetSlot::Slot0: ADC1->OFR1 = offsetValue;	break;
		case OffsetSlot::Slot1: ADC1->OFR2 = offsetValue;	break;
		case OffsetSlot::Slot2: ADC1->OFR3 = offsetValue;	break;
		case OffsetSlot::Slot3: ADC1->OFR4 = offsetValue;	break;
		default:
			return false;	// invalid slot
	}

	return true;
}

bool
modm::platform::Adc1::disableChannelOffset(const OffsetSlot slot)
{
	if (ADC1->CR & (ADC_CR_ADSTART | ADC_CR_JADSTART))
	{
		// ADC is currently converting, cannot disable offset
		return false;
	}


	const uint32_t enableMask = ADC_OFR1_OFFSET1_Msk;
	switch (slot)
	{
		case OffsetSlot::Slot0: ADC1->OFR1 &= ~enableMask; break;
		case OffsetSlot::Slot1: ADC1->OFR2 &= ~enableMask; break;
		case OffsetSlot::Slot2: ADC1->OFR3 &= ~enableMask; break;
		case OffsetSlot::Slot3: ADC1->OFR4 &= ~enableMask; break;
		default:
			return false;  // invalid slot
	}
	return true;
}