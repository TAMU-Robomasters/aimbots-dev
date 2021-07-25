/*
 * Copyright (c) 2013-2014, 2016, Kevin Läufer
 * Copyright (c) 2013-2017, Niklas Hauser
 * Copyright (c) 2017, Fabian Greif
 * Copyright (c) 2017, Sascha Schade
 * Copyright (c) 2018, Christopher Durand
 * Copyright (c) 2018, Lucas Mösch
 *
 * This file is part of the modm project.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
// ----------------------------------------------------------------------------

#ifndef MODM_STM32_UARTHAL_2_HPP
#	error 	"Don't include this file directly, use uart_hal_2.hpp instead!"
#endif
#include <modm/platform/clock/rcc.hpp>

// ----------------------------------------------------------------------------
void
modm::platform::UsartHal2::setParity(const Parity parity)
{
	uint32_t flags = USART2->CR1;
	flags &= ~(USART_CR1_PCE | USART_CR1_PS | USART_CR1_M);
	flags |= static_cast<uint32_t>(parity);
	if (parity != Parity::Disabled) {
		// Parity Bit counts as 9th bit -> enable 9 data bits
		flags |= USART_CR1_M;
	}
	USART2->CR1 = flags;

}

void
modm::platform::UsartHal2::enable()
{
	Rcc::enable<Peripheral::Usart2>();
	USART2->CR1 |= USART_CR1_UE;		// Uart Enable
}

void
modm::platform::UsartHal2::disable()
{
	// TX, RX, Uart, etc. Disable
	USART2->CR1 = 0;
	Rcc::disable<Peripheral::Usart2>();
}
template<class SystemClock, modm::baudrate_t baudrate,
		modm::platform::UsartHal2::OversamplingMode oversample>
void modm_always_inline
modm::platform::UsartHal2::initialize(Parity parity)
{
	initializeWithBrr(UartBaudrate::getBrr<SystemClock::Usart2, baudrate>(), parity, oversample);
}
template<class SystemClock, modm::baudrate_t baudrate>
void modm_always_inline
modm::platform::UsartHal2::initialize(Parity parity)
{
	initializeWithBrr(UartBaudrate::getBrr<SystemClock::Usart2, baudrate>(), parity,
					  UartBaudrate::getOversamplingMode(SystemClock::Usart2, baudrate));
}


void inline
modm::platform::UsartHal2::initializeWithBrr(uint16_t brr, Parity parity, OversamplingMode oversample)
{
	enable();
	// DIRTY HACK: disable and reenable uart to be able to set
	//             baud rate as well as parity
	USART2->CR1 &= ~USART_CR1_UE;	// Uart Disable
	// set baudrate
	USART2->BRR = brr;
	setParity(parity);
	setOversamplingMode(oversample);
	USART2->CR1 |=  USART_CR1_UE;	// Uart Reenable
}

void
modm::platform::UsartHal2::setOversamplingMode(OversamplingMode mode)
{
	if(mode == OversamplingMode::By16) {
		USART2->CR1 &= ~static_cast<uint32_t>(OversamplingMode::By8);
	} else {
		USART2->CR1 |=  static_cast<uint32_t>(OversamplingMode::By8);
	}

}
void
modm::platform::UsartHal2::setSpiClock(SpiClock clk)
{
	if(clk == SpiClock::Disabled) {
		USART2->CR2 &= ~static_cast<uint32_t>(SpiClock::Enabled);
	} else {
		USART2->CR2 |=  static_cast<uint32_t>(SpiClock::Enabled);
	}

}

void
modm::platform::UsartHal2::setSpiDataMode(SpiDataMode mode)
{
	USART2->CR2 =
		(USART2->CR2 & ~static_cast<uint32_t>(SpiDataMode::Mode3))
		| static_cast<uint32_t>(mode);

}

void
modm::platform::UsartHal2::setWordLength(WordLength length)
{
	USART2->CR1 =
#ifdef USART_CR1_M1
		(USART2->CR1 & ~(USART_CR1_M0 | USART_CR1_M1))
#else
		(USART2->CR1 & ~USART_CR1_M)
#endif
		| static_cast<uint32_t>(length);

}

void
modm::platform::UsartHal2::setLastBitClockPulse(LastBitClockPulse pulse)
{
	if(pulse == LastBitClockPulse::DoNotOutput) {
		USART2->CR2 &= ~static_cast<uint32_t>(LastBitClockPulse::Output);
	} else {
		USART2->CR2 |=  static_cast<uint32_t>(LastBitClockPulse::Output);
	}

}
void
modm::platform::UsartHal2::write(uint8_t data)
{
	USART2->DR = data;
}

void
modm::platform::UsartHal2::read(uint8_t &data)
{
	data = USART2->DR;
}

void
modm::platform::UsartHal2::setTransmitterEnable(const bool enable)
{
	if (enable) {
		USART2->CR1 |=  USART_CR1_TE;
	} else {
		USART2->CR1 &= ~USART_CR1_TE;
	}
}

void
modm::platform::UsartHal2::setReceiverEnable(bool enable)
{
	if (enable) {
		USART2->CR1 |=  USART_CR1_RE;
	} else {
		USART2->CR1 &= ~USART_CR1_RE;
	}
}

void
modm::platform::UsartHal2::enableOperation()
{
	USART2->CR1 |= USART_CR1_UE;
}

void
modm::platform::UsartHal2::disableOperation()
{
	USART2->CR1 &= ~USART_CR1_UE;
}

bool
modm::platform::UsartHal2::isReceiveRegisterNotEmpty()
{
	return USART2->SR & USART_SR_RXNE;
}

bool
modm::platform::UsartHal2::isTransmitRegisterEmpty()
{
	return USART2->SR & USART_SR_TXE;
}

void
modm::platform::UsartHal2::enableInterruptVector(bool enable, uint32_t priority)
{
	if (enable) {
		// Set priority for the interrupt vector
		NVIC_SetPriority(USART2_IRQn, priority);

		// register IRQ at the NVIC
		NVIC_EnableIRQ(USART2_IRQn);
	}
	else {
		NVIC_DisableIRQ(USART2_IRQn);
	}
}

void
modm::platform::UsartHal2::enableInterrupt(Interrupt_t interrupt)
{
	USART2->CR1 |= interrupt.value;
}

void
modm::platform::UsartHal2::disableInterrupt(Interrupt_t interrupt)
{
	USART2->CR1 &= ~interrupt.value;
}

modm::platform::UsartHal2::InterruptFlag_t
modm::platform::UsartHal2::getInterruptFlags()
{
	return InterruptFlag_t( USART2->SR );
}

void
modm::platform::UsartHal2::acknowledgeInterruptFlags(InterruptFlag_t flags)
{
	/* Interrupts must be cleared manually by accessing SR and DR.
	 * Overrun Interrupt, Noise flag detected, Framing Error, Parity Error
	 * p779: "It is cleared by a software sequence (an read to the
	 * USART_SR register followed by a read to the USART_DR register"
	 */
	if (flags & InterruptFlag::OverrunError) {
		uint32_t tmp;
		tmp = USART2->SR;
		tmp = USART2->DR;
		(void) tmp;
	}
	(void) flags;	// avoid compiler warning
}