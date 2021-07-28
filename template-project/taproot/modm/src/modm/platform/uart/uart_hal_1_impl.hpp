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

#ifndef MODM_STM32_UARTHAL_1_HPP
#	error 	"Don't include this file directly, use uart_hal_1.hpp instead!"
#endif
#include <modm/platform/clock/rcc.hpp>

// ----------------------------------------------------------------------------
void
modm::platform::UsartHal1::setParity(const Parity parity)
{
	uint32_t flags = USART1->CR1;
	flags &= ~(USART_CR1_PCE | USART_CR1_PS | USART_CR1_M);
	flags |= static_cast<uint32_t>(parity);
	if (parity != Parity::Disabled) {
		// Parity Bit counts as 9th bit -> enable 9 data bits
		flags |= USART_CR1_M;
	}
	USART1->CR1 = flags;

}

void
modm::platform::UsartHal1::enable()
{
	Rcc::enable<Peripheral::Usart1>();
	USART1->CR1 |= USART_CR1_UE;		// Uart Enable
}

void
modm::platform::UsartHal1::disable()
{
	// TX, RX, Uart, etc. Disable
	USART1->CR1 = 0;
	Rcc::disable<Peripheral::Usart1>();
}
template<class SystemClock, modm::baudrate_t baudrate,
		modm::platform::UsartHal1::OversamplingMode oversample>
void modm_always_inline
modm::platform::UsartHal1::initialize(Parity parity)
{
	initializeWithBrr(UartBaudrate::getBrr<SystemClock::Usart1, baudrate>(), parity, oversample);
}
template<class SystemClock, modm::baudrate_t baudrate>
void modm_always_inline
modm::platform::UsartHal1::initialize(Parity parity)
{
	initializeWithBrr(UartBaudrate::getBrr<SystemClock::Usart1, baudrate>(), parity,
					  UartBaudrate::getOversamplingMode(SystemClock::Usart1, baudrate));
}


void inline
modm::platform::UsartHal1::initializeWithBrr(uint16_t brr, Parity parity, OversamplingMode oversample)
{
	enable();
	// DIRTY HACK: disable and reenable uart to be able to set
	//             baud rate as well as parity
	USART1->CR1 &= ~USART_CR1_UE;	// Uart Disable
	// set baudrate
	USART1->BRR = brr;
	setParity(parity);
	setOversamplingMode(oversample);
	USART1->CR1 |=  USART_CR1_UE;	// Uart Reenable
}

void
modm::platform::UsartHal1::setOversamplingMode(OversamplingMode mode)
{
	if(mode == OversamplingMode::By16) {
		USART1->CR1 &= ~static_cast<uint32_t>(OversamplingMode::By8);
	} else {
		USART1->CR1 |=  static_cast<uint32_t>(OversamplingMode::By8);
	}

}
void
modm::platform::UsartHal1::setSpiClock(SpiClock clk)
{
	if(clk == SpiClock::Disabled) {
		USART1->CR2 &= ~static_cast<uint32_t>(SpiClock::Enabled);
	} else {
		USART1->CR2 |=  static_cast<uint32_t>(SpiClock::Enabled);
	}

}

void
modm::platform::UsartHal1::setSpiDataMode(SpiDataMode mode)
{
	USART1->CR2 =
		(USART1->CR2 & ~static_cast<uint32_t>(SpiDataMode::Mode3))
		| static_cast<uint32_t>(mode);

}

void
modm::platform::UsartHal1::setWordLength(WordLength length)
{
	USART1->CR1 =
#ifdef USART_CR1_M1
		(USART1->CR1 & ~(USART_CR1_M0 | USART_CR1_M1))
#else
		(USART1->CR1 & ~USART_CR1_M)
#endif
		| static_cast<uint32_t>(length);

}

void
modm::platform::UsartHal1::setLastBitClockPulse(LastBitClockPulse pulse)
{
	if(pulse == LastBitClockPulse::DoNotOutput) {
		USART1->CR2 &= ~static_cast<uint32_t>(LastBitClockPulse::Output);
	} else {
		USART1->CR2 |=  static_cast<uint32_t>(LastBitClockPulse::Output);
	}

}
void
modm::platform::UsartHal1::write(uint8_t data)
{
	USART1->DR = data;
}

void
modm::platform::UsartHal1::read(uint8_t &data)
{
	data = USART1->DR;
}

void
modm::platform::UsartHal1::setTransmitterEnable(const bool enable)
{
	if (enable) {
		USART1->CR1 |=  USART_CR1_TE;
	} else {
		USART1->CR1 &= ~USART_CR1_TE;
	}
}

void
modm::platform::UsartHal1::setReceiverEnable(bool enable)
{
	if (enable) {
		USART1->CR1 |=  USART_CR1_RE;
	} else {
		USART1->CR1 &= ~USART_CR1_RE;
	}
}

void
modm::platform::UsartHal1::enableOperation()
{
	USART1->CR1 |= USART_CR1_UE;
}

void
modm::platform::UsartHal1::disableOperation()
{
	USART1->CR1 &= ~USART_CR1_UE;
}

bool
modm::platform::UsartHal1::isReceiveRegisterNotEmpty()
{
	return USART1->SR & USART_SR_RXNE;
}

bool
modm::platform::UsartHal1::isTransmitRegisterEmpty()
{
	return USART1->SR & USART_SR_TXE;
}

void
modm::platform::UsartHal1::enableInterruptVector(bool enable, uint32_t priority)
{
	if (enable) {
		// Set priority for the interrupt vector
		NVIC_SetPriority(USART1_IRQn, priority);

		// register IRQ at the NVIC
		NVIC_EnableIRQ(USART1_IRQn);
	}
	else {
		NVIC_DisableIRQ(USART1_IRQn);
	}
}

void
modm::platform::UsartHal1::enableInterrupt(Interrupt_t interrupt)
{
	USART1->CR1 |= interrupt.value;
}

void
modm::platform::UsartHal1::disableInterrupt(Interrupt_t interrupt)
{
	USART1->CR1 &= ~interrupt.value;
}

modm::platform::UsartHal1::InterruptFlag_t
modm::platform::UsartHal1::getInterruptFlags()
{
	return InterruptFlag_t( USART1->SR );
}

void
modm::platform::UsartHal1::acknowledgeInterruptFlags(InterruptFlag_t flags)
{
	/* Interrupts must be cleared manually by accessing SR and DR.
	 * Overrun Interrupt, Noise flag detected, Framing Error, Parity Error
	 * p779: "It is cleared by a software sequence (an read to the
	 * USART_SR register followed by a read to the USART_DR register"
	 */
	if (flags & InterruptFlag::OverrunError) {
		uint32_t tmp;
		tmp = USART1->SR;
		tmp = USART1->DR;
		(void) tmp;
	}
	(void) flags;	// avoid compiler warning
}