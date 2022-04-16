/*
 * Copyright (c) 2013-2014, 2016, Kevin Läufer
 * Copyright (c) 2013-2017, Niklas Hauser
 * Copyright (c) 2017, Fabian Greif
 * Copyright (c) 2017, Sascha Schade
 * Copyright (c) 2018, Christopher Durand
 * Copyright (c) 2018, Lucas Mösch
 * Copyright (c) 2021, Raphael Lehmann
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
#include <modm/math/algorithm/prescaler.hpp>

namespace modm::platform
{

// ----------------------------------------------------------------------------
void
UsartHal1::enable()
{
	Rcc::enable<Peripheral::Usart1>();
}

void
UsartHal1::disable()
{
	// TX, RX, Uart, etc. Disable
	USART1->CR1 = 0;
	Rcc::disable<Peripheral::Usart1>();
}

void
UsartHal1::enableOperation()
{
	USART1->CR1 |= USART_CR1_UE;
}

void
UsartHal1::disableOperation()
{
	USART1->CR1 &= ~USART_CR1_UE;
}

template< class SystemClock, modm::baudrate_t baudrate, modm::percent_t tolerance >
void
UsartHal1::initialize(Parity parity, WordLength length)
{
	enable();
	disableOperation();

	constexpr uint32_t scalar = (baudrate * 16 > SystemClock::Usart1) ? 8 : 16;
	constexpr uint32_t max = ((scalar == 16) ? (1ul << 16) : (1ul << 15)) - 1ul;
	constexpr auto result = Prescaler::from_range(SystemClock::Usart1, baudrate, 1, max);
	modm::PeripheralDriver::assertBaudrateInTolerance< result.frequency, baudrate, tolerance >();

	uint32_t cr1 = USART1->CR1;
	// set baudrate and oversampling
	if constexpr (scalar == 16) {
		// When OVER8 = 0:, BRR[3:0] = USARTDIV[3:0].
		USART1->BRR = result.prescaler;
		cr1 &= ~USART_CR1_OVER8;
	} else {
		// When OVER8 = 1: BRR[15:4] = USARTDIV[15:4]
		// BRR[2:0] = USARTDIV[3:0] shifted 1 bit to the right. BRR[3] must be kept cleared.
		USART1->BRR = (result.prescaler & ~0b1111) | ((result.prescaler & 0b1111) >> 1);
		cr1 |= USART_CR1_OVER8;
	}
	// Set parity
	cr1 &= ~(USART_CR1_PCE | USART_CR1_PS);
	cr1 |= static_cast<uint32_t>(parity);

	// Set word length
#ifdef USART_CR1_M1
	cr1	&= ~(USART_CR1_M0 | USART_CR1_M1);
#else
	cr1	&= ~USART_CR1_M;
#endif
	cr1 |= static_cast<uint32_t>(length);

	USART1->CR1 = cr1;
}

void
UsartHal1::setSpiClock(SpiClock clk, LastBitClockPulse pulse)
{
	uint32_t cr2 = USART1->CR2;
	cr2 &= ~(USART_CR2_LBCL | USART_CR2_CLKEN);
	cr2 |= static_cast<uint32_t>(clk) | static_cast<uint32_t>(pulse);
	USART1->CR2 = cr2;
}

void
UsartHal1::setSpiDataMode(SpiDataMode mode)
{
	uint32_t cr2 = USART1->CR2;
	cr2 &= ~(USART_CR2_CPOL | USART_CR2_CPHA);
	cr2 |= static_cast<uint32_t>(mode);
	USART1->CR2 = cr2;
}
void
UsartHal1::write(uint16_t data)
{
	USART1->DR = data;
}

void
UsartHal1::read(uint8_t &data)
{
	data = USART1->DR;
}

void
UsartHal1::read(uint16_t &data)
{
	data = USART1->DR;
}

void
UsartHal1::setTransmitterEnable(bool enable)
{
	if (enable) {
		USART1->CR1 |=  USART_CR1_TE;
	} else {
		USART1->CR1 &= ~USART_CR1_TE;
	}
}

void
UsartHal1::setReceiverEnable(bool enable)
{
	if (enable) {
		USART1->CR1 |=  USART_CR1_RE;
	} else {
		USART1->CR1 &= ~USART_CR1_RE;
	}
}

bool
UsartHal1::isReceiveRegisterNotEmpty()
{
	return USART1->SR & USART_SR_RXNE;
}

bool
UsartHal1::isTransmitRegisterEmpty()
{
	return USART1->SR & USART_SR_TXE;
}

void
UsartHal1::enableInterruptVector(bool enable, uint32_t priority)
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
UsartHal1::setInterruptPriority(uint32_t priority)
{
	NVIC_SetPriority(USART1_IRQn, priority);
}

void
UsartHal1::enableInterrupt(Interrupt_t interrupt)
{
	USART1->CR1 |= interrupt.value;
}

void
UsartHal1::disableInterrupt(Interrupt_t interrupt)
{
	USART1->CR1 &= ~interrupt.value;
}

UsartHal1::InterruptFlag_t
UsartHal1::getInterruptFlags()
{
	return InterruptFlag_t( USART1->SR );
}

void
UsartHal1::acknowledgeInterruptFlags(InterruptFlag_t flags)
{
	/* Interrupts must be cleared manually by accessing SR and DR.
	 * Overrun Interrupt, Noise flag detected, Framing Error, Parity Error
	 * p779: "It is cleared by a software sequence (an read to the
	 * USART_SR register followed by a read to the USART_DR register"
	 */
	if (flags.value & 0xful) {
		uint32_t tmp;
		tmp = USART1->SR;
		tmp = USART1->DR;
		(void) tmp;
	}
	(void) flags;	// avoid compiler warning
}

} // namespace modm::platform