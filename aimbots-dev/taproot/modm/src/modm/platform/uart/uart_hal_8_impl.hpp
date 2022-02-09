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

#ifndef MODM_STM32_UARTHAL_8_HPP
#	error 	"Don't include this file directly, use uart_hal_8.hpp instead!"
#endif
#include <modm/platform/clock/rcc.hpp>
#include <modm/math/algorithm/prescaler.hpp>

namespace modm::platform
{

// ----------------------------------------------------------------------------
void
UartHal8::enable()
{
	Rcc::enable<Peripheral::Uart8>();
}

void
UartHal8::disable()
{
	// TX, RX, Uart, etc. Disable
	UART8->CR1 = 0;
	Rcc::disable<Peripheral::Uart8>();
}

void
UartHal8::enableOperation()
{
	UART8->CR1 |= USART_CR1_UE;
}

void
UartHal8::disableOperation()
{
	UART8->CR1 &= ~USART_CR1_UE;
}

template< class SystemClock, modm::baudrate_t baudrate, modm::percent_t tolerance >
void
UartHal8::initialize(Parity parity, WordLength length)
{
	enable();
	disableOperation();

	constexpr uint32_t scalar = (baudrate * 16 > SystemClock::Uart8) ? 8 : 16;
	constexpr uint32_t max = ((scalar == 16) ? (1ul << 16) : (1ul << 15)) - 1ul;
	constexpr auto result = Prescaler::from_range(SystemClock::Uart8, baudrate, 1, max);
	modm::PeripheralDriver::assertBaudrateInTolerance< result.frequency, baudrate, tolerance >();

	uint32_t cr1 = UART8->CR1;
	// set baudrate and oversampling
	if constexpr (scalar == 16) {
		// When OVER8 = 0:, BRR[3:0] = USARTDIV[3:0].
		UART8->BRR = result.prescaler;
		cr1 &= ~USART_CR1_OVER8;
	} else {
		// When OVER8 = 1: BRR[15:4] = USARTDIV[15:4]
		// BRR[2:0] = USARTDIV[3:0] shifted 1 bit to the right. BRR[3] must be kept cleared.
		UART8->BRR = (result.prescaler & ~0b1111) | ((result.prescaler & 0b1111) >> 1);
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

	UART8->CR1 = cr1;
}

void
UartHal8::write(uint16_t data)
{
	UART8->DR = data;
}

void
UartHal8::read(uint8_t &data)
{
	data = UART8->DR;
}

void
UartHal8::read(uint16_t &data)
{
	data = UART8->DR;
}

void
UartHal8::setTransmitterEnable(bool enable)
{
	if (enable) {
		UART8->CR1 |=  USART_CR1_TE;
	} else {
		UART8->CR1 &= ~USART_CR1_TE;
	}
}

void
UartHal8::setReceiverEnable(bool enable)
{
	if (enable) {
		UART8->CR1 |=  USART_CR1_RE;
	} else {
		UART8->CR1 &= ~USART_CR1_RE;
	}
}

bool
UartHal8::isReceiveRegisterNotEmpty()
{
	return UART8->SR & USART_SR_RXNE;
}

bool
UartHal8::isTransmitRegisterEmpty()
{
	return UART8->SR & USART_SR_TXE;
}

void
UartHal8::enableInterruptVector(bool enable, uint32_t priority)
{
	if (enable) {
		// Set priority for the interrupt vector
		NVIC_SetPriority(UART8_IRQn, priority);

		// register IRQ at the NVIC
		NVIC_EnableIRQ(UART8_IRQn);
	}
	else {
		NVIC_DisableIRQ(UART8_IRQn);
	}
}

void
UartHal8::setInterruptPriority(uint32_t priority)
{
	NVIC_SetPriority(UART8_IRQn, priority);
}

void
UartHal8::enableInterrupt(Interrupt_t interrupt)
{
	UART8->CR1 |= interrupt.value;
}

void
UartHal8::disableInterrupt(Interrupt_t interrupt)
{
	UART8->CR1 &= ~interrupt.value;
}

UartHal8::InterruptFlag_t
UartHal8::getInterruptFlags()
{
	return InterruptFlag_t( UART8->SR );
}

void
UartHal8::acknowledgeInterruptFlags(InterruptFlag_t flags)
{
	/* Interrupts must be cleared manually by accessing SR and DR.
	 * Overrun Interrupt, Noise flag detected, Framing Error, Parity Error
	 * p779: "It is cleared by a software sequence (an read to the
	 * USART_SR register followed by a read to the USART_DR register"
	 */
	if (flags.value & 0xful) {
		uint32_t tmp;
		tmp = UART8->SR;
		tmp = UART8->DR;
		(void) tmp;
	}
	(void) flags;	// avoid compiler warning
}

} // namespace modm::platform