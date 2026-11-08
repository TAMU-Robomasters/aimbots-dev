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

#ifndef MODM_STM32_UARTHAL_5_HPP
#	error 	"Don't include this file directly, use uart_hal_5.hpp instead!"
#endif
#include <modm/platform/clock/rcc.hpp>
#include <modm/math/algorithm/prescaler.hpp>

namespace modm::platform
{

// ----------------------------------------------------------------------------
void
UartHal5::enable()
{
	Rcc::enable<Peripheral::Uart5>();
}

void
UartHal5::disable()
{
	// TX, RX, Uart, etc. Disable
	UART5->CR1 = 0;
	Rcc::disable<Peripheral::Uart5>();
}

void
UartHal5::enableOperation()
{
	UART5->CR1 |= USART_CR1_UE;
}

void
UartHal5::disableOperation()
{
	UART5->CR1 &= ~USART_CR1_UE;
}

template< class SystemClock, modm::baudrate_t baudrate, modm::percent_t tolerance >
void
UartHal5::initialize(Parity parity, WordLength length)
{
	enable();
	disableOperation();

	constexpr uint32_t max = (1ul << 16) - 1ul;
	constexpr auto result = Prescaler::from_linear(SystemClock::Uart5, baudrate, 16, max);
	modm::PeripheralDriver::assertBaudrateInTolerance< result.frequency, baudrate, tolerance >();

	uint32_t cr1 = UART5->CR1;
	// Set baudrate
	UART5->BRR = result.prescaler;
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

	UART5->CR1 = cr1;
}

void
UartHal5::write(uint16_t data)
{
	UART5->TDR = data;
}

void
UartHal5::read(uint8_t &data)
{
	data = UART5->RDR;
}

void
UartHal5::read(uint16_t &data)
{
	data = UART5->RDR;
}

void
UartHal5::setTransmitterEnable(bool enable)
{
	if (enable) {
		UART5->CR1 |=  USART_CR1_TE;
	} else {
		UART5->CR1 &= ~USART_CR1_TE;
	}
}

void
UartHal5::setReceiverEnable(bool enable)
{
	if (enable) {
		UART5->CR1 |=  USART_CR1_RE;
	} else {
		UART5->CR1 &= ~USART_CR1_RE;
	}
}

bool
UartHal5::isReceiveRegisterNotEmpty()
{
	return UART5->ISR & USART_ISR_RXNE;
}

bool
UartHal5::isTransmitRegisterEmpty()
{
	return UART5->ISR & USART_ISR_TXE;
}

bool
UartHal5::isTransmissionComplete()
{
	return UART5->ISR & USART_ISR_TC;
}

void
UartHal5::enableInterruptVector(bool enable, uint32_t priority)
{
	if (enable) {
		// Set priority for the interrupt vector
		NVIC_SetPriority(UART5_IRQn, priority);

		// register IRQ at the NVIC
		NVIC_EnableIRQ(UART5_IRQn);
	}
	else {
		NVIC_DisableIRQ(UART5_IRQn);
	}
}

void
UartHal5::setInterruptPriority(uint32_t priority)
{
	NVIC_SetPriority(UART5_IRQn, priority);
}

void
UartHal5::enableInterrupt(Interrupt_t interrupt)
{
	UART5->CR1 |= interrupt.value;
}

void
UartHal5::disableInterrupt(Interrupt_t interrupt)
{
	UART5->CR1 &= ~interrupt.value;
}

UartHal5::InterruptFlag_t
UartHal5::getInterruptFlags()
{
	return InterruptFlag_t( UART5->ISR );
}

void
UartHal5::acknowledgeInterruptFlags(InterruptFlag_t flags)
{
	// Flags are cleared by writing a one to the flag position.
	// Writing a zero is (hopefully) ignored.
	UART5->ICR = flags.value;
}

} // namespace modm::platform