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

#ifndef MODM_STM32_UARTHAL_6_HPP
#	error 	"Don't include this file directly, use uart_hal_6.hpp instead!"
#endif
#include <modm/platform/clock/rcc.hpp>
#include <modm/math/algorithm/prescaler.hpp>

namespace modm::platform
{

// ----------------------------------------------------------------------------
void
UsartHal6::enable()
{
	Rcc::enable<Peripheral::Usart6>();
}

void
UsartHal6::disable()
{
	// TX, RX, Uart, etc. Disable
	USART6->CR1 = 0;
	Rcc::disable<Peripheral::Usart6>();
}

void
UsartHal6::enableOperation()
{
	USART6->CR1 |= USART_CR1_UE;
}

void
UsartHal6::disableOperation()
{
	USART6->CR1 &= ~USART_CR1_UE;
}

template< class SystemClock, modm::baudrate_t baudrate, modm::percent_t tolerance >
void
UsartHal6::initialize(Parity parity, WordLength length)
{
	enable();
	disableOperation();

	constexpr uint32_t max = (1ul << 16) - 1ul;
	constexpr auto result = Prescaler::from_linear(SystemClock::Usart6, baudrate, 16, max);
	modm::PeripheralDriver::assertBaudrateInTolerance< result.frequency, baudrate, tolerance >();

	uint32_t cr1 = USART6->CR1;
	// Set baudrate
	USART6->BRR = result.prescaler;
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

	USART6->CR1 = cr1;
}

void
UsartHal6::setSpiClock(SpiClock clk, LastBitClockPulse pulse)
{
	uint32_t cr2 = USART6->CR2;
	cr2 &= ~(USART_CR2_LBCL | USART_CR2_CLKEN);
	cr2 |= static_cast<uint32_t>(clk) | static_cast<uint32_t>(pulse);
	USART6->CR2 = cr2;
}

void
UsartHal6::setSpiDataMode(SpiDataMode mode)
{
	uint32_t cr2 = USART6->CR2;
	cr2 &= ~(USART_CR2_CPOL | USART_CR2_CPHA);
	cr2 |= static_cast<uint32_t>(mode);
	USART6->CR2 = cr2;
}
void
UsartHal6::write(uint16_t data)
{
	USART6->TDR = data;
}

void
UsartHal6::read(uint8_t &data)
{
	data = USART6->RDR;
}

void
UsartHal6::read(uint16_t &data)
{
	data = USART6->RDR;
}

void
UsartHal6::setTransmitterEnable(bool enable)
{
	if (enable) {
		USART6->CR1 |=  USART_CR1_TE;
	} else {
		USART6->CR1 &= ~USART_CR1_TE;
	}
}

void
UsartHal6::setReceiverEnable(bool enable)
{
	if (enable) {
		USART6->CR1 |=  USART_CR1_RE;
	} else {
		USART6->CR1 &= ~USART_CR1_RE;
	}
}

bool
UsartHal6::isReceiveRegisterNotEmpty()
{
	return USART6->ISR & USART_ISR_RXNE;
}

bool
UsartHal6::isTransmitRegisterEmpty()
{
	return USART6->ISR & USART_ISR_TXE;
}

bool
UsartHal6::isTransmissionComplete()
{
	return USART6->ISR & USART_ISR_TC;
}

void
UsartHal6::enableInterruptVector(bool enable, uint32_t priority)
{
	if (enable) {
		// Set priority for the interrupt vector
		NVIC_SetPriority(USART6_IRQn, priority);

		// register IRQ at the NVIC
		NVIC_EnableIRQ(USART6_IRQn);
	}
	else {
		NVIC_DisableIRQ(USART6_IRQn);
	}
}

void
UsartHal6::setInterruptPriority(uint32_t priority)
{
	NVIC_SetPriority(USART6_IRQn, priority);
}

void
UsartHal6::enableInterrupt(Interrupt_t interrupt)
{
	USART6->CR1 |= interrupt.value;
}

void
UsartHal6::disableInterrupt(Interrupt_t interrupt)
{
	USART6->CR1 &= ~interrupt.value;
}

UsartHal6::InterruptFlag_t
UsartHal6::getInterruptFlags()
{
	return InterruptFlag_t( USART6->ISR );
}

void
UsartHal6::acknowledgeInterruptFlags(InterruptFlag_t flags)
{
	// Flags are cleared by writing a one to the flag position.
	// Writing a zero is (hopefully) ignored.
	USART6->ICR = flags.value;
}

} // namespace modm::platform