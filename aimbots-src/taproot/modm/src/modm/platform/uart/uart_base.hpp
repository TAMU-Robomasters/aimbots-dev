/*
 * Copyright (c) 2013, Kevin Läufer
 * Copyright (c) 2014-2018, Niklas Hauser
 * Copyright (c) 2017, Sascha Schade
 *
 * This file is part of the modm project.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
// ----------------------------------------------------------------------------

#ifndef MODM_STM32_UART_BASE_HPP
#define MODM_STM32_UART_BASE_HPP

#include <stdint.h>
#include "../device.hpp"
#include <modm/architecture/interface/register.hpp>
#include <modm/architecture/interface/interrupt.hpp>

/// @cond
// STM has some weird ideas about continuity
#ifndef USART_BRR_DIV_MANTISSA
#define	USART_BRR_DIV_MANTISSA	USART_BRR_DIV_Mantissa
#endif
#ifndef USART_BRR_DIV_FRACTION
#define	USART_BRR_DIV_FRACTION	USART_BRR_DIV_Fraction
#endif
/// @endcond


namespace modm::platform
{

/**
 * Base class for the UART classes
 *
 * Provides some common enum that do not depend on the specific UART.
 *
 * @author Kevin Laeufer
 * @ingroup		modm_platform_uart
 */
class UartBase
{

public:
	enum class
	Interrupt : uint32_t
	{
		/// Call interrupt when a parity error occurred.
		ParityError	= USART_CR1_PEIE,
		/// Call interrupt when transmit register is empty (i.e. the byte has been transfered to the shift out register
		TxEmpty		= USART_CR1_TXEIE,
		/// Called when the byte was completely transmitted
		TxComplete	= USART_CR1_TCIE,
		/// Call interrupt when char received (RXNE) or overrun occurred (ORE)
		RxNotEmpty	= USART_CR1_RXNEIE,
	};
	MODM_FLAGS32(Interrupt);

	enum class
	InterruptFlag : uint32_t
	{
		/// Set if the transmit data register is empty.
		TxEmpty			= USART_SR_TXE,
		/// Set if the transmission is complete.
		TxComplete		= USART_SR_TC,
		/// Set if the receive data register is not empty.
		RxNotEmpty		= USART_SR_RXNE,
		/// Set if receive register was not cleared.
		OverrunError	= USART_SR_ORE,
		/// Set if a de-synchronization, excessive noise or a break character is detected
		FramingError 	= USART_SR_FE,
		/// Set if a parity error was detected.
		ParityError		= USART_SR_PE,
	};
	MODM_FLAGS32(InterruptFlag);

	enum class
	Parity : uint32_t
	{
		Disabled 	= 0,
		Even 		= USART_CR1_PCE,
		Odd  		= USART_CR1_PCE | USART_CR1_PS,
	};

	enum class
	WordLength : uint32_t
	{
#ifdef USART_CR1_M1
		Bit7 = USART_CR1_M1,
		Bit8 = 0,
		Bit9 = USART_CR1_M0,
#else
		Bit8 = 0,
		Bit9 = USART_CR1_M,
#endif
	};

	enum class
	SpiClock : uint32_t
	{
		Disabled = 0b0,
		Enabled  = USART_CR2_CLKEN,
	};

	enum class
	LastBitClockPulse : uint32_t
	{
		DoNotOutput = 0b0,
		Output = USART_CR2_LBCL,
	};

	enum class
	SpiDataMode : uint32_t
	{
		Mode0 = 0b00,
		Mode1 = USART_CR2_CPHA,
		Mode2 = USART_CR2_CPOL,
		Mode3 = USART_CR2_CPOL | USART_CR2_CPHA,
	};
};

}	// namespace modm::platform

#endif // MODM_STM32_UART_BASE_HPP