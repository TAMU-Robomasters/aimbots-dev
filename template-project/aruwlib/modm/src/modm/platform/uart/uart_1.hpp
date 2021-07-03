/*
 * Copyright (c) 2009-2012, Fabian Greif
 * Copyright (c) 2010, Martin Rosekeit
 * Copyright (c) 2011, Georgi Grinshpun
 * Copyright (c) 2011, 2013-2017, Niklas Hauser
 * Copyright (c) 2012, Sascha Schade
 * Copyright (c) 2013, 2016, Kevin Läufer
 *
 * This file is part of the modm project.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
// ----------------------------------------------------------------------------

#ifndef MODM_STM32_UART_1_HPP
#define MODM_STM32_UART_1_HPP

#include <modm/architecture/interface/uart.hpp>
#include <modm/platform/gpio/connector.hpp>
#include "uart_base.hpp"
#include "uart_baudrate.hpp"
#include "uart_hal_1.hpp"

namespace modm
{

namespace platform
{

/**
 * Universal asynchronous receiver transmitter (USART1)
 *
 * @author		Kevin Laeufer
 * @author		Niklas Hauser
 * @ingroup		modm_platform_uart modm_platform_uart_1
 */
class Usart1 : public UartBase, public ::modm::Uart
{
private:
	/// Second stage initialize for buffered uart
	// that need to be implemented in the .cpp
	static void
	initializeBuffered(uint32_t interruptPriority);
public:
	using Hal = UsartHal1;
	// Expose jinja template parameters to be checked by e.g. drivers or application
	static constexpr size_t RxBufferSize = 256;
	static constexpr size_t TxBufferSize = 256;

public:
	template< template<Peripheral _> class... Signals >
	static void
	connect(Gpio::InputType InputTypeRx = Gpio::InputType::PullUp,
	        Gpio::OutputType OutputTypeTx = Gpio::OutputType::PushPull)
	{
		using Connector = GpioConnector<Peripheral::Usart1, Signals...>;
		using Tx = typename Connector::template GetSignal< Gpio::Signal::Tx >;
		using Rx = typename Connector::template GetSignal< Gpio::Signal::Rx >;
		static_assert(((Connector::template IsValid<Tx> and Connector::template IsValid<Rx>) and sizeof...(Signals) == 2) or
					  ((Connector::template IsValid<Tx> or  Connector::template IsValid<Rx>) and sizeof...(Signals) == 1),
					  "Usart1::connect() requires one Tx and/or one Rx signal!");

		// Connector::disconnect();
		Tx::setOutput(OutputTypeTx);
		Rx::setInput(InputTypeRx);
		Connector::connect();
	}

	template< class SystemClock, baudrate_t baudrate, percent_t tolerance=pct(1) >
	static void modm_always_inline
	initialize(uint32_t interruptPriority = 12, Parity parity = Parity::Disabled)
	{
		UsartHal1::initializeWithBrr(
				UartBaudrate::getBrr<SystemClock::Usart1, baudrate, tolerance>(),
				parity,
				UartBaudrate::getOversamplingMode(SystemClock::Usart1, baudrate));
		initializeBuffered(interruptPriority);
		UsartHal1::setTransmitterEnable(true);
		UsartHal1::setReceiverEnable(true);
	}

	static void
	writeBlocking(uint8_t data);

	static void
	writeBlocking(const uint8_t *data, std::size_t length);

	static void
	flushWriteBuffer();

	static bool
	write(uint8_t data);

	static std::size_t
	write(const uint8_t *data, std::size_t length);

	static bool
	isWriteFinished();

	static std::size_t
	transmitBufferSize();

	static std::size_t
	discardTransmitBuffer();

	static bool
	read(uint8_t &data);

	static std::size_t
	read(uint8_t *buffer, std::size_t length);

	static std::size_t
	receiveBufferSize();

	static std::size_t
	discardReceiveBuffer();

	static bool
	hasError();

	static void
	clearError();

};

}	// namespace platform

}	// namespace modm

#endif // MODM_STM32_UART_1_HPP