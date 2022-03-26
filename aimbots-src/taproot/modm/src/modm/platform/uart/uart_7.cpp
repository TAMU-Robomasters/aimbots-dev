/*
 * Copyright (c) 2009, Martin Rosekeit
 * Copyright (c) 2009-2011, Fabian Greif
 * Copyright (c) 2010-2011, 2013, Georgi Grinshpun
 * Copyright (c) 2013-2014, Sascha Schade
 * Copyright (c) 2013, 2016, Kevin Läufer
 * Copyright (c) 2013-2017, Niklas Hauser
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

#include "../device.hpp"
#include "uart_7.hpp"
#include <modm/architecture/interface/atomic_lock.hpp>
#include <modm/architecture/driver/atomic/queue.hpp>

namespace
{
	static modm::atomic::Queue<uint8_t, 256> rxBuffer;
	static modm::atomic::Queue<uint8_t, 256> txBuffer;
}
namespace modm::platform
{

void
Uart7::writeBlocking(uint8_t data)
{
	while(!UartHal7::isTransmitRegisterEmpty());
	UartHal7::write(data);
}

void
Uart7::writeBlocking(const uint8_t *data, std::size_t length)
{
	while (length-- != 0) {
		writeBlocking(*data++);
	}
}

void
Uart7::flushWriteBuffer()
{
	while(!isWriteFinished());
}

bool
Uart7::write(uint8_t data)
{
	if(txBuffer.isEmpty() && UartHal7::isTransmitRegisterEmpty()) {
		UartHal7::write(data);
	} else {
		if (!txBuffer.push(data))
			return false;
		// Disable interrupts while enabling the transmit interrupt
		atomic::Lock lock;
		// Transmit Data Register Empty Interrupt Enable
		UartHal7::enableInterrupt(Interrupt::TxEmpty);
	}
	return true;
}

std::size_t
Uart7::write(const uint8_t *data, std::size_t length)
{
	uint32_t i = 0;
	for (; i < length; ++i)
	{
		if (!write(*data++)) {
			return i;
		}
	}
	return i;
}

bool
Uart7::isWriteFinished()
{
	return txBuffer.isEmpty() && UartHal7::isTransmitRegisterEmpty();
}

std::size_t
Uart7::transmitBufferSize()
{
	return txBuffer.getSize();
}

std::size_t
Uart7::discardTransmitBuffer()
{
	std::size_t count = 0;
	// disable interrupt since buffer will be cleared
	UartHal7::disableInterrupt(UartHal7::Interrupt::TxEmpty);
	while(!txBuffer.isEmpty()) {
		++count;
		txBuffer.pop();
	}
	return count;
}

bool
Uart7::read(uint8_t &data)
{
	if (rxBuffer.isEmpty()) {
		return false;
	} else {
		data = rxBuffer.get();
		rxBuffer.pop();
		return true;
	}
}

std::size_t
Uart7::read(uint8_t *data, std::size_t length)
{
	uint32_t i = 0;
	for (; i < length; ++i)
	{
		if (rxBuffer.isEmpty()) {
			return i;
		} else {
			*data++ = rxBuffer.get();
			rxBuffer.pop();
		}
	}
	return i;
}

std::size_t
Uart7::receiveBufferSize()
{
	return rxBuffer.getSize();
}

std::size_t
Uart7::discardReceiveBuffer()
{
	std::size_t count = 0;
	while(!rxBuffer.isEmpty()) {
		++count;
		rxBuffer.pop();
	}
	return count;
}

bool
Uart7::hasError()
{
	return UartHal7::getInterruptFlags().any(
		UartHal7::InterruptFlag::ParityError |
#ifdef USART_ISR_NE
		UartHal7::InterruptFlag::NoiseError |
#endif
		UartHal7::InterruptFlag::OverrunError | UartHal7::InterruptFlag::FramingError);
}
void
Uart7::clearError()
{
	return UartHal7::acknowledgeInterruptFlags(
		UartHal7::InterruptFlag::ParityError |
#ifdef USART_ISR_NE
		UartHal7::InterruptFlag::NoiseError |
#endif
		UartHal7::InterruptFlag::OverrunError | UartHal7::InterruptFlag::FramingError);
}

}	// namespace modm::platform

MODM_ISR(UART7)
{
	using namespace modm::platform;
	if (UartHal7::isReceiveRegisterNotEmpty()) {
		// TODO: save the errors
		uint8_t data;
		UartHal7::read(data);
		rxBuffer.push(data);
	}
	if (UartHal7::isTransmitRegisterEmpty()) {
		if (txBuffer.isEmpty()) {
			// transmission finished, disable TxEmpty interrupt
			UartHal7::disableInterrupt(UartHal7::Interrupt::TxEmpty);
		}
		else {
			UartHal7::write(txBuffer.get());
			txBuffer.pop();
		}
	}
	UartHal7::acknowledgeInterruptFlags(UartHal7::InterruptFlag::OverrunError);
}
