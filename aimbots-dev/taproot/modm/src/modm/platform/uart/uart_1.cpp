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
#include "uart_1.hpp"
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
Usart1::writeBlocking(uint8_t data)
{
	while(!UsartHal1::isTransmitRegisterEmpty());
	UsartHal1::write(data);
}

void
Usart1::writeBlocking(const uint8_t *data, std::size_t length)
{
	while (length-- != 0) {
		writeBlocking(*data++);
	}
}

void
Usart1::flushWriteBuffer()
{
	while(!isWriteFinished());
}

bool
Usart1::write(uint8_t data)
{
	if(txBuffer.isEmpty() && UsartHal1::isTransmitRegisterEmpty()) {
		UsartHal1::write(data);
	} else {
		if (!txBuffer.push(data))
			return false;
		// Disable interrupts while enabling the transmit interrupt
		atomic::Lock lock;
		// Transmit Data Register Empty Interrupt Enable
		UsartHal1::enableInterrupt(Interrupt::TxEmpty);
	}
	return true;
}

std::size_t
Usart1::write(const uint8_t *data, std::size_t length)
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
Usart1::isWriteFinished()
{
	return txBuffer.isEmpty() && UsartHal1::isTransmitRegisterEmpty();
}

std::size_t
Usart1::transmitBufferSize()
{
	return txBuffer.getSize();
}

std::size_t
Usart1::discardTransmitBuffer()
{
	std::size_t count = 0;
	// disable interrupt since buffer will be cleared
	UsartHal1::disableInterrupt(UsartHal1::Interrupt::TxEmpty);
	while(!txBuffer.isEmpty()) {
		++count;
		txBuffer.pop();
	}
	return count;
}

bool
Usart1::read(uint8_t &data)
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
Usart1::read(uint8_t *data, std::size_t length)
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
Usart1::receiveBufferSize()
{
	return rxBuffer.getSize();
}

std::size_t
Usart1::discardReceiveBuffer()
{
	std::size_t count = 0;
	while(!rxBuffer.isEmpty()) {
		++count;
		rxBuffer.pop();
	}
	return count;
}

bool
Usart1::hasError()
{
	return UsartHal1::getInterruptFlags().any(
		UsartHal1::InterruptFlag::ParityError |
#ifdef USART_ISR_NE
		UsartHal1::InterruptFlag::NoiseError |
#endif
		UsartHal1::InterruptFlag::OverrunError | UsartHal1::InterruptFlag::FramingError);
}
void
Usart1::clearError()
{
	return UsartHal1::acknowledgeInterruptFlags(
		UsartHal1::InterruptFlag::ParityError |
#ifdef USART_ISR_NE
		UsartHal1::InterruptFlag::NoiseError |
#endif
		UsartHal1::InterruptFlag::OverrunError | UsartHal1::InterruptFlag::FramingError);
}

}	// namespace modm::platform

MODM_ISR(USART1)
{
	using namespace modm::platform;
	if (UsartHal1::isReceiveRegisterNotEmpty()) {
		// TODO: save the errors
		uint8_t data;
		UsartHal1::read(data);
		rxBuffer.push(data);
	}
	if (UsartHal1::isTransmitRegisterEmpty()) {
		if (txBuffer.isEmpty()) {
			// transmission finished, disable TxEmpty interrupt
			UsartHal1::disableInterrupt(UsartHal1::Interrupt::TxEmpty);
		}
		else {
			UsartHal1::write(txBuffer.get());
			txBuffer.pop();
		}
	}
	UsartHal1::acknowledgeInterruptFlags(UsartHal1::InterruptFlag::OverrunError);
}
