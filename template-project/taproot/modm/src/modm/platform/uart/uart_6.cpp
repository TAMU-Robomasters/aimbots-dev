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
#include "uart_6.hpp"
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
Usart6::writeBlocking(uint8_t data)
{
	while(!UsartHal6::isTransmitRegisterEmpty());
	UsartHal6::write(data);
}

void
Usart6::writeBlocking(const uint8_t *data, std::size_t length)
{
	while (length-- != 0) {
		writeBlocking(*data++);
	}
}

void
Usart6::flushWriteBuffer()
{
	while(!isWriteFinished());
}

bool
Usart6::write(uint8_t data)
{
	if(txBuffer.isEmpty() && UsartHal6::isTransmitRegisterEmpty()) {
		UsartHal6::write(data);
	} else {
		if (!txBuffer.push(data))
			return false;
		// Disable interrupts while enabling the transmit interrupt
		atomic::Lock lock;
		// Transmit Data Register Empty Interrupt Enable
		UsartHal6::enableInterrupt(Interrupt::TxEmpty);
	}
	return true;
}

std::size_t
Usart6::write(const uint8_t *data, std::size_t length)
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
Usart6::isWriteFinished()
{
	return txBuffer.isEmpty() && UsartHal6::isTransmitRegisterEmpty();
}

std::size_t
Usart6::transmitBufferSize()
{
	return txBuffer.getSize();
}

std::size_t
Usart6::discardTransmitBuffer()
{
	std::size_t count = 0;
	// disable interrupt since buffer will be cleared
	UsartHal6::disableInterrupt(UsartHal6::Interrupt::TxEmpty);
	while(!txBuffer.isEmpty()) {
		++count;
		txBuffer.pop();
	}
	return count;
}

bool
Usart6::read(uint8_t &data)
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
Usart6::read(uint8_t *data, std::size_t length)
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
Usart6::receiveBufferSize()
{
	return rxBuffer.getSize();
}

std::size_t
Usart6::discardReceiveBuffer()
{
	std::size_t count = 0;
	while(!rxBuffer.isEmpty()) {
		++count;
		rxBuffer.pop();
	}
	return count;
}

bool
Usart6::hasError()
{
	return UsartHal6::getInterruptFlags().any(
		UsartHal6::InterruptFlag::ParityError |
#ifdef USART_ISR_NE
		UsartHal6::InterruptFlag::NoiseError |
#endif
		UsartHal6::InterruptFlag::OverrunError | UsartHal6::InterruptFlag::FramingError);
}
void
Usart6::clearError()
{
	return UsartHal6::acknowledgeInterruptFlags(
		UsartHal6::InterruptFlag::ParityError |
#ifdef USART_ISR_NE
		UsartHal6::InterruptFlag::NoiseError |
#endif
		UsartHal6::InterruptFlag::OverrunError | UsartHal6::InterruptFlag::FramingError);
}

}	// namespace modm::platform

MODM_ISR(USART6)
{
	using namespace modm::platform;
	if (UsartHal6::isReceiveRegisterNotEmpty()) {
		// TODO: save the errors
		uint8_t data;
		UsartHal6::read(data);
		rxBuffer.push(data);
	}
	if (UsartHal6::isTransmitRegisterEmpty()) {
		if (txBuffer.isEmpty()) {
			// transmission finished, disable TxEmpty interrupt
			UsartHal6::disableInterrupt(UsartHal6::Interrupt::TxEmpty);
		}
		else {
			UsartHal6::write(txBuffer.get());
			txBuffer.pop();
		}
	}
	UsartHal6::acknowledgeInterruptFlags(UsartHal6::InterruptFlag::OverrunError);
}
