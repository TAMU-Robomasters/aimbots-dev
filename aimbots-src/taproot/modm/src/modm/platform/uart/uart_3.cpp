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
#include "uart_3.hpp"
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
Usart3::writeBlocking(uint8_t data)
{
	while(!UsartHal3::isTransmitRegisterEmpty());
	UsartHal3::write(data);
}

void
Usart3::writeBlocking(const uint8_t *data, std::size_t length)
{
	while (length-- != 0) {
		writeBlocking(*data++);
	}
}

void
Usart3::flushWriteBuffer()
{
	while(!isWriteFinished());
}

bool
Usart3::write(uint8_t data)
{
	if(txBuffer.isEmpty() && UsartHal3::isTransmitRegisterEmpty()) {
		UsartHal3::write(data);
	} else {
		if (!txBuffer.push(data))
			return false;
		// Disable interrupts while enabling the transmit interrupt
		atomic::Lock lock;
		// Transmit Data Register Empty Interrupt Enable
		UsartHal3::enableInterrupt(Interrupt::TxEmpty);
	}
	return true;
}

std::size_t
Usart3::write(const uint8_t *data, std::size_t length)
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
Usart3::isWriteFinished()
{
	return txBuffer.isEmpty() && UsartHal3::isTransmitRegisterEmpty();
}

std::size_t
Usart3::transmitBufferSize()
{
	return txBuffer.getSize();
}

std::size_t
Usart3::discardTransmitBuffer()
{
	std::size_t count = 0;
	// disable interrupt since buffer will be cleared
	UsartHal3::disableInterrupt(UsartHal3::Interrupt::TxEmpty);
	while(!txBuffer.isEmpty()) {
		++count;
		txBuffer.pop();
	}
	return count;
}

bool
Usart3::read(uint8_t &data)
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
Usart3::read(uint8_t *data, std::size_t length)
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
Usart3::receiveBufferSize()
{
	return rxBuffer.getSize();
}

std::size_t
Usart3::discardReceiveBuffer()
{
	std::size_t count = 0;
	while(!rxBuffer.isEmpty()) {
		++count;
		rxBuffer.pop();
	}
	return count;
}

bool
Usart3::hasError()
{
	return UsartHal3::getInterruptFlags().any(
		UsartHal3::InterruptFlag::ParityError |
#ifdef USART_ISR_NE
		UsartHal3::InterruptFlag::NoiseError |
#endif
		UsartHal3::InterruptFlag::OverrunError | UsartHal3::InterruptFlag::FramingError);
}
void
Usart3::clearError()
{
	return UsartHal3::acknowledgeInterruptFlags(
		UsartHal3::InterruptFlag::ParityError |
#ifdef USART_ISR_NE
		UsartHal3::InterruptFlag::NoiseError |
#endif
		UsartHal3::InterruptFlag::OverrunError | UsartHal3::InterruptFlag::FramingError);
}

}	// namespace modm::platform

MODM_ISR(USART3)
{
	using namespace modm::platform;
	if (UsartHal3::isReceiveRegisterNotEmpty()) {
		// TODO: save the errors
		uint8_t data;
		UsartHal3::read(data);
		rxBuffer.push(data);
	}
	if (UsartHal3::isTransmitRegisterEmpty()) {
		if (txBuffer.isEmpty()) {
			// transmission finished, disable TxEmpty interrupt
			UsartHal3::disableInterrupt(UsartHal3::Interrupt::TxEmpty);
		}
		else {
			UsartHal3::write(txBuffer.get());
			txBuffer.pop();
		}
	}
	UsartHal3::acknowledgeInterruptFlags(UsartHal3::InterruptFlag::OverrunError);
}
