/*
 * Copyright (c) 2013, Sascha Schade
 * Copyright (c) 2013-2014, 2016, Kevin Läufer
 * Copyright (c) 2013-2017, 2021, Niklas Hauser
 * Copyright (c) 2021, Raphael Lehmann
 *
 * This file is part of the modm project.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
// ----------------------------------------------------------------------------

#ifndef MODM_STM32_UARTHAL_10_HPP
#define MODM_STM32_UARTHAL_10_HPP
#include <stdint.h>
#include "../device.hpp"
#include "uart_base.hpp"
#include <modm/architecture/interface/peripheral.hpp>
#include <modm/utils/inplace_function.hpp>


namespace modm::platform
{

/**
 * Universal asynchronous receiver transmitter (UsartHal10)
 *
 * Not available on the low- and medium density devices.
 *
 * Very basic implementation that exposes more hardware features than
 * the regular Usart classes.
 *
 * @author		Kevin Laeufer
 * @ingroup		modm_platform_uart
 */
class UsartHal10 : public UartBase
{
public:
	static constexpr bool isExtended = true;
	static const Peripheral UartPeripheral = Peripheral::Usart10;
	static inline modm::inplace_function<bool(bool)> InterruptCallback;

	/// Enables the clock, resets the hardware
	/// @warning Call `enableOperation()` to start the peripheral!
	static inline void
	enable();

	/// Disables the hw module (by disabling its clock line)
	static inline void
	disable();

	/// Set the UE (USART enable) bit
	static inline void
	enableOperation();

	/// Clear the UE (USART enable) bit
	static inline void
	disableOperation();

	/// @warning Call `enableOperation()` after this to start the peripheral!
	template< class SystemClock, baudrate_t baudrate, percent_t tolerance >
	static inline void
	initialize(Parity parity, WordLength length);

	/// @warning You must call `disableOperations()` before this function!
	static inline void
	setSpiClock(SpiClock clk, LastBitClockPulse pulse);

	/// @warning You must call `disableOperations()` before this function!
	static inline void
	setSpiDataMode(SpiDataMode mode);
	/**
	 * \brief	Write a single byte to the transmit register
	 *
	 * @warning 	This method does NOT do any sanity checks!!
	 *				It is your responsibility to check if the register
	 *				is empty!
	 */
	static inline void
	write(uint16_t data);

	/**
	 * Saves the value of the receive register to data
	 *
	 * @warning 	This method does NOT do any sanity checks!!
	 *				It is your responsibility to check if the register
	 *				contains something useful!
	 */
	static inline void
	read(uint8_t &data);

	static inline void
	read(uint16_t &data);

	static inline void
	setTransmitterEnable(bool enable);

	static inline void
	setReceiverEnable(bool enable);

	/// Returns true if data has been received
	static inline bool
	isReceiveRegisterNotEmpty();

	/// Returns true if data can be written
	static inline bool
	isTransmitRegisterEmpty();

	/// Returns true if the transmission of a frame containing data is complete
	static inline bool
	isTransmissionComplete();

	static inline void
	enableInterruptVector(bool enable, uint32_t priority);

	static inline void
	enableInterrupt(Interrupt_t interrupt);

	static inline void
	disableInterrupt(Interrupt_t interrupt);

	static inline void
	setInterruptPriority(uint32_t priority);

	static inline InterruptFlag_t
	getInterruptFlags();

	static inline void
	acknowledgeInterruptFlags(InterruptFlag_t flags);
};

}	// namespace modm::platform

#include "uart_hal_10_impl.hpp"
#endif