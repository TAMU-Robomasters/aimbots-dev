/*
 * Copyright (c) 2013, Kevin Läufer
 * Copyright (c) 2013-2017, Niklas Hauser
 * Copyright (c) 2014, Daniel Krebs
 * Copyright (c) 2020, Mike Wolfram
 *
 * This file is part of the modm project.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
// ----------------------------------------------------------------------------

#ifndef MODM_STM32_SPI_HAL5_HPP
#	error 	"Don't include this file directly, use 'spi_hal5.hpp' instead!"
#endif
#include <modm/platform/clock/rcc.hpp>

void inline
modm::platform::SpiHal5::enable()
{
	Rcc::enable<Peripheral::Spi5>();
	SPI5->CR1 |= SPI_CR1_SPE;		// SPI Enable
}

void inline
modm::platform::SpiHal5::disable()
{
	SPI5->CR1 = 0;
	Rcc::disable<Peripheral::Spi5>();
}

void inline
modm::platform::SpiHal5::initialize(Prescaler prescaler,
		MasterSelection masterSelection, DataMode dataMode,
		DataOrder dataOrder, DataSize dataSize)
{
	enable();
	// disable peripheral
	SPI5->CR1 &= ~SPI_CR1_SPE;
	// set parameters
	SPI5->CR1 = 	  static_cast<uint32_t>(dataMode)
						| static_cast<uint32_t>(dataOrder)
						| static_cast<uint32_t>(masterSelection)
						| static_cast<uint32_t>(prescaler)
						| static_cast<uint32_t>(dataSize);
	if(masterSelection == MasterSelection::Master) {
		SPI5->CR2 |=  SPI_CR2_SSOE; // for master mode
	}
	// reenable peripheral
	SPI5->CR1 |= SPI_CR1_SPE;
}

void inline
modm::platform::SpiHal5::setDataMode(DataMode dataMode)
{
	SPI5->CR1 = (SPI5->CR1 & ~static_cast<uint32_t>(DataMode::All))
										 | static_cast<uint32_t>(dataMode);
}

void inline
modm::platform::SpiHal5::setDataOrder(DataOrder dataOrder)
{
	SPI5->CR1 = (SPI5->CR1 & ~static_cast<uint32_t>(DataOrder::All))
										 | static_cast<uint32_t>(dataOrder);
}

void inline
modm::platform::SpiHal5::setDataSize(DataSize dataSize)
{
	// TODO: implement as set/reset bit
	SPI5->CR1 = (SPI5->CR1 & ~static_cast<uint32_t>(DataSize::All))
										 | static_cast<uint32_t>(dataSize);
}

void inline
modm::platform::SpiHal5::setMasterSelection(MasterSelection masterSelection)
{
	// TODO: implement as set/reset bit
	SPI5->CR1 = (SPI5->CR1 & ~static_cast<uint32_t>(MasterSelection::All))
										 | static_cast<uint32_t>(masterSelection);
}

inline bool
modm::platform::SpiHal5::isReceiveRegisterNotEmpty()
{
	return static_cast<bool>(getInterruptFlags() & InterruptFlag::RxBufferNotEmpty);
}

inline bool
modm::platform::SpiHal5::isTransmitRegisterEmpty()
{
	return static_cast<bool>(getInterruptFlags() & InterruptFlag::TxBufferEmpty);
}

void inline
modm::platform::SpiHal5::write(uint16_t data)
{
	SPI5->DR = data;
}

void inline
modm::platform::SpiHal5::write(uint8_t data)
{
	write(static_cast<uint16_t>(data));
}

void inline
modm::platform::SpiHal5::read(uint8_t &data)
{
	data = static_cast<uint8_t>(SPI5->DR);
}

void inline
modm::platform::SpiHal5::read(uint16_t &data)
{
	data = static_cast<uint16_t>(SPI5->DR);
}

void inline
modm::platform::SpiHal5::enableInterruptVector(bool enable, uint32_t priority)
{
	if (enable) {
		// Set priority for the interrupt vector
		NVIC_SetPriority(SPI5_IRQn, priority);
		// register IRQ at the NVIC
		NVIC_EnableIRQ(SPI5_IRQn);
	}
	else {
		NVIC_DisableIRQ(SPI5_IRQn);
	}
}

void inline
modm::platform::SpiHal5::enableInterrupt(Interrupt_t interrupt)
{
	SPI5->CR2 |= interrupt.value;
}

void inline
modm::platform::SpiHal5::disableInterrupt(Interrupt_t interrupt)
{
	SPI5->CR2 &= ~interrupt.value;
}

modm::platform::SpiHal5::InterruptFlag_t inline
modm::platform::SpiHal5::getInterruptFlags()
{
	return InterruptFlag_t(SPI5->SR);
}

void inline
modm::platform::SpiHal5::acknowledgeInterruptFlag(InterruptFlag_t /*flags*/)
{
	// TODO: implement; see STM32F3 reference manual p. 736
	// SPI5->SR = flags.value;
}