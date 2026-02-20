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

#ifndef MODM_STM32_SPI_HAL2_HPP
#	error 	"Don't include this file directly, use 'spi_hal2.hpp' instead!"
#endif
#include <modm/platform/clock/rcc.hpp>

void inline
modm::platform::SpiHal2::enable()
{
	Rcc::enable<Peripheral::Spi2>();
	SPI2->CR1 |= SPI_CR1_SPE;		// SPI Enable
}

void inline
modm::platform::SpiHal2::disable()
{
	SPI2->CR1 = 0;
	Rcc::disable<Peripheral::Spi2>();
}

void inline
modm::platform::SpiHal2::initialize(Prescaler prescaler,
		MasterSelection masterSelection, DataMode dataMode,
		DataOrder dataOrder, DataSize dataSize)
{
	enable();
	// disable peripheral
	SPI2->CR1 &= ~SPI_CR1_SPE;
	// set parameters
	SPI2->CR1 = 	  static_cast<uint32_t>(dataMode)
						| static_cast<uint32_t>(dataOrder)
						| static_cast<uint32_t>(masterSelection)
						| static_cast<uint32_t>(prescaler)
						| static_cast<uint32_t>(dataSize);
	if(masterSelection == MasterSelection::Master) {
		SPI2->CR2 |=  SPI_CR2_SSOE; // for master mode
	}
	// reenable peripheral
	SPI2->CR1 |= SPI_CR1_SPE;
}

void inline
modm::platform::SpiHal2::setDataMode(DataMode dataMode)
{
	SPI2->CR1 = (SPI2->CR1 & ~static_cast<uint32_t>(DataMode::All))
										 | static_cast<uint32_t>(dataMode);
}

void inline
modm::platform::SpiHal2::setDataOrder(DataOrder dataOrder)
{
	SPI2->CR1 = (SPI2->CR1 & ~static_cast<uint32_t>(DataOrder::All))
										 | static_cast<uint32_t>(dataOrder);
}

void inline
modm::platform::SpiHal2::setDataSize(DataSize dataSize)
{
	// TODO: implement as set/reset bit
	SPI2->CR1 = (SPI2->CR1 & ~static_cast<uint32_t>(DataSize::All))
										 | static_cast<uint32_t>(dataSize);
}

void inline
modm::platform::SpiHal2::setMasterSelection(MasterSelection masterSelection)
{
	// TODO: implement as set/reset bit
	SPI2->CR1 = (SPI2->CR1 & ~static_cast<uint32_t>(MasterSelection::All))
										 | static_cast<uint32_t>(masterSelection);
}

inline bool
modm::platform::SpiHal2::isReceiveRegisterNotEmpty()
{
	return static_cast<bool>(getInterruptFlags() & InterruptFlag::RxBufferNotEmpty);
}

inline bool
modm::platform::SpiHal2::isTransmitRegisterEmpty()
{
	return static_cast<bool>(getInterruptFlags() & InterruptFlag::TxBufferEmpty);
}

void inline
modm::platform::SpiHal2::write(uint16_t data)
{
	SPI2->DR = data;
}

void inline
modm::platform::SpiHal2::write(uint8_t data)
{
	write(static_cast<uint16_t>(data));
}

void inline
modm::platform::SpiHal2::read(uint8_t &data)
{
	data = static_cast<uint8_t>(SPI2->DR);
}

void inline
modm::platform::SpiHal2::read(uint16_t &data)
{
	data = static_cast<uint16_t>(SPI2->DR);
}

void inline
modm::platform::SpiHal2::enableInterruptVector(bool enable, uint32_t priority)
{
	if (enable) {
		// Set priority for the interrupt vector
		NVIC_SetPriority(SPI2_IRQn, priority);
		// register IRQ at the NVIC
		NVIC_EnableIRQ(SPI2_IRQn);
	}
	else {
		NVIC_DisableIRQ(SPI2_IRQn);
	}
}

void inline
modm::platform::SpiHal2::enableInterrupt(Interrupt_t interrupt)
{
	SPI2->CR2 |= interrupt.value;
}

void inline
modm::platform::SpiHal2::disableInterrupt(Interrupt_t interrupt)
{
	SPI2->CR2 &= ~interrupt.value;
}

modm::platform::SpiHal2::InterruptFlag_t inline
modm::platform::SpiHal2::getInterruptFlags()
{
	return InterruptFlag_t(SPI2->SR);
}

void inline
modm::platform::SpiHal2::acknowledgeInterruptFlag(InterruptFlag_t /*flags*/)
{
	// TODO: implement; see STM32F3 reference manual p. 736
	// SPI2->SR = flags.value;
}