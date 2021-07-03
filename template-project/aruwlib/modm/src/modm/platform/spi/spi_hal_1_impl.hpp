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

#ifndef MODM_STM32_SPI_HAL1_HPP
#	error 	"Don't include this file directly, use 'spi_hal1.hpp' instead!"
#endif
#include <modm/platform/clock/rcc.hpp>

void inline
modm::platform::SpiHal1::enable()
{
	Rcc::enable<Peripheral::Spi1>();
	SPI1->CR1 |= SPI_CR1_SPE;		// SPI Enable
}

void inline
modm::platform::SpiHal1::disable()
{
	SPI1->CR1 = 0;
	Rcc::disable<Peripheral::Spi1>();
}

void inline
modm::platform::SpiHal1::initialize(Prescaler prescaler,
		MasterSelection masterSelection, DataMode dataMode,
		DataOrder dataOrder, DataSize dataSize)
{
	enable();
	// disable peripheral
	SPI1->CR1 &= ~SPI_CR1_SPE;
	// set parameters
	SPI1->CR1 = 	  static_cast<uint32_t>(dataMode)
						| static_cast<uint32_t>(dataOrder)
						| static_cast<uint32_t>(masterSelection)
						| static_cast<uint32_t>(prescaler)
						| static_cast<uint32_t>(dataSize);
	if(masterSelection == MasterSelection::Master) {
		SPI1->CR2 |=  SPI_CR2_SSOE; // for master mode
	}
	// reenable peripheral
	SPI1->CR1 |= SPI_CR1_SPE;
}

void inline
modm::platform::SpiHal1::setDataMode(DataMode dataMode)
{
	SPI1->CR1 = (SPI1->CR1 & ~static_cast<uint32_t>(DataMode::All))
										 | static_cast<uint32_t>(dataMode);
}

void inline
modm::platform::SpiHal1::setDataOrder(DataOrder dataOrder)
{
	SPI1->CR1 = (SPI1->CR1 & ~static_cast<uint32_t>(DataOrder::All))
										 | static_cast<uint32_t>(dataOrder);
}

void inline
modm::platform::SpiHal1::setDataSize(DataSize dataSize)
{
	// TODO: implement as set/reset bit
	SPI1->CR1 = (SPI1->CR1 & ~static_cast<uint32_t>(DataSize::All))
										 | static_cast<uint32_t>(dataSize);
}

void inline
modm::platform::SpiHal1::setMasterSelection(MasterSelection masterSelection)
{
	// TODO: implement as set/reset bit
	SPI1->CR1 = (SPI1->CR1 & ~static_cast<uint32_t>(MasterSelection::All))
										 | static_cast<uint32_t>(masterSelection);
}

inline bool
modm::platform::SpiHal1::isReceiveRegisterNotEmpty()
{
	return static_cast<bool>(getInterruptFlags() & InterruptFlag::RxBufferNotEmpty);
}

inline bool
modm::platform::SpiHal1::isTransmitRegisterEmpty()
{
	return static_cast<bool>(getInterruptFlags() & InterruptFlag::TxBufferEmpty);
}

void inline
modm::platform::SpiHal1::write(uint16_t data)
{
	SPI1->DR = data;
}

void inline
modm::platform::SpiHal1::write(uint8_t data)
{
	write(static_cast<uint16_t>(data));
}

void inline
modm::platform::SpiHal1::read(uint8_t &data)
{
	data = static_cast<uint8_t>(SPI1->DR);
}

void inline
modm::platform::SpiHal1::read(uint16_t &data)
{
	data = static_cast<uint16_t>(SPI1->DR);
}

void inline
modm::platform::SpiHal1::enableInterruptVector(bool enable, uint32_t priority)
{
	if (enable) {
		// Set priority for the interrupt vector
		NVIC_SetPriority(SPI1_IRQn, priority);
		// register IRQ at the NVIC
		NVIC_EnableIRQ(SPI1_IRQn);
	}
	else {
		NVIC_DisableIRQ(SPI1_IRQn);
	}
}

void inline
modm::platform::SpiHal1::enableInterrupt(Interrupt_t interrupt)
{
	SPI1->CR2 |= interrupt.value;
}

void inline
modm::platform::SpiHal1::disableInterrupt(Interrupt_t interrupt)
{
	SPI1->CR2 &= ~interrupt.value;
}

modm::platform::SpiHal1::InterruptFlag_t inline
modm::platform::SpiHal1::getInterruptFlags()
{
	return InterruptFlag_t(SPI1->SR);
}

void inline
modm::platform::SpiHal1::acknowledgeInterruptFlag(InterruptFlag_t /*flags*/)
{
	// TODO: implement; see STM32F3 reference manual p. 736
	// SPI1->SR = flags.value;
}