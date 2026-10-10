/*
* Copyright (c) 2013, Kevin Läufer
* Copyright (c) 2013-2017, Niklas Hauser
* Copyright (c) 2014, Daniel Krebs
* Copyright (c) 2020, Mike Wolfram
* Copyright (c) 2023, Christopher Durand
*
* This file is part of the modm project.
*
* This Source Code Form is subject to the terms of the Mozilla Public
* License, v. 2.0. If a copy of the MPL was not distributed with this
* file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/
// ----------------------------------------------------------------------------

#ifndef MODM_STM32H7_SPI_HAL1_HPP
#	error 	"Don't include this file directly, use 'spi_hal1.hpp' instead!"
#endif
#include <modm/platform/clock/rcc.hpp>

namespace modm::platform
{

inline void
SpiHal1::enableClock()
{
	Rcc::enable<Peripheral::Spi1>();
}

inline void
SpiHal1::disableClock()
{
	Rcc::disable<Peripheral::Spi1>();
}

inline void
SpiHal1::initialize(Prescaler prescaler,
		MasterSelection masterSelection, DataMode dataMode,
		DataOrder dataOrder, DataSize dataSize)
{
	enableClock();
	disableTransfer();

	acknowledgeInterruptFlags(
		StatusFlag::EndOfTransfer |
		StatusFlag::TxTransferFilled |
		StatusFlag::Underrun |
		StatusFlag::Overrun |
		StatusFlag::CrcError |
		StatusFlag::TiFrameError |
		StatusFlag::ModeFault |
		StatusFlag::Reload |
		StatusFlag::Suspension
	);

	// initialize with unlimited transfer size
	setTransferSize(0);

	SPI1->CR1 = 0;

	SPI1->CFG2 = static_cast<uint32_t>(dataMode)
		| static_cast<uint32_t>(dataOrder)
		| static_cast<uint32_t>(masterSelection)
		| SPI_CFG2_SSOE; // disable multi-master support to prevent spurious mode fault

	SPI1->CFG1 = static_cast<uint32_t>(prescaler)
		| static_cast<uint32_t>(dataSize);
}

inline void
SpiHal1::setDataMode(DataMode dataMode)
{
	SPI1->CFG2 = (SPI1->CFG2 & ~static_cast<uint32_t>(DataMode::Mask))
										| static_cast<uint32_t>(dataMode);
}

inline void
SpiHal1::setDataOrder(DataOrder dataOrder)
{
	SPI1->CFG2 = (SPI1->CFG2 & ~static_cast<uint32_t>(DataOrder::Mask))
										| static_cast<uint32_t>(dataOrder);
}

inline void
SpiHal1::setDataSize(DataSize dataSize)
{
	SPI1->CFG1 = (SPI1->CFG1 & ~SPI_CFG1_DSIZE_Msk)
										| static_cast<uint32_t>(dataSize);
}

inline void
SpiHal1::setMasterSelection(MasterSelection masterSelection)
{
	SPI1->CFG2 = (SPI1->CFG2 & ~static_cast<uint32_t>(MasterSelection::Mask))
										| static_cast<uint32_t>(masterSelection);
}

inline void
SpiHal1::setDuplexMode(DuplexMode mode)
{
	SPI1->CFG2 = (SPI1->CFG2 & ~static_cast<uint32_t>(DuplexMode::Mask))
										| static_cast<uint32_t>(mode);
}

inline void
SpiHal1::setCrcEnabled(bool enabled)
{
	if (enabled) {
		SPI1->CFG1 |= SPI_CFG1_CRCEN;
	} else {
		SPI1->CFG1 &= ~SPI_CFG1_CRCEN;
	}
}

inline void
SpiHal1::setCrcSize(DataSize crcSize)
{
	if ((crcSize == DataSize::Bit16) or (crcSize == DataSize::Bit32)) {
		SPI1->CR1 |= SPI_CR1_CRC33_17;
	} else {
		SPI1->CR1 &= ~SPI_CR1_CRC33_17;
	}
	SPI1->CFG1 = (SPI1->CFG1 & ~SPI_CFG1_CRCSIZE_Msk)
										| (static_cast<uint32_t>(crcSize) << SPI_CFG1_CRCSIZE_Pos);
}

inline void
SpiHal1::setCrcPolynomial(uint32_t poly)
{
	SPI1->CRCPOLY = poly;
}

inline void
SpiHal1::setCrcInitialValue(CrcInit init)
{
	SPI1->CR1 = (SPI1->CR1 & ~static_cast<uint32_t>(CrcInit::Mask)) |
		static_cast<uint32_t>(init);
}

inline void
SpiHal1::setTransferSize(uint16_t size, uint16_t reload)
{
	static_assert(SPI_CR2_TSIZE_Pos == 0);
	SPI1->CR2 = (reload << SPI_CR2_TSER_Pos) | size;
}

inline void
SpiHal1::setDmaMode(DmaMode_t mode)
{
	SPI1->CFG1 = (SPI1->CFG1 & ~(DmaMode::Tx | DmaMode::Rx).value)
										| mode.value;
}

inline void
SpiHal1::setSlaveSelectMode(SlaveSelectMode mode)
{
	SPI1->CFG2 = (SPI1->CFG2 & ~SPI_CFG2_SSM_Msk)
										| uint32_t(mode);
}

inline void
SpiHal1::setSlaveSelectPolarity(SlaveSelectPolarity polarity)
{
	SPI1->CFG2 = (SPI1->CFG2 & ~SPI_CFG2_SSIOP_Msk)
										| uint32_t(polarity);
}

inline void
SpiHal1::setSlaveSelectState(bool state)
{
	if (state) {
		SPI1->CR1 |= SPI_CR1_SSI;
	} else  {
		SPI1->CR1 &= ~SPI_CR1_SSI;
	}
}

inline void
SpiHal1::write(uint8_t data)
{
	// Write with 8-bit access
	auto* const ptr = reinterpret_cast<__IO uint8_t*>(&SPI1->TXDR);
	*ptr = data;
}

inline void
SpiHal1::write16(uint16_t data)
{
	// Write with 16-bit access
	// SPI1->TXDR is of type "volatile uint32_t".
	// [[gnu::may_alias]] is required to avoid undefined behaviour due to strict aliasing violations.
	auto* const [[gnu::may_alias]] ptr = reinterpret_cast<__IO uint16_t*>(&SPI1->TXDR);
	*ptr = data;
}

inline void
SpiHal1::write32(uint32_t data)
{
	SPI1->TXDR = data;
}

inline uint8_t
SpiHal1::read()
{
	// Read with 8-bit access
	return *reinterpret_cast<const __IO uint8_t*>(&SPI1->RXDR);
}

inline uint16_t
SpiHal1::read16()
{
	// Read with 16-bit access
	// SPI1->RXDR is of type "const volatile uint32_t".
	// [[gnu::may_alias]] is required to avoid undefined behaviour due to strict aliasing violations.
	auto* const [[gnu::may_alias]] ptr = reinterpret_cast<const __IO uint16_t*>(&SPI1->RXDR);
	return *ptr;
}

inline uint32_t
SpiHal1::read32()
{
	return SPI1->RXDR;
}

inline void
SpiHal1::enableInterruptVector(bool enable, uint32_t priority)
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

inline void
SpiHal1::enableInterrupt(Interrupt_t interrupt)
{
	SPI1->IER |= interrupt.value;
}

inline void
SpiHal1::disableInterrupt(Interrupt_t interrupt)
{
	SPI1->IER &= ~interrupt.value;
}

inline void
SpiHal1::acknowledgeInterruptFlags(StatusFlag_t flags)
{
	constexpr auto mask =
		SPI_IFCR_EOTC |
		SPI_IFCR_TXTFC |
		SPI_IFCR_UDRC |
		SPI_IFCR_OVRC |
		SPI_IFCR_CRCEC |
		SPI_IFCR_TIFREC |
		SPI_IFCR_MODFC |
		SPI_IFCR_TSERFC |
		SPI_IFCR_SUSPC;

	SPI1->IFCR = flags.value & mask;
}

inline SpiHal1::StatusFlag_t
SpiHal1::status()
{
	return StatusFlag_t(SPI1->SR);
}

inline bool
SpiHal1::isTransferCompleted()
{
	return bool(status() & StatusFlag::EndOfTransfer);
}

inline bool
SpiHal1::isTxCompleted()
{
	return bool(status() & StatusFlag::TxTransferComplete);
}

inline bool
SpiHal1::isTxFifoFull()
{
	return !(status() & StatusFlag::TxPacketSpaceAvailable);
}

inline bool
SpiHal1::isRxDataAvailable()
{
	return bool(status() & StatusFlag::RxPacketAvailable);
}

inline void
SpiHal1::enableTransfer()
{
	SPI1->CR1 |= SPI_CR1_SPE;
}

inline void
SpiHal1::disableTransfer()
{
	SPI1->CR1 &= ~SPI_CR1_SPE;
}

inline bool
SpiHal1::isTransferEnabled()
{
	return (SPI1->CR1 & SPI_CR1_SPE);
}

inline void
SpiHal1::startMasterTransfer()
{
	SPI1->CR1 |= SPI_CR1_CSTART;
}

inline void
SpiHal1::suspendMasterTransfer()
{
	SPI1->CR1 |= SPI_CR1_CSUSP;
}

inline uint32_t
SpiHal1::transmitCrc()
{
	return SPI1->TXCRC;
}

inline uint32_t
SpiHal1::receiveCrc()
{
	return SPI1->RXCRC;
}

inline volatile uint32_t*
SpiHal1::transmitRegister()
{
	return &SPI1->TXDR;
}

inline const volatile uint32_t*
SpiHal1::receiveRegister()
{
	return &SPI1->RXDR;
}

} // namespace modm::platform