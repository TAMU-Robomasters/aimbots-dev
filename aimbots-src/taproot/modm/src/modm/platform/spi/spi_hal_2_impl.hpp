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

#ifndef MODM_STM32H7_SPI_HAL2_HPP
#	error 	"Don't include this file directly, use 'spi_hal2.hpp' instead!"
#endif
#include <modm/platform/clock/rcc.hpp>

namespace modm::platform
{

inline void
SpiHal2::enableClock()
{
	Rcc::enable<Peripheral::Spi2>();
}

inline void
SpiHal2::disableClock()
{
	Rcc::disable<Peripheral::Spi2>();
}

inline void
SpiHal2::initialize(Prescaler prescaler,
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

	SPI2->CR1 = 0;

	SPI2->CFG2 = static_cast<uint32_t>(dataMode)
		| static_cast<uint32_t>(dataOrder)
		| static_cast<uint32_t>(masterSelection)
		| SPI_CFG2_SSOE; // disable multi-master support to prevent spurious mode fault

	SPI2->CFG1 = static_cast<uint32_t>(prescaler)
		| static_cast<uint32_t>(dataSize);
}

inline void
SpiHal2::setDataMode(DataMode dataMode)
{
	SPI2->CFG2 = (SPI2->CFG2 & ~static_cast<uint32_t>(DataMode::Mask))
										| static_cast<uint32_t>(dataMode);
}

inline void
SpiHal2::setDataOrder(DataOrder dataOrder)
{
	SPI2->CFG2 = (SPI2->CFG2 & ~static_cast<uint32_t>(DataOrder::Mask))
										| static_cast<uint32_t>(dataOrder);
}

inline void
SpiHal2::setDataSize(DataSize dataSize)
{
	SPI2->CFG1 = (SPI2->CFG1 & ~SPI_CFG1_DSIZE_Msk)
										| static_cast<uint32_t>(dataSize);
}

inline void
SpiHal2::setMasterSelection(MasterSelection masterSelection)
{
	SPI2->CFG2 = (SPI2->CFG2 & ~static_cast<uint32_t>(MasterSelection::Mask))
										| static_cast<uint32_t>(masterSelection);
}

inline void
SpiHal2::setDuplexMode(DuplexMode mode)
{
	SPI2->CFG2 = (SPI2->CFG2 & ~static_cast<uint32_t>(DuplexMode::Mask))
										| static_cast<uint32_t>(mode);
}

inline void
SpiHal2::setCrcEnabled(bool enabled)
{
	if (enabled) {
		SPI2->CFG1 |= SPI_CFG1_CRCEN;
	} else {
		SPI2->CFG1 &= ~SPI_CFG1_CRCEN;
	}
}

inline void
SpiHal2::setCrcSize(DataSize crcSize)
{
	if ((crcSize == DataSize::Bit16) or (crcSize == DataSize::Bit32)) {
		SPI2->CR1 |= SPI_CR1_CRC33_17;
	} else {
		SPI2->CR1 &= ~SPI_CR1_CRC33_17;
	}
	SPI2->CFG1 = (SPI2->CFG1 & ~SPI_CFG1_CRCSIZE_Msk)
										| (static_cast<uint32_t>(crcSize) << SPI_CFG1_CRCSIZE_Pos);
}

inline void
SpiHal2::setCrcPolynomial(uint32_t poly)
{
	SPI2->CRCPOLY = poly;
}

inline void
SpiHal2::setCrcInitialValue(CrcInit init)
{
	SPI2->CR1 = (SPI2->CR1 & ~static_cast<uint32_t>(CrcInit::Mask)) |
		static_cast<uint32_t>(init);
}

inline void
SpiHal2::setTransferSize(uint16_t size, uint16_t reload)
{
	static_assert(SPI_CR2_TSIZE_Pos == 0);
	SPI2->CR2 = (reload << SPI_CR2_TSER_Pos) | size;
}

inline void
SpiHal2::setDmaMode(DmaMode_t mode)
{
	SPI2->CFG1 = (SPI2->CFG1 & ~(DmaMode::Tx | DmaMode::Rx).value)
										| mode.value;
}

inline void
SpiHal2::setSlaveSelectMode(SlaveSelectMode mode)
{
	SPI2->CFG2 = (SPI2->CFG2 & ~SPI_CFG2_SSM_Msk)
										| uint32_t(mode);
}

inline void
SpiHal2::setSlaveSelectPolarity(SlaveSelectPolarity polarity)
{
	SPI2->CFG2 = (SPI2->CFG2 & ~SPI_CFG2_SSIOP_Msk)
										| uint32_t(polarity);
}

inline void
SpiHal2::setSlaveSelectState(bool state)
{
	if (state) {
		SPI2->CR1 |= SPI_CR1_SSI;
	} else  {
		SPI2->CR1 &= ~SPI_CR1_SSI;
	}
}

inline void
SpiHal2::write(uint8_t data)
{
	// Write with 8-bit access
	auto* const ptr = reinterpret_cast<__IO uint8_t*>(&SPI2->TXDR);
	*ptr = data;
}

inline void
SpiHal2::write16(uint16_t data)
{
	// Write with 16-bit access
	// SPI2->TXDR is of type "volatile uint32_t".
	// [[gnu::may_alias]] is required to avoid undefined behaviour due to strict aliasing violations.
	auto* const [[gnu::may_alias]] ptr = reinterpret_cast<__IO uint16_t*>(&SPI2->TXDR);
	*ptr = data;
}

inline void
SpiHal2::write32(uint32_t data)
{
	SPI2->TXDR = data;
}

inline uint8_t
SpiHal2::read()
{
	// Read with 8-bit access
	return *reinterpret_cast<const __IO uint8_t*>(&SPI2->RXDR);
}

inline uint16_t
SpiHal2::read16()
{
	// Read with 16-bit access
	// SPI2->RXDR is of type "const volatile uint32_t".
	// [[gnu::may_alias]] is required to avoid undefined behaviour due to strict aliasing violations.
	auto* const [[gnu::may_alias]] ptr = reinterpret_cast<const __IO uint16_t*>(&SPI2->RXDR);
	return *ptr;
}

inline uint32_t
SpiHal2::read32()
{
	return SPI2->RXDR;
}

inline void
SpiHal2::enableInterruptVector(bool enable, uint32_t priority)
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

inline void
SpiHal2::enableInterrupt(Interrupt_t interrupt)
{
	SPI2->IER |= interrupt.value;
}

inline void
SpiHal2::disableInterrupt(Interrupt_t interrupt)
{
	SPI2->IER &= ~interrupt.value;
}

inline void
SpiHal2::acknowledgeInterruptFlags(StatusFlag_t flags)
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

	SPI2->IFCR = flags.value & mask;
}

inline SpiHal2::StatusFlag_t
SpiHal2::status()
{
	return StatusFlag_t(SPI2->SR);
}

inline bool
SpiHal2::isTransferCompleted()
{
	return bool(status() & StatusFlag::EndOfTransfer);
}

inline bool
SpiHal2::isTxCompleted()
{
	return bool(status() & StatusFlag::TxTransferComplete);
}

inline bool
SpiHal2::isTxFifoFull()
{
	return !(status() & StatusFlag::TxPacketSpaceAvailable);
}

inline bool
SpiHal2::isRxDataAvailable()
{
	return bool(status() & StatusFlag::RxPacketAvailable);
}

inline void
SpiHal2::enableTransfer()
{
	SPI2->CR1 |= SPI_CR1_SPE;
}

inline void
SpiHal2::disableTransfer()
{
	SPI2->CR1 &= ~SPI_CR1_SPE;
}

inline bool
SpiHal2::isTransferEnabled()
{
	return (SPI2->CR1 & SPI_CR1_SPE);
}

inline void
SpiHal2::startMasterTransfer()
{
	SPI2->CR1 |= SPI_CR1_CSTART;
}

inline void
SpiHal2::suspendMasterTransfer()
{
	SPI2->CR1 |= SPI_CR1_CSUSP;
}

inline uint32_t
SpiHal2::transmitCrc()
{
	return SPI2->TXCRC;
}

inline uint32_t
SpiHal2::receiveCrc()
{
	return SPI2->RXCRC;
}

inline volatile uint32_t*
SpiHal2::transmitRegister()
{
	return &SPI2->TXDR;
}

inline const volatile uint32_t*
SpiHal2::receiveRegister()
{
	return &SPI2->RXDR;
}

} // namespace modm::platform