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

#ifndef MODM_STM32H7_SPI_HAL3_HPP
#	error 	"Don't include this file directly, use 'spi_hal3.hpp' instead!"
#endif
#include <modm/platform/clock/rcc.hpp>

namespace modm::platform
{

inline void
SpiHal3::enableClock()
{
	Rcc::enable<Peripheral::Spi3>();
}

inline void
SpiHal3::disableClock()
{
	Rcc::disable<Peripheral::Spi3>();
}

inline void
SpiHal3::initialize(Prescaler prescaler,
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

	SPI3->CR1 = 0;

	SPI3->CFG2 = static_cast<uint32_t>(dataMode)
		| static_cast<uint32_t>(dataOrder)
		| static_cast<uint32_t>(masterSelection)
		| SPI_CFG2_SSOE; // disable multi-master support to prevent spurious mode fault

	SPI3->CFG1 = static_cast<uint32_t>(prescaler)
		| static_cast<uint32_t>(dataSize);
}

inline void
SpiHal3::setDataMode(DataMode dataMode)
{
	SPI3->CFG2 = (SPI3->CFG2 & ~static_cast<uint32_t>(DataMode::Mask))
										| static_cast<uint32_t>(dataMode);
}

inline void
SpiHal3::setDataOrder(DataOrder dataOrder)
{
	SPI3->CFG2 = (SPI3->CFG2 & ~static_cast<uint32_t>(DataOrder::Mask))
										| static_cast<uint32_t>(dataOrder);
}

inline void
SpiHal3::setDataSize(DataSize dataSize)
{
	SPI3->CFG1 = (SPI3->CFG1 & ~SPI_CFG1_DSIZE_Msk)
										| static_cast<uint32_t>(dataSize);
}

inline void
SpiHal3::setMasterSelection(MasterSelection masterSelection)
{
	SPI3->CFG2 = (SPI3->CFG2 & ~static_cast<uint32_t>(MasterSelection::Mask))
										| static_cast<uint32_t>(masterSelection);
}

inline void
SpiHal3::setDuplexMode(DuplexMode mode)
{
	SPI3->CFG2 = (SPI3->CFG2 & ~static_cast<uint32_t>(DuplexMode::Mask))
										| static_cast<uint32_t>(mode);
}

inline void
SpiHal3::setCrcEnabled(bool enabled)
{
	if (enabled) {
		SPI3->CFG1 |= SPI_CFG1_CRCEN;
	} else {
		SPI3->CFG1 &= ~SPI_CFG1_CRCEN;
	}
}

inline void
SpiHal3::setCrcSize(DataSize crcSize)
{
	if ((crcSize == DataSize::Bit16) or (crcSize == DataSize::Bit32)) {
		SPI3->CR1 |= SPI_CR1_CRC33_17;
	} else {
		SPI3->CR1 &= ~SPI_CR1_CRC33_17;
	}
	SPI3->CFG1 = (SPI3->CFG1 & ~SPI_CFG1_CRCSIZE_Msk)
										| (static_cast<uint32_t>(crcSize) << SPI_CFG1_CRCSIZE_Pos);
}

inline void
SpiHal3::setCrcPolynomial(uint32_t poly)
{
	SPI3->CRCPOLY = poly;
}

inline void
SpiHal3::setCrcInitialValue(CrcInit init)
{
	SPI3->CR1 = (SPI3->CR1 & ~static_cast<uint32_t>(CrcInit::Mask)) |
		static_cast<uint32_t>(init);
}

inline void
SpiHal3::setTransferSize(uint16_t size, uint16_t reload)
{
	static_assert(SPI_CR2_TSIZE_Pos == 0);
	SPI3->CR2 = (reload << SPI_CR2_TSER_Pos) | size;
}

inline void
SpiHal3::setDmaMode(DmaMode_t mode)
{
	SPI3->CFG1 = (SPI3->CFG1 & ~(DmaMode::Tx | DmaMode::Rx).value)
										| mode.value;
}

inline void
SpiHal3::setSlaveSelectMode(SlaveSelectMode mode)
{
	SPI3->CFG2 = (SPI3->CFG2 & ~SPI_CFG2_SSM_Msk)
										| uint32_t(mode);
}

inline void
SpiHal3::setSlaveSelectPolarity(SlaveSelectPolarity polarity)
{
	SPI3->CFG2 = (SPI3->CFG2 & ~SPI_CFG2_SSIOP_Msk)
										| uint32_t(polarity);
}

inline void
SpiHal3::setSlaveSelectState(bool state)
{
	if (state) {
		SPI3->CR1 |= SPI_CR1_SSI;
	} else  {
		SPI3->CR1 &= ~SPI_CR1_SSI;
	}
}

inline void
SpiHal3::write(uint8_t data)
{
	// Write with 8-bit access
	auto* const ptr = reinterpret_cast<__IO uint8_t*>(&SPI3->TXDR);
	*ptr = data;
}

inline void
SpiHal3::write16(uint16_t data)
{
	// Write with 16-bit access
	// SPI3->TXDR is of type "volatile uint32_t".
	// [[gnu::may_alias]] is required to avoid undefined behaviour due to strict aliasing violations.
	auto* const [[gnu::may_alias]] ptr = reinterpret_cast<__IO uint16_t*>(&SPI3->TXDR);
	*ptr = data;
}

inline void
SpiHal3::write32(uint32_t data)
{
	SPI3->TXDR = data;
}

inline uint8_t
SpiHal3::read()
{
	// Read with 8-bit access
	return *reinterpret_cast<const __IO uint8_t*>(&SPI3->RXDR);
}

inline uint16_t
SpiHal3::read16()
{
	// Read with 16-bit access
	// SPI3->RXDR is of type "const volatile uint32_t".
	// [[gnu::may_alias]] is required to avoid undefined behaviour due to strict aliasing violations.
	auto* const [[gnu::may_alias]] ptr = reinterpret_cast<const __IO uint16_t*>(&SPI3->RXDR);
	return *ptr;
}

inline uint32_t
SpiHal3::read32()
{
	return SPI3->RXDR;
}

inline void
SpiHal3::enableInterruptVector(bool enable, uint32_t priority)
{
	if (enable) {
		// Set priority for the interrupt vector
		NVIC_SetPriority(SPI3_IRQn, priority);
		// register IRQ at the NVIC
		NVIC_EnableIRQ(SPI3_IRQn);
	}
	else {
		NVIC_DisableIRQ(SPI3_IRQn);
	}
}

inline void
SpiHal3::enableInterrupt(Interrupt_t interrupt)
{
	SPI3->IER |= interrupt.value;
}

inline void
SpiHal3::disableInterrupt(Interrupt_t interrupt)
{
	SPI3->IER &= ~interrupt.value;
}

inline void
SpiHal3::acknowledgeInterruptFlags(StatusFlag_t flags)
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

	SPI3->IFCR = flags.value & mask;
}

inline SpiHal3::StatusFlag_t
SpiHal3::status()
{
	return StatusFlag_t(SPI3->SR);
}

inline bool
SpiHal3::isTransferCompleted()
{
	return bool(status() & StatusFlag::EndOfTransfer);
}

inline bool
SpiHal3::isTxCompleted()
{
	return bool(status() & StatusFlag::TxTransferComplete);
}

inline bool
SpiHal3::isTxFifoFull()
{
	return !(status() & StatusFlag::TxPacketSpaceAvailable);
}

inline bool
SpiHal3::isRxDataAvailable()
{
	return bool(status() & StatusFlag::RxPacketAvailable);
}

inline void
SpiHal3::enableTransfer()
{
	SPI3->CR1 |= SPI_CR1_SPE;
}

inline void
SpiHal3::disableTransfer()
{
	SPI3->CR1 &= ~SPI_CR1_SPE;
}

inline bool
SpiHal3::isTransferEnabled()
{
	return (SPI3->CR1 & SPI_CR1_SPE);
}

inline void
SpiHal3::startMasterTransfer()
{
	SPI3->CR1 |= SPI_CR1_CSTART;
}

inline void
SpiHal3::suspendMasterTransfer()
{
	SPI3->CR1 |= SPI_CR1_CSUSP;
}

inline uint32_t
SpiHal3::transmitCrc()
{
	return SPI3->TXCRC;
}

inline uint32_t
SpiHal3::receiveCrc()
{
	return SPI3->RXCRC;
}

inline volatile uint32_t*
SpiHal3::transmitRegister()
{
	return &SPI3->TXDR;
}

inline const volatile uint32_t*
SpiHal3::receiveRegister()
{
	return &SPI3->RXDR;
}

} // namespace modm::platform