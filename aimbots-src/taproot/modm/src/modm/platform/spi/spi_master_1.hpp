/*
 * Copyright (c) 2009-2011, Fabian Greif
 * Copyright (c) 2010, Martin Rosekeit
 * Copyright (c) 2011-2017, Niklas Hauser
 * Copyright (c) 2012, Georgi Grinshpun
 * Copyright (c) 2013, Kevin Läufer
 * Copyright (c) 2014, Sascha Schade
 * Copyright (c) 2022-2023, Christopher Durand
 *
 * This file is part of the modm project.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
// ----------------------------------------------------------------------------

#ifndef MODM_STM32H7_SPI_MASTER1_HPP
#define MODM_STM32H7_SPI_MASTER1_HPP

#include <modm/architecture/interface/spi_lock.hpp>
#include <modm/architecture/interface/spi_master.hpp>
#include <modm/platform/gpio/connector.hpp>
#include <modm/math/algorithm/prescaler.hpp>
#include <modm/processing/fiber.hpp>
#include "spi_hal_1.hpp"

namespace modm::platform
{

/**
 * Serial peripheral interface (SPI1).
 *
 * @ingroup	modm_platform_spi modm_platform_spi_1
 */
class SpiMaster1 : public modm::SpiMaster, public SpiLock<SpiMaster1>
{
public:
	using Hal = SpiHal1;

	using DataMode = Hal::DataMode;
	using DataOrder = Hal::DataOrder;
	using DataSize = Hal::DataSize;

	template< class... Signals >
	static void
	connect()
	{
		using Connector = GpioConnector<Peripheral::Spi1, Signals...>;
		using Sck = typename Connector::template GetSignal<Gpio::Signal::Sck>;
		using Mosi = typename Connector::template GetSignal<Gpio::Signal::Mosi>;
		using Miso = typename Connector::template GetSignal<Gpio::Signal::Miso>;

		Sck::setOutput(Gpio::OutputType::PushPull);
		Mosi::setOutput(Gpio::OutputType::PushPull);
		Miso::setInput(Gpio::InputType::Floating);
		Connector::connect();
	}

	template< class SystemClock, baudrate_t baudrate, percent_t tolerance=pct(5) >
	static void
	initialize()
	{
		constexpr auto result = modm::Prescaler::from_power(SystemClock::Spi1, baudrate, 2, 256);
		assertBaudrateInTolerance< result.frequency, baudrate, tolerance >();

		// translate the prescaler into the bitmapping
		constexpr Hal::Prescaler prescaler{result.index << SPI_CFG1_MBR_Pos};

		Hal::initialize(prescaler);
		Hal::enableTransfer();
		Hal::startMasterTransfer();
	}

	static void
	setDataMode(DataMode mode)
	{
		finishTransfer();
		Hal::setDataMode(mode);
		Hal::enableTransfer();
		Hal::startMasterTransfer();
	}

	static void
	setDataOrder(DataOrder order)
	{
		finishTransfer();
		Hal::setDataOrder(order);
		Hal::enableTransfer();
		Hal::startMasterTransfer();
	}

	static void
	setDataSize(DataSize size)
	{
		finishTransfer();
		Hal::setDataSize(static_cast<SpiHal1::DataSize>(size));
		Hal::enableTransfer();
		Hal::startMasterTransfer();
	}

	[[deprecated("Use transfer() instead!")]] // DEPRECATED: 2026q3
	static uint8_t
	transferBlocking(uint8_t data)
	{
		return transfer(data);
	}

	[[deprecated("Use transfer() instead!")]] // DEPRECATED: 2026q3
	static void
	transferBlocking(const uint8_t* tx, uint8_t* rx, std::size_t length)
	{
		transfer(tx, rx, length);
	}


	static uint8_t
	transfer(uint8_t data);

	static void
	transfer(const uint8_t* tx, uint8_t* rx, std::size_t length);

private:
	static void
	finishTransfer();
};

} // namespace modm::platform

#endif // MODM_STM32H7_SPI_MASTER1_HPP