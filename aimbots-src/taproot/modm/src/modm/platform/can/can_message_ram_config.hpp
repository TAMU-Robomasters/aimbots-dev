/*
 * Copyright (c) 2024, Kaelin Laundry
 *
 * This file is part of the modm project.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
// ----------------------------------------------------------------------------


#ifndef MODM_STM32_FDCAN_MESSAGE_RAM_CONFIG_HPP
#define MODM_STM32_FDCAN_MESSAGE_RAM_CONFIG_HPP

namespace modm::platform::fdcan
{


static constexpr uint32_t fifoElementSizeBytes = 2*MessageRamConfig::RamWordSize + modm::can::Message::capacity;
static constexpr MessageRamConfig Fdcan1MessageRamConfig
{
	.filterCountStandard		= 28,
	.filterCountExtended		= 8,
	.rxFifo0Elements			= 14,
	.rxFifo0ElementSizeBytes	= fifoElementSizeBytes,
	.rxFifo1Elements			= 14,
	.rxFifo1ElementSizeBytes	= fifoElementSizeBytes,
    .rxBufferElements			= 0, // Unused
	.rxBufferElementSizeBytes	= fifoElementSizeBytes,
	.txEventFifoElements		= 3, // Currently unused but must be allocated for non-H7 series fixed layout
    .txFifoElements				= 3,
	.txFifoElementSizeBytes		= fifoElementSizeBytes,
};
static constexpr MessageRamConfig Fdcan2MessageRamConfig
{
	.filterCountStandard		= 28,
	.filterCountExtended		= 8,
	.rxFifo0Elements			= 14,
	.rxFifo0ElementSizeBytes	= fifoElementSizeBytes,
	.rxFifo1Elements			= 14,
	.rxFifo1ElementSizeBytes	= fifoElementSizeBytes,
    .rxBufferElements			= 0, // Unused
	.rxBufferElementSizeBytes	= fifoElementSizeBytes,
	.txEventFifoElements		= 3, // Currently unused but must be allocated for non-H7 series fixed layout
    .txFifoElements				= 3,
	.txFifoElementSizeBytes		= fifoElementSizeBytes,
};
static constexpr MessageRamConfig Fdcan3MessageRamConfig
{
	.filterCountStandard		= 28,
	.filterCountExtended		= 8,
	.rxFifo0Elements			= 14,
	.rxFifo0ElementSizeBytes	= fifoElementSizeBytes,
	.rxFifo1Elements			= 14,
	.rxFifo1ElementSizeBytes	= fifoElementSizeBytes,
    .rxBufferElements			= 0, // Unused
	.rxBufferElementSizeBytes	= fifoElementSizeBytes,
	.txEventFifoElements		= 3, // Currently unused but must be allocated for non-H7 series fixed layout
    .txFifoElements				= 3,
	.txFifoElementSizeBytes		= fifoElementSizeBytes,
};
static constexpr uint32_t Fdcan1MessageRamBaseWords = 0;
static constexpr uint32_t Fdcan1MessageRamBoundWords = Fdcan1MessageRamBaseWords + Fdcan1MessageRamConfig.totalSectionWords();

static_assert(Fdcan1MessageRamBoundWords <= 2560);
static constexpr uint32_t Fdcan2MessageRamBaseWords = Fdcan1MessageRamBaseWords + Fdcan1MessageRamConfig.totalSectionWords();
static constexpr uint32_t Fdcan2MessageRamBoundWords = Fdcan2MessageRamBaseWords + Fdcan2MessageRamConfig.totalSectionWords();

static_assert(Fdcan2MessageRamBoundWords <= 2560);
static constexpr uint32_t Fdcan3MessageRamBaseWords = Fdcan2MessageRamBaseWords + Fdcan2MessageRamConfig.totalSectionWords();
static constexpr uint32_t Fdcan3MessageRamBoundWords = Fdcan3MessageRamBaseWords + Fdcan3MessageRamConfig.totalSectionWords();

static_assert(Fdcan3MessageRamBoundWords <= 2560);
}  // modm::platform::fdcan

#endif  // MODM_STM32_FDCAN_MESSAGE_RAM_CONFIG_HPP