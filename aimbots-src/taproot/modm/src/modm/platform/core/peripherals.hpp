/*
 * Copyright (c) 2019, Niklas Hauser
 *
 * This file is part of the modm project.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
// ----------------------------------------------------------------------------

#ifndef MODM_STM32_PERIPHERALS_HPP
#define MODM_STM32_PERIPHERALS_HPP

namespace modm::platform
{

enum class
Peripheral
{
	BitBang,
	Adc1,
	Adc2,
	Adc3,
	Can1,
	Can2,
	Crc,
	Dac,
	Dcmi,
	Dma1,
	Dma2,
	Dma2d,
	Eth,
	Flash,
	Fmc,
	I2c1,
	I2c2,
	I2c3,
	I2s,
	I2s2,
	I2s3,
	Iwdg,
	Rcc,
	Rng,
	Rtc,
	Sai1,
	Sdio,
	Spi1,
	Spi2,
	Spi3,
	Spi4,
	Spi5,
	Spi6,
	Sys,
	Tim1,
	Tim10,
	Tim11,
	Tim12,
	Tim13,
	Tim14,
	Tim2,
	Tim3,
	Tim4,
	Tim5,
	Tim6,
	Tim7,
	Tim8,
	Tim9,
	Uart4,
	Uart5,
	Uart7,
	Uart8,
	Usart1,
	Usart2,
	Usart3,
	Usart6,
	Usbotgfs,
	Usbotghs,
	Wwdg,
	Syscfg = Sys,
};

}

#endif // MODM_STM32_PERIPHERALS_HPP