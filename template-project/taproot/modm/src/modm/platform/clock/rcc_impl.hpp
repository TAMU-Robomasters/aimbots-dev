/*
 * Copyright (c) 2019, 2021 Niklas Hauser
 *
 * This file is part of the modm project.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
// ----------------------------------------------------------------------------

namespace modm::platform
{

constexpr Rcc::flash_latency
Rcc::computeFlashLatency(uint32_t Core_Hz, uint16_t Core_mV)
{
	constexpr uint32_t flash_latency_1800[] =
	{
		20000000,
		40000000,
		60000000,
		80000000,
		100000000,
		120000000,
		140000000,
		160000000,
		168000000,
	};
	constexpr uint32_t flash_latency_2100[] =
	{
		22000000,
		44000000,
		66000000,
		88000000,
		110000000,
		132000000,
		154000000,
		176000000,
		180000000,
	};
	constexpr uint32_t flash_latency_2400[] =
	{
		24000000,
		48000000,
		72000000,
		96000000,
		120000000,
		144000000,
		168000000,
		180000000,
	};
	constexpr uint32_t flash_latency_2700[] =
	{
		30000000,
		60000000,
		90000000,
		120000000,
		150000000,
		180000000,
	};
	const uint32_t *lut(flash_latency_1800);
	uint8_t lut_size(sizeof(flash_latency_1800) / sizeof(uint32_t));
	// find the right table for the voltage
	if (2700 <= Core_mV) {
		lut = flash_latency_2700;
		lut_size = sizeof(flash_latency_2700) / sizeof(uint32_t);
	}
	else if (2400 <= Core_mV) {
		lut = flash_latency_2400;
		lut_size = sizeof(flash_latency_2400) / sizeof(uint32_t);
	}
	else if (2100 <= Core_mV) {
		lut = flash_latency_2100;
		lut_size = sizeof(flash_latency_2100) / sizeof(uint32_t);
	}
	// find the next highest frequency in the table
	uint8_t latency(0);
	uint32_t max_freq(0);
	while (latency < lut_size)
	{
		if (Core_Hz <= (max_freq = lut[latency]))
			break;
		latency++;
	}
	return {latency, max_freq};
}

template< uint32_t Core_Hz, uint16_t Core_mV = 3300 >
uint32_t
Rcc::setFlashLatency()
{
	constexpr flash_latency fl = computeFlashLatency(Core_Hz, Core_mV);
	static_assert(Core_Hz <= fl.max_frequency, "CPU Frequency is too high for this core voltage!");

	uint32_t acr = FLASH->ACR & ~FLASH_ACR_LATENCY;
	// set flash latency
	acr |= fl.latency;
	// enable flash prefetch and data and instruction cache
	acr |= FLASH_ACR_PRFTEN | FLASH_ACR_DCEN | FLASH_ACR_ICEN;
	FLASH->ACR = acr;
	__DSB(); __ISB();
	return fl.max_frequency;
}

template< uint32_t Core_Hz >
void
Rcc::updateCoreFrequency()
{
	SystemCoreClock = Core_Hz;
	delay_fcpu_MHz = computeDelayMhz(Core_Hz);
	delay_ns_per_loop = computeDelayNsPerLoop(Core_Hz);
}

constexpr bool
rcc_check_enable(Peripheral peripheral)
{
	switch(peripheral) {
		case Peripheral::Adc1:
		case Peripheral::Adc2:
		case Peripheral::Adc3:
		case Peripheral::Can1:
		case Peripheral::Can2:
		case Peripheral::Crc:
		case Peripheral::Dac:
		case Peripheral::Dcmi:
		case Peripheral::Dma1:
		case Peripheral::Dma2:
		case Peripheral::Dma2d:
		case Peripheral::Eth:
		case Peripheral::Fmc:
		case Peripheral::I2c1:
		case Peripheral::I2c2:
		case Peripheral::I2c3:
		case Peripheral::Rng:
		case Peripheral::Rtc:
		case Peripheral::Sai1:
		case Peripheral::Sdio:
		case Peripheral::Spi1:
		case Peripheral::Spi2:
		case Peripheral::Spi3:
		case Peripheral::Spi4:
		case Peripheral::Spi5:
		case Peripheral::Spi6:
		case Peripheral::Tim1:
		case Peripheral::Tim10:
		case Peripheral::Tim11:
		case Peripheral::Tim12:
		case Peripheral::Tim13:
		case Peripheral::Tim14:
		case Peripheral::Tim2:
		case Peripheral::Tim3:
		case Peripheral::Tim4:
		case Peripheral::Tim5:
		case Peripheral::Tim6:
		case Peripheral::Tim7:
		case Peripheral::Tim8:
		case Peripheral::Tim9:
		case Peripheral::Uart4:
		case Peripheral::Uart5:
		case Peripheral::Uart7:
		case Peripheral::Uart8:
		case Peripheral::Usart1:
		case Peripheral::Usart2:
		case Peripheral::Usart3:
		case Peripheral::Usart6:
		case Peripheral::Usbotgfs:
		case Peripheral::Usbotghs:
		case Peripheral::Wwdg:
			return true;
		default:
			return false;
	}
}

template< Peripheral peripheral >
void
Rcc::enable()
{
	static_assert(rcc_check_enable(peripheral),
		"Rcc::enable() doesn't know this peripheral!");

	__DSB();
	if constexpr (peripheral == Peripheral::Adc1)
		if (not Rcc::isEnabled<peripheral>()) {
			RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
		}
	if constexpr (peripheral == Peripheral::Adc2)
		if (not Rcc::isEnabled<peripheral>()) {
			RCC->APB2ENR |= RCC_APB2ENR_ADC2EN;
		}
	if constexpr (peripheral == Peripheral::Adc3)
		if (not Rcc::isEnabled<peripheral>()) {
			RCC->APB2ENR |= RCC_APB2ENR_ADC3EN;
		}
	if constexpr (peripheral == Peripheral::Can1)
		if (not Rcc::isEnabled<peripheral>()) {
			RCC->APB1ENR |= RCC_APB1ENR_CAN1EN; __DSB();
			RCC->APB1RSTR |= RCC_APB1RSTR_CAN1RST; __DSB();
			RCC->APB1RSTR &= ~RCC_APB1RSTR_CAN1RST;
		}
	if constexpr (peripheral == Peripheral::Can2)
		if (not Rcc::isEnabled<peripheral>()) {
			RCC->APB1ENR |= RCC_APB1ENR_CAN2EN; __DSB();
			RCC->APB1RSTR |= RCC_APB1RSTR_CAN2RST; __DSB();
			RCC->APB1RSTR &= ~RCC_APB1RSTR_CAN2RST;
		}
	if constexpr (peripheral == Peripheral::Crc)
		if (not Rcc::isEnabled<peripheral>()) {
			RCC->AHB1ENR |= RCC_AHB1ENR_CRCEN; __DSB();
			RCC->AHB1RSTR |= RCC_AHB1RSTR_CRCRST; __DSB();
			RCC->AHB1RSTR &= ~RCC_AHB1RSTR_CRCRST;
		}
	if constexpr (peripheral == Peripheral::Dac)
		if (not Rcc::isEnabled<peripheral>()) {
			RCC->APB1ENR |= RCC_APB1ENR_DACEN; __DSB();
			RCC->APB1RSTR |= RCC_APB1RSTR_DACRST; __DSB();
			RCC->APB1RSTR &= ~RCC_APB1RSTR_DACRST;
		}
	if constexpr (peripheral == Peripheral::Dcmi)
		if (not Rcc::isEnabled<peripheral>()) {
			RCC->AHB2ENR |= RCC_AHB2ENR_DCMIEN; __DSB();
			RCC->AHB2RSTR |= RCC_AHB2RSTR_DCMIRST; __DSB();
			RCC->AHB2RSTR &= ~RCC_AHB2RSTR_DCMIRST;
		}
	if constexpr (peripheral == Peripheral::Dma1)
		if (not Rcc::isEnabled<peripheral>()) {
			RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN; __DSB();
			RCC->AHB1RSTR |= RCC_AHB1RSTR_DMA1RST; __DSB();
			RCC->AHB1RSTR &= ~RCC_AHB1RSTR_DMA1RST;
		}
	if constexpr (peripheral == Peripheral::Dma2)
		if (not Rcc::isEnabled<peripheral>()) {
			RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN; __DSB();
			RCC->AHB1RSTR |= RCC_AHB1RSTR_DMA2RST; __DSB();
			RCC->AHB1RSTR &= ~RCC_AHB1RSTR_DMA2RST;
		}
	if constexpr (peripheral == Peripheral::Dma2d)
		if (not Rcc::isEnabled<peripheral>()) {
			RCC->AHB1ENR |= RCC_AHB1ENR_DMA2DEN; __DSB();
			RCC->AHB1RSTR |= RCC_AHB1RSTR_DMA2DRST; __DSB();
			RCC->AHB1RSTR &= ~RCC_AHB1RSTR_DMA2DRST;
		}
	if constexpr (peripheral == Peripheral::Eth)
		if (not Rcc::isEnabled<peripheral>()) {
			RCC->AHB1ENR |= RCC_AHB1ENR_ETHMACEN; __DSB();
			RCC->AHB1RSTR |= RCC_AHB1RSTR_ETHMACRST; __DSB();
			RCC->AHB1RSTR &= ~RCC_AHB1RSTR_ETHMACRST; __DSB();
			RCC->AHB1ENR |= RCC_AHB1ENR_ETHMACRXEN; __DSB();
			RCC->AHB1ENR |= RCC_AHB1ENR_ETHMACTXEN;
		}
	if constexpr (peripheral == Peripheral::Fmc)
		if (not Rcc::isEnabled<peripheral>()) {
			RCC->AHB3ENR |= RCC_AHB3ENR_FMCEN; __DSB();
			RCC->AHB3RSTR |= RCC_AHB3RSTR_FMCRST; __DSB();
			RCC->AHB3RSTR &= ~RCC_AHB3RSTR_FMCRST;
		}
	if constexpr (peripheral == Peripheral::I2c1)
		if (not Rcc::isEnabled<peripheral>()) {
			RCC->APB1ENR |= RCC_APB1ENR_I2C1EN; __DSB();
			RCC->APB1RSTR |= RCC_APB1RSTR_I2C1RST; __DSB();
			RCC->APB1RSTR &= ~RCC_APB1RSTR_I2C1RST;
		}
	if constexpr (peripheral == Peripheral::I2c2)
		if (not Rcc::isEnabled<peripheral>()) {
			RCC->APB1ENR |= RCC_APB1ENR_I2C2EN; __DSB();
			RCC->APB1RSTR |= RCC_APB1RSTR_I2C2RST; __DSB();
			RCC->APB1RSTR &= ~RCC_APB1RSTR_I2C2RST;
		}
	if constexpr (peripheral == Peripheral::I2c3)
		if (not Rcc::isEnabled<peripheral>()) {
			RCC->APB1ENR |= RCC_APB1ENR_I2C3EN; __DSB();
			RCC->APB1RSTR |= RCC_APB1RSTR_I2C3RST; __DSB();
			RCC->APB1RSTR &= ~RCC_APB1RSTR_I2C3RST;
		}
	if constexpr (peripheral == Peripheral::Rng)
		if (not Rcc::isEnabled<peripheral>()) {
			RCC->AHB2ENR |= RCC_AHB2ENR_RNGEN; __DSB();
			RCC->AHB2RSTR |= RCC_AHB2RSTR_RNGRST; __DSB();
			RCC->AHB2RSTR &= ~RCC_AHB2RSTR_RNGRST;
		}
	if constexpr (peripheral == Peripheral::Rtc)
		if (not Rcc::isEnabled<peripheral>()) {
			RCC->BDCR |= RCC_BDCR_RTCEN;
		}
	if constexpr (peripheral == Peripheral::Sai1)
		if (not Rcc::isEnabled<peripheral>()) {
			RCC->APB2ENR |= RCC_APB2ENR_SAI1EN; __DSB();
			RCC->APB2RSTR |= RCC_APB2RSTR_SAI1RST; __DSB();
			RCC->APB2RSTR &= ~RCC_APB2RSTR_SAI1RST;
		}
	if constexpr (peripheral == Peripheral::Sdio)
		if (not Rcc::isEnabled<peripheral>()) {
			RCC->APB2ENR |= RCC_APB2ENR_SDIOEN; __DSB();
			RCC->APB2RSTR |= RCC_APB2RSTR_SDIORST; __DSB();
			RCC->APB2RSTR &= ~RCC_APB2RSTR_SDIORST;
		}
	if constexpr (peripheral == Peripheral::Spi1)
		if (not Rcc::isEnabled<peripheral>()) {
			RCC->APB2ENR |= RCC_APB2ENR_SPI1EN; __DSB();
			RCC->APB2RSTR |= RCC_APB2RSTR_SPI1RST; __DSB();
			RCC->APB2RSTR &= ~RCC_APB2RSTR_SPI1RST;
		}
	if constexpr (peripheral == Peripheral::Spi2)
		if (not Rcc::isEnabled<peripheral>()) {
			RCC->APB1ENR |= RCC_APB1ENR_SPI2EN; __DSB();
			RCC->APB1RSTR |= RCC_APB1RSTR_SPI2RST; __DSB();
			RCC->APB1RSTR &= ~RCC_APB1RSTR_SPI2RST;
		}
	if constexpr (peripheral == Peripheral::Spi3)
		if (not Rcc::isEnabled<peripheral>()) {
			RCC->APB1ENR |= RCC_APB1ENR_SPI3EN; __DSB();
			RCC->APB1RSTR |= RCC_APB1RSTR_SPI3RST; __DSB();
			RCC->APB1RSTR &= ~RCC_APB1RSTR_SPI3RST;
		}
	if constexpr (peripheral == Peripheral::Spi4)
		if (not Rcc::isEnabled<peripheral>()) {
			RCC->APB2ENR |= RCC_APB2ENR_SPI4EN; __DSB();
			RCC->APB2RSTR |= RCC_APB2RSTR_SPI4RST; __DSB();
			RCC->APB2RSTR &= ~RCC_APB2RSTR_SPI4RST;
		}
	if constexpr (peripheral == Peripheral::Spi5)
		if (not Rcc::isEnabled<peripheral>()) {
			RCC->APB2ENR |= RCC_APB2ENR_SPI5EN; __DSB();
			RCC->APB2RSTR |= RCC_APB2RSTR_SPI5RST; __DSB();
			RCC->APB2RSTR &= ~RCC_APB2RSTR_SPI5RST;
		}
	if constexpr (peripheral == Peripheral::Spi6)
		if (not Rcc::isEnabled<peripheral>()) {
			RCC->APB2ENR |= RCC_APB2ENR_SPI6EN; __DSB();
			RCC->APB2RSTR |= RCC_APB2RSTR_SPI6RST; __DSB();
			RCC->APB2RSTR &= ~RCC_APB2RSTR_SPI6RST;
		}
	if constexpr (peripheral == Peripheral::Tim1)
		if (not Rcc::isEnabled<peripheral>()) {
			RCC->APB2ENR |= RCC_APB2ENR_TIM1EN; __DSB();
			RCC->APB2RSTR |= RCC_APB2RSTR_TIM1RST; __DSB();
			RCC->APB2RSTR &= ~RCC_APB2RSTR_TIM1RST;
		}
	if constexpr (peripheral == Peripheral::Tim10)
		if (not Rcc::isEnabled<peripheral>()) {
			RCC->APB2ENR |= RCC_APB2ENR_TIM10EN; __DSB();
			RCC->APB2RSTR |= RCC_APB2RSTR_TIM10RST; __DSB();
			RCC->APB2RSTR &= ~RCC_APB2RSTR_TIM10RST;
		}
	if constexpr (peripheral == Peripheral::Tim11)
		if (not Rcc::isEnabled<peripheral>()) {
			RCC->APB2ENR |= RCC_APB2ENR_TIM11EN; __DSB();
			RCC->APB2RSTR |= RCC_APB2RSTR_TIM11RST; __DSB();
			RCC->APB2RSTR &= ~RCC_APB2RSTR_TIM11RST;
		}
	if constexpr (peripheral == Peripheral::Tim12)
		if (not Rcc::isEnabled<peripheral>()) {
			RCC->APB1ENR |= RCC_APB1ENR_TIM12EN; __DSB();
			RCC->APB1RSTR |= RCC_APB1RSTR_TIM12RST; __DSB();
			RCC->APB1RSTR &= ~RCC_APB1RSTR_TIM12RST;
		}
	if constexpr (peripheral == Peripheral::Tim13)
		if (not Rcc::isEnabled<peripheral>()) {
			RCC->APB1ENR |= RCC_APB1ENR_TIM13EN; __DSB();
			RCC->APB1RSTR |= RCC_APB1RSTR_TIM13RST; __DSB();
			RCC->APB1RSTR &= ~RCC_APB1RSTR_TIM13RST;
		}
	if constexpr (peripheral == Peripheral::Tim14)
		if (not Rcc::isEnabled<peripheral>()) {
			RCC->APB1ENR |= RCC_APB1ENR_TIM14EN; __DSB();
			RCC->APB1RSTR |= RCC_APB1RSTR_TIM14RST; __DSB();
			RCC->APB1RSTR &= ~RCC_APB1RSTR_TIM14RST;
		}
	if constexpr (peripheral == Peripheral::Tim2)
		if (not Rcc::isEnabled<peripheral>()) {
			RCC->APB1ENR |= RCC_APB1ENR_TIM2EN; __DSB();
			RCC->APB1RSTR |= RCC_APB1RSTR_TIM2RST; __DSB();
			RCC->APB1RSTR &= ~RCC_APB1RSTR_TIM2RST;
		}
	if constexpr (peripheral == Peripheral::Tim3)
		if (not Rcc::isEnabled<peripheral>()) {
			RCC->APB1ENR |= RCC_APB1ENR_TIM3EN; __DSB();
			RCC->APB1RSTR |= RCC_APB1RSTR_TIM3RST; __DSB();
			RCC->APB1RSTR &= ~RCC_APB1RSTR_TIM3RST;
		}
	if constexpr (peripheral == Peripheral::Tim4)
		if (not Rcc::isEnabled<peripheral>()) {
			RCC->APB1ENR |= RCC_APB1ENR_TIM4EN; __DSB();
			RCC->APB1RSTR |= RCC_APB1RSTR_TIM4RST; __DSB();
			RCC->APB1RSTR &= ~RCC_APB1RSTR_TIM4RST;
		}
	if constexpr (peripheral == Peripheral::Tim5)
		if (not Rcc::isEnabled<peripheral>()) {
			RCC->APB1ENR |= RCC_APB1ENR_TIM5EN; __DSB();
			RCC->APB1RSTR |= RCC_APB1RSTR_TIM5RST; __DSB();
			RCC->APB1RSTR &= ~RCC_APB1RSTR_TIM5RST;
		}
	if constexpr (peripheral == Peripheral::Tim6)
		if (not Rcc::isEnabled<peripheral>()) {
			RCC->APB1ENR |= RCC_APB1ENR_TIM6EN; __DSB();
			RCC->APB1RSTR |= RCC_APB1RSTR_TIM6RST; __DSB();
			RCC->APB1RSTR &= ~RCC_APB1RSTR_TIM6RST;
		}
	if constexpr (peripheral == Peripheral::Tim7)
		if (not Rcc::isEnabled<peripheral>()) {
			RCC->APB1ENR |= RCC_APB1ENR_TIM7EN; __DSB();
			RCC->APB1RSTR |= RCC_APB1RSTR_TIM7RST; __DSB();
			RCC->APB1RSTR &= ~RCC_APB1RSTR_TIM7RST;
		}
	if constexpr (peripheral == Peripheral::Tim8)
		if (not Rcc::isEnabled<peripheral>()) {
			RCC->APB2ENR |= RCC_APB2ENR_TIM8EN; __DSB();
			RCC->APB2RSTR |= RCC_APB2RSTR_TIM8RST; __DSB();
			RCC->APB2RSTR &= ~RCC_APB2RSTR_TIM8RST;
		}
	if constexpr (peripheral == Peripheral::Tim9)
		if (not Rcc::isEnabled<peripheral>()) {
			RCC->APB2ENR |= RCC_APB2ENR_TIM9EN; __DSB();
			RCC->APB2RSTR |= RCC_APB2RSTR_TIM9RST; __DSB();
			RCC->APB2RSTR &= ~RCC_APB2RSTR_TIM9RST;
		}
	if constexpr (peripheral == Peripheral::Uart4)
		if (not Rcc::isEnabled<peripheral>()) {
			RCC->APB1ENR |= RCC_APB1ENR_UART4EN; __DSB();
			RCC->APB1RSTR |= RCC_APB1RSTR_UART4RST; __DSB();
			RCC->APB1RSTR &= ~RCC_APB1RSTR_UART4RST;
		}
	if constexpr (peripheral == Peripheral::Uart5)
		if (not Rcc::isEnabled<peripheral>()) {
			RCC->APB1ENR |= RCC_APB1ENR_UART5EN; __DSB();
			RCC->APB1RSTR |= RCC_APB1RSTR_UART5RST; __DSB();
			RCC->APB1RSTR &= ~RCC_APB1RSTR_UART5RST;
		}
	if constexpr (peripheral == Peripheral::Uart7)
		if (not Rcc::isEnabled<peripheral>()) {
			RCC->APB1ENR |= RCC_APB1ENR_UART7EN; __DSB();
			RCC->APB1RSTR |= RCC_APB1RSTR_UART7RST; __DSB();
			RCC->APB1RSTR &= ~RCC_APB1RSTR_UART7RST;
		}
	if constexpr (peripheral == Peripheral::Uart8)
		if (not Rcc::isEnabled<peripheral>()) {
			RCC->APB1ENR |= RCC_APB1ENR_UART8EN; __DSB();
			RCC->APB1RSTR |= RCC_APB1RSTR_UART8RST; __DSB();
			RCC->APB1RSTR &= ~RCC_APB1RSTR_UART8RST;
		}
	if constexpr (peripheral == Peripheral::Usart1)
		if (not Rcc::isEnabled<peripheral>()) {
			RCC->APB2ENR |= RCC_APB2ENR_USART1EN; __DSB();
			RCC->APB2RSTR |= RCC_APB2RSTR_USART1RST; __DSB();
			RCC->APB2RSTR &= ~RCC_APB2RSTR_USART1RST;
		}
	if constexpr (peripheral == Peripheral::Usart2)
		if (not Rcc::isEnabled<peripheral>()) {
			RCC->APB1ENR |= RCC_APB1ENR_USART2EN; __DSB();
			RCC->APB1RSTR |= RCC_APB1RSTR_USART2RST; __DSB();
			RCC->APB1RSTR &= ~RCC_APB1RSTR_USART2RST;
		}
	if constexpr (peripheral == Peripheral::Usart3)
		if (not Rcc::isEnabled<peripheral>()) {
			RCC->APB1ENR |= RCC_APB1ENR_USART3EN; __DSB();
			RCC->APB1RSTR |= RCC_APB1RSTR_USART3RST; __DSB();
			RCC->APB1RSTR &= ~RCC_APB1RSTR_USART3RST;
		}
	if constexpr (peripheral == Peripheral::Usart6)
		if (not Rcc::isEnabled<peripheral>()) {
			RCC->APB2ENR |= RCC_APB2ENR_USART6EN; __DSB();
			RCC->APB2RSTR |= RCC_APB2RSTR_USART6RST; __DSB();
			RCC->APB2RSTR &= ~RCC_APB2RSTR_USART6RST;
		}
	if constexpr (peripheral == Peripheral::Usbotgfs)
		if (not Rcc::isEnabled<peripheral>()) {
			RCC->AHB2ENR |= RCC_AHB2ENR_OTGFSEN; __DSB();
			RCC->AHB2RSTR |= RCC_AHB2RSTR_OTGFSRST; __DSB();
			RCC->AHB2RSTR &= ~RCC_AHB2RSTR_OTGFSRST;
		}
	if constexpr (peripheral == Peripheral::Usbotghs)
		if (not Rcc::isEnabled<peripheral>()) {
			RCC->AHB1ENR |= RCC_AHB1ENR_OTGHSEN; __DSB();
			RCC->AHB1ENR |= RCC_AHB1ENR_OTGHSULPIEN;
		}
	if constexpr (peripheral == Peripheral::Wwdg)
		if (not Rcc::isEnabled<peripheral>()) {
			RCC->APB1ENR |= RCC_APB1ENR_WWDGEN; __DSB();
			RCC->APB1RSTR |= RCC_APB1RSTR_WWDGRST; __DSB();
			RCC->APB1RSTR &= ~RCC_APB1RSTR_WWDGRST;
		}
	__DSB();
}

template< Peripheral peripheral >
void
Rcc::disable()
{
	static_assert(rcc_check_enable(peripheral),
		"Rcc::disable() doesn't know this peripheral!");

	__DSB();
	if constexpr (peripheral == Peripheral::Adc1)
		RCC->APB2ENR &= ~RCC_APB2ENR_ADC1EN;
	if constexpr (peripheral == Peripheral::Adc2)
		RCC->APB2ENR &= ~RCC_APB2ENR_ADC2EN;
	if constexpr (peripheral == Peripheral::Adc3)
		RCC->APB2ENR &= ~RCC_APB2ENR_ADC3EN;
	if constexpr (peripheral == Peripheral::Can1)
		RCC->APB1ENR &= ~RCC_APB1ENR_CAN1EN;
	if constexpr (peripheral == Peripheral::Can2)
		RCC->APB1ENR &= ~RCC_APB1ENR_CAN2EN;
	if constexpr (peripheral == Peripheral::Crc)
		RCC->AHB1ENR &= ~RCC_AHB1ENR_CRCEN;
	if constexpr (peripheral == Peripheral::Dac)
		RCC->APB1ENR &= ~RCC_APB1ENR_DACEN;
	if constexpr (peripheral == Peripheral::Dcmi)
		RCC->AHB2ENR &= ~RCC_AHB2ENR_DCMIEN;
	if constexpr (peripheral == Peripheral::Dma1)
		RCC->AHB1ENR &= ~RCC_AHB1ENR_DMA1EN;
	if constexpr (peripheral == Peripheral::Dma2)
		RCC->AHB1ENR &= ~RCC_AHB1ENR_DMA2EN;
	if constexpr (peripheral == Peripheral::Dma2d)
		RCC->AHB1ENR &= ~RCC_AHB1ENR_DMA2DEN;
	if constexpr (peripheral == Peripheral::Eth)
		RCC->AHB1ENR &= ~RCC_AHB1ENR_ETHMACEN; __DSB();
		RCC->AHB1ENR &= ~RCC_AHB1ENR_ETHMACRXEN; __DSB();
		RCC->AHB1ENR &= ~RCC_AHB1ENR_ETHMACTXEN;
	if constexpr (peripheral == Peripheral::Fmc)
		RCC->AHB3ENR &= ~RCC_AHB3ENR_FMCEN;
	if constexpr (peripheral == Peripheral::I2c1)
		RCC->APB1ENR &= ~RCC_APB1ENR_I2C1EN;
	if constexpr (peripheral == Peripheral::I2c2)
		RCC->APB1ENR &= ~RCC_APB1ENR_I2C2EN;
	if constexpr (peripheral == Peripheral::I2c3)
		RCC->APB1ENR &= ~RCC_APB1ENR_I2C3EN;
	if constexpr (peripheral == Peripheral::Rng)
		RCC->AHB2ENR &= ~RCC_AHB2ENR_RNGEN;
	if constexpr (peripheral == Peripheral::Rtc)
		RCC->BDCR &= ~RCC_BDCR_RTCEN;
	if constexpr (peripheral == Peripheral::Sai1)
		RCC->APB2ENR &= ~RCC_APB2ENR_SAI1EN;
	if constexpr (peripheral == Peripheral::Sdio)
		RCC->APB2ENR &= ~RCC_APB2ENR_SDIOEN;
	if constexpr (peripheral == Peripheral::Spi1)
		RCC->APB2ENR &= ~RCC_APB2ENR_SPI1EN;
	if constexpr (peripheral == Peripheral::Spi2)
		RCC->APB1ENR &= ~RCC_APB1ENR_SPI2EN;
	if constexpr (peripheral == Peripheral::Spi3)
		RCC->APB1ENR &= ~RCC_APB1ENR_SPI3EN;
	if constexpr (peripheral == Peripheral::Spi4)
		RCC->APB2ENR &= ~RCC_APB2ENR_SPI4EN;
	if constexpr (peripheral == Peripheral::Spi5)
		RCC->APB2ENR &= ~RCC_APB2ENR_SPI5EN;
	if constexpr (peripheral == Peripheral::Spi6)
		RCC->APB2ENR &= ~RCC_APB2ENR_SPI6EN;
	if constexpr (peripheral == Peripheral::Tim1)
		RCC->APB2ENR &= ~RCC_APB2ENR_TIM1EN;
	if constexpr (peripheral == Peripheral::Tim10)
		RCC->APB2ENR &= ~RCC_APB2ENR_TIM10EN;
	if constexpr (peripheral == Peripheral::Tim11)
		RCC->APB2ENR &= ~RCC_APB2ENR_TIM11EN;
	if constexpr (peripheral == Peripheral::Tim12)
		RCC->APB1ENR &= ~RCC_APB1ENR_TIM12EN;
	if constexpr (peripheral == Peripheral::Tim13)
		RCC->APB1ENR &= ~RCC_APB1ENR_TIM13EN;
	if constexpr (peripheral == Peripheral::Tim14)
		RCC->APB1ENR &= ~RCC_APB1ENR_TIM14EN;
	if constexpr (peripheral == Peripheral::Tim2)
		RCC->APB1ENR &= ~RCC_APB1ENR_TIM2EN;
	if constexpr (peripheral == Peripheral::Tim3)
		RCC->APB1ENR &= ~RCC_APB1ENR_TIM3EN;
	if constexpr (peripheral == Peripheral::Tim4)
		RCC->APB1ENR &= ~RCC_APB1ENR_TIM4EN;
	if constexpr (peripheral == Peripheral::Tim5)
		RCC->APB1ENR &= ~RCC_APB1ENR_TIM5EN;
	if constexpr (peripheral == Peripheral::Tim6)
		RCC->APB1ENR &= ~RCC_APB1ENR_TIM6EN;
	if constexpr (peripheral == Peripheral::Tim7)
		RCC->APB1ENR &= ~RCC_APB1ENR_TIM7EN;
	if constexpr (peripheral == Peripheral::Tim8)
		RCC->APB2ENR &= ~RCC_APB2ENR_TIM8EN;
	if constexpr (peripheral == Peripheral::Tim9)
		RCC->APB2ENR &= ~RCC_APB2ENR_TIM9EN;
	if constexpr (peripheral == Peripheral::Uart4)
		RCC->APB1ENR &= ~RCC_APB1ENR_UART4EN;
	if constexpr (peripheral == Peripheral::Uart5)
		RCC->APB1ENR &= ~RCC_APB1ENR_UART5EN;
	if constexpr (peripheral == Peripheral::Uart7)
		RCC->APB1ENR &= ~RCC_APB1ENR_UART7EN;
	if constexpr (peripheral == Peripheral::Uart8)
		RCC->APB1ENR &= ~RCC_APB1ENR_UART8EN;
	if constexpr (peripheral == Peripheral::Usart1)
		RCC->APB2ENR &= ~RCC_APB2ENR_USART1EN;
	if constexpr (peripheral == Peripheral::Usart2)
		RCC->APB1ENR &= ~RCC_APB1ENR_USART2EN;
	if constexpr (peripheral == Peripheral::Usart3)
		RCC->APB1ENR &= ~RCC_APB1ENR_USART3EN;
	if constexpr (peripheral == Peripheral::Usart6)
		RCC->APB2ENR &= ~RCC_APB2ENR_USART6EN;
	if constexpr (peripheral == Peripheral::Usbotgfs)
		RCC->AHB2ENR &= ~RCC_AHB2ENR_OTGFSEN;
	if constexpr (peripheral == Peripheral::Usbotghs)
		RCC->AHB1ENR &= ~RCC_AHB1ENR_OTGHSEN; __DSB();
		RCC->AHB1ENR &= ~RCC_AHB1ENR_OTGHSULPIEN;
	if constexpr (peripheral == Peripheral::Wwdg)
		RCC->APB1ENR &= ~RCC_APB1ENR_WWDGEN;
	__DSB();
}

template< Peripheral peripheral >
bool
Rcc::isEnabled()
{
	static_assert(rcc_check_enable(peripheral),
		"Rcc::isEnabled() doesn't know this peripheral!");

	if constexpr (peripheral == Peripheral::Adc1)
		return RCC->APB2ENR & RCC_APB2ENR_ADC1EN;
	if constexpr (peripheral == Peripheral::Adc2)
		return RCC->APB2ENR & RCC_APB2ENR_ADC2EN;
	if constexpr (peripheral == Peripheral::Adc3)
		return RCC->APB2ENR & RCC_APB2ENR_ADC3EN;
	if constexpr (peripheral == Peripheral::Can1)
		return RCC->APB1ENR & RCC_APB1ENR_CAN1EN;
	if constexpr (peripheral == Peripheral::Can2)
		return RCC->APB1ENR & RCC_APB1ENR_CAN2EN;
	if constexpr (peripheral == Peripheral::Crc)
		return RCC->AHB1ENR & RCC_AHB1ENR_CRCEN;
	if constexpr (peripheral == Peripheral::Dac)
		return RCC->APB1ENR & RCC_APB1ENR_DACEN;
	if constexpr (peripheral == Peripheral::Dcmi)
		return RCC->AHB2ENR & RCC_AHB2ENR_DCMIEN;
	if constexpr (peripheral == Peripheral::Dma1)
		return RCC->AHB1ENR & RCC_AHB1ENR_DMA1EN;
	if constexpr (peripheral == Peripheral::Dma2)
		return RCC->AHB1ENR & RCC_AHB1ENR_DMA2EN;
	if constexpr (peripheral == Peripheral::Dma2d)
		return RCC->AHB1ENR & RCC_AHB1ENR_DMA2DEN;
	if constexpr (peripheral == Peripheral::Eth)
		return RCC->AHB1ENR & RCC_AHB1ENR_ETHMACEN;
	if constexpr (peripheral == Peripheral::Fmc)
		return RCC->AHB3ENR & RCC_AHB3ENR_FMCEN;
	if constexpr (peripheral == Peripheral::I2c1)
		return RCC->APB1ENR & RCC_APB1ENR_I2C1EN;
	if constexpr (peripheral == Peripheral::I2c2)
		return RCC->APB1ENR & RCC_APB1ENR_I2C2EN;
	if constexpr (peripheral == Peripheral::I2c3)
		return RCC->APB1ENR & RCC_APB1ENR_I2C3EN;
	if constexpr (peripheral == Peripheral::Rng)
		return RCC->AHB2ENR & RCC_AHB2ENR_RNGEN;
	if constexpr (peripheral == Peripheral::Rtc)
		return RCC->BDCR & RCC_BDCR_RTCEN;
	if constexpr (peripheral == Peripheral::Sai1)
		return RCC->APB2ENR & RCC_APB2ENR_SAI1EN;
	if constexpr (peripheral == Peripheral::Sdio)
		return RCC->APB2ENR & RCC_APB2ENR_SDIOEN;
	if constexpr (peripheral == Peripheral::Spi1)
		return RCC->APB2ENR & RCC_APB2ENR_SPI1EN;
	if constexpr (peripheral == Peripheral::Spi2)
		return RCC->APB1ENR & RCC_APB1ENR_SPI2EN;
	if constexpr (peripheral == Peripheral::Spi3)
		return RCC->APB1ENR & RCC_APB1ENR_SPI3EN;
	if constexpr (peripheral == Peripheral::Spi4)
		return RCC->APB2ENR & RCC_APB2ENR_SPI4EN;
	if constexpr (peripheral == Peripheral::Spi5)
		return RCC->APB2ENR & RCC_APB2ENR_SPI5EN;
	if constexpr (peripheral == Peripheral::Spi6)
		return RCC->APB2ENR & RCC_APB2ENR_SPI6EN;
	if constexpr (peripheral == Peripheral::Tim1)
		return RCC->APB2ENR & RCC_APB2ENR_TIM1EN;
	if constexpr (peripheral == Peripheral::Tim10)
		return RCC->APB2ENR & RCC_APB2ENR_TIM10EN;
	if constexpr (peripheral == Peripheral::Tim11)
		return RCC->APB2ENR & RCC_APB2ENR_TIM11EN;
	if constexpr (peripheral == Peripheral::Tim12)
		return RCC->APB1ENR & RCC_APB1ENR_TIM12EN;
	if constexpr (peripheral == Peripheral::Tim13)
		return RCC->APB1ENR & RCC_APB1ENR_TIM13EN;
	if constexpr (peripheral == Peripheral::Tim14)
		return RCC->APB1ENR & RCC_APB1ENR_TIM14EN;
	if constexpr (peripheral == Peripheral::Tim2)
		return RCC->APB1ENR & RCC_APB1ENR_TIM2EN;
	if constexpr (peripheral == Peripheral::Tim3)
		return RCC->APB1ENR & RCC_APB1ENR_TIM3EN;
	if constexpr (peripheral == Peripheral::Tim4)
		return RCC->APB1ENR & RCC_APB1ENR_TIM4EN;
	if constexpr (peripheral == Peripheral::Tim5)
		return RCC->APB1ENR & RCC_APB1ENR_TIM5EN;
	if constexpr (peripheral == Peripheral::Tim6)
		return RCC->APB1ENR & RCC_APB1ENR_TIM6EN;
	if constexpr (peripheral == Peripheral::Tim7)
		return RCC->APB1ENR & RCC_APB1ENR_TIM7EN;
	if constexpr (peripheral == Peripheral::Tim8)
		return RCC->APB2ENR & RCC_APB2ENR_TIM8EN;
	if constexpr (peripheral == Peripheral::Tim9)
		return RCC->APB2ENR & RCC_APB2ENR_TIM9EN;
	if constexpr (peripheral == Peripheral::Uart4)
		return RCC->APB1ENR & RCC_APB1ENR_UART4EN;
	if constexpr (peripheral == Peripheral::Uart5)
		return RCC->APB1ENR & RCC_APB1ENR_UART5EN;
	if constexpr (peripheral == Peripheral::Uart7)
		return RCC->APB1ENR & RCC_APB1ENR_UART7EN;
	if constexpr (peripheral == Peripheral::Uart8)
		return RCC->APB1ENR & RCC_APB1ENR_UART8EN;
	if constexpr (peripheral == Peripheral::Usart1)
		return RCC->APB2ENR & RCC_APB2ENR_USART1EN;
	if constexpr (peripheral == Peripheral::Usart2)
		return RCC->APB1ENR & RCC_APB1ENR_USART2EN;
	if constexpr (peripheral == Peripheral::Usart3)
		return RCC->APB1ENR & RCC_APB1ENR_USART3EN;
	if constexpr (peripheral == Peripheral::Usart6)
		return RCC->APB2ENR & RCC_APB2ENR_USART6EN;
	if constexpr (peripheral == Peripheral::Usbotgfs)
		return RCC->AHB2ENR & RCC_AHB2ENR_OTGFSEN;
	if constexpr (peripheral == Peripheral::Usbotghs)
		return RCC->AHB1ENR & RCC_AHB1ENR_OTGHSEN;
	if constexpr (peripheral == Peripheral::Wwdg)
		return RCC->APB1ENR & RCC_APB1ENR_WWDGEN;
}

}   // namespace modm::platform