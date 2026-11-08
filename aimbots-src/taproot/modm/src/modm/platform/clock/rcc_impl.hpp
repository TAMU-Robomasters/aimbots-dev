/*
 * Copyright (c) 2019, 2021, 2024, Niklas Hauser
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
	constexpr uint32_t flash_latency_950[] =
	{
		35000000,
		70000000,
		85000000,
	};
	constexpr uint32_t flash_latency_1050[] =
	{
		50000000,
		100000000,
		150000000,
	};
	constexpr uint32_t flash_latency_1150[] =
	{
		67000000,
		133000000,
		200000000,
	};
	constexpr uint32_t flash_latency_1260[] =
	{
		70000000,
		140000000,
		210000000,
		275000000,
	};
	const uint32_t *lut(flash_latency_950);
	uint8_t lut_size(sizeof(flash_latency_950) / sizeof(uint32_t));
	// find the right table for the voltage
	if (1260 <= Core_mV) {
		lut = flash_latency_1260;
		lut_size = sizeof(flash_latency_1260) / sizeof(uint32_t);
	}
	else if (1150 <= Core_mV) {
		lut = flash_latency_1150;
		lut_size = sizeof(flash_latency_1150) / sizeof(uint32_t);
	}
	else if (1050 <= Core_mV) {
		lut = flash_latency_1050;
		lut_size = sizeof(flash_latency_1050) / sizeof(uint32_t);
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

template< uint32_t Core_Hz, uint16_t Core_mV>
uint32_t
Rcc::setFlashLatency()
{
	constexpr flash_latency fl = computeFlashLatency(Core_Hz, Core_mV);
	static_assert(Core_Hz <= fl.max_frequency, "CPU Frequency is too high for this core voltage!");

	uint32_t acr = FLASH->ACR & ~FLASH_ACR_LATENCY;
	// set flash latency
	acr |= fl.latency;
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
		case Peripheral::Adc3:
		case Peripheral::Bdma:
		case Peripheral::Comp1:
		case Peripheral::Cordic:
		case Peripheral::Crc:
		case Peripheral::Dac1:
		case Peripheral::Dcmi:
		case Peripheral::Dfsdm1:
		case Peripheral::Dma1:
		case Peripheral::Dma2:
		case Peripheral::Dma2d:
		case Peripheral::Dts:
		case Peripheral::Fdcan1:
		case Peripheral::Fmac:
		case Peripheral::Fmc:
		case Peripheral::Hdmicec:
		case Peripheral::I2c1:
		case Peripheral::I2c2:
		case Peripheral::I2c3:
		case Peripheral::I2c4:
		case Peripheral::I2c5:
		case Peripheral::Lptim1:
		case Peripheral::Lptim2:
		case Peripheral::Lptim3:
		case Peripheral::Lptim4:
		case Peripheral::Lptim5:
		case Peripheral::Lpuart1:
		case Peripheral::Ltdc:
		case Peripheral::Mdios:
		case Peripheral::Mdma:
		case Peripheral::Rng:
		case Peripheral::Rtc:
		case Peripheral::Sai1:
		case Peripheral::Sai4:
		case Peripheral::Sdmmc1:
		case Peripheral::Sdmmc2:
		case Peripheral::Spdifrx:
		case Peripheral::Spi1:
		case Peripheral::Spi2:
		case Peripheral::Spi3:
		case Peripheral::Spi4:
		case Peripheral::Spi6:
		case Peripheral::Tim1:
		case Peripheral::Tim12:
		case Peripheral::Tim13:
		case Peripheral::Tim14:
		case Peripheral::Tim15:
		case Peripheral::Tim16:
		case Peripheral::Tim17:
		case Peripheral::Tim2:
		case Peripheral::Tim23:
		case Peripheral::Tim24:
		case Peripheral::Tim3:
		case Peripheral::Tim4:
		case Peripheral::Tim5:
		case Peripheral::Tim6:
		case Peripheral::Tim7:
		case Peripheral::Tim8:
		case Peripheral::Uart4:
		case Peripheral::Uart5:
		case Peripheral::Uart7:
		case Peripheral::Uart8:
		case Peripheral::Uart9:
		case Peripheral::Usart1:
		case Peripheral::Usart10:
		case Peripheral::Usart2:
		case Peripheral::Usart3:
		case Peripheral::Usart6:
		case Peripheral::Usbotghs:
		case Peripheral::Usbotghsulpi:
		case Peripheral::Wwdg1:
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
	if constexpr (peripheral == Peripheral::Adc3)
		if (not Rcc::isEnabled<peripheral>()) {
			RCC->AHB4ENR |= RCC_AHB4ENR_ADC3EN; __DSB();
			RCC->AHB4RSTR |= RCC_AHB4RSTR_ADC3RST; __DSB();
			RCC->AHB4RSTR &= ~RCC_AHB4RSTR_ADC3RST;
		}
	if constexpr (peripheral == Peripheral::Bdma)
		if (not Rcc::isEnabled<peripheral>()) {
			RCC->AHB4ENR |= RCC_AHB4ENR_BDMAEN; __DSB();
			RCC->AHB4RSTR |= RCC_AHB4RSTR_BDMARST; __DSB();
			RCC->AHB4RSTR &= ~RCC_AHB4RSTR_BDMARST;
		}
	if constexpr (peripheral == Peripheral::Comp1)
		if (not Rcc::isEnabled<peripheral>()) {
			RCC->APB4ENR |= RCC_APB4ENR_COMP12EN; __DSB();
			RCC->APB4RSTR |= RCC_APB4RSTR_COMP12RST; __DSB();
			RCC->APB4RSTR &= ~RCC_APB4RSTR_COMP12RST;
		}
	if constexpr (peripheral == Peripheral::Cordic)
		if (not Rcc::isEnabled<peripheral>()) {
			RCC->AHB2ENR |= RCC_AHB2ENR_CORDICEN; __DSB();
			RCC->AHB2RSTR |= RCC_AHB2RSTR_CORDICRST; __DSB();
			RCC->AHB2RSTR &= ~RCC_AHB2RSTR_CORDICRST;
		}
	if constexpr (peripheral == Peripheral::Crc)
		if (not Rcc::isEnabled<peripheral>()) {
			RCC->AHB4ENR |= RCC_AHB4ENR_CRCEN; __DSB();
			RCC->AHB4RSTR |= RCC_AHB4RSTR_CRCRST; __DSB();
			RCC->AHB4RSTR &= ~RCC_AHB4RSTR_CRCRST;
		}
	if constexpr (peripheral == Peripheral::Dac1)
		if (not Rcc::isEnabled<peripheral>()) {
			RCC->APB1LENR |= RCC_APB1LENR_DAC12EN; __DSB();
			RCC->APB1LRSTR |= RCC_APB1LRSTR_DAC12RST; __DSB();
			RCC->APB1LRSTR &= ~RCC_APB1LRSTR_DAC12RST;
		}
	if constexpr (peripheral == Peripheral::Dcmi)
		if (not Rcc::isEnabled<peripheral>()) {
			RCC->AHB2ENR |= RCC_AHB2ENR_DCMIEN; __DSB();
			RCC->AHB2RSTR |= RCC_AHB2RSTR_DCMIRST; __DSB();
			RCC->AHB2RSTR &= ~RCC_AHB2RSTR_DCMIRST;
		}
	if constexpr (peripheral == Peripheral::Dfsdm1)
		if (not Rcc::isEnabled<peripheral>()) {
			RCC->APB2ENR |= RCC_APB2ENR_DFSDM1EN; __DSB();
			RCC->APB2RSTR |= RCC_APB2RSTR_DFSDM1RST; __DSB();
			RCC->APB2RSTR &= ~RCC_APB2RSTR_DFSDM1RST;
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
			RCC->AHB3ENR |= RCC_AHB3ENR_DMA2DEN; __DSB();
			RCC->AHB3RSTR |= RCC_AHB3RSTR_DMA2DRST; __DSB();
			RCC->AHB3RSTR &= ~RCC_AHB3RSTR_DMA2DRST;
		}
	if constexpr (peripheral == Peripheral::Dts)
		if (not Rcc::isEnabled<peripheral>()) {
			RCC->APB4ENR |= RCC_APB4ENR_DTSEN; __DSB();
			RCC->APB4RSTR |= RCC_APB4RSTR_DTSRST; __DSB();
			RCC->APB4RSTR &= ~RCC_APB4RSTR_DTSRST;
		}
	if constexpr (peripheral == Peripheral::Fdcan1)
		if (not Rcc::isEnabled<peripheral>()) {
			RCC->APB1HENR |= RCC_APB1HENR_FDCANEN; __DSB();
			RCC->APB1HRSTR |= RCC_APB1HRSTR_FDCANRST; __DSB();
			RCC->APB1HRSTR &= ~RCC_APB1HRSTR_FDCANRST;
		}
	if constexpr (peripheral == Peripheral::Fmac)
		if (not Rcc::isEnabled<peripheral>()) {
			RCC->AHB2ENR |= RCC_AHB2ENR_FMACEN; __DSB();
			RCC->AHB2RSTR |= RCC_AHB2RSTR_FMACRST; __DSB();
			RCC->AHB2RSTR &= ~RCC_AHB2RSTR_FMACRST;
		}
	if constexpr (peripheral == Peripheral::Fmc)
		if (not Rcc::isEnabled<peripheral>()) {
			RCC->AHB3ENR |= RCC_AHB3ENR_FMCEN; __DSB();
			RCC->AHB3RSTR |= RCC_AHB3RSTR_FMCRST; __DSB();
			RCC->AHB3RSTR &= ~RCC_AHB3RSTR_FMCRST;
		}
	if constexpr (peripheral == Peripheral::Hdmicec)
		if (not Rcc::isEnabled<peripheral>()) {
			RCC->APB1LLPENR |= RCC_APB1LLPENR_HDMICECEN; __DSB();
			RCC->APB1LRSTR |= RCC_APB1LRSTR_HDMICECRST; __DSB();
			RCC->APB1LRSTR &= ~RCC_APB1LRSTR_HDMICECRST;
		}
	if constexpr (peripheral == Peripheral::I2c1)
		if (not Rcc::isEnabled<peripheral>()) {
			RCC->APB1LENR |= RCC_APB1LENR_I2C1EN; __DSB();
			RCC->APB1LRSTR |= RCC_APB1LRSTR_I2C1RST; __DSB();
			RCC->APB1LRSTR &= ~RCC_APB1LRSTR_I2C1RST;
		}
	if constexpr (peripheral == Peripheral::I2c2)
		if (not Rcc::isEnabled<peripheral>()) {
			RCC->APB1LENR |= RCC_APB1LENR_I2C2EN; __DSB();
			RCC->APB1LRSTR |= RCC_APB1LRSTR_I2C2RST; __DSB();
			RCC->APB1LRSTR &= ~RCC_APB1LRSTR_I2C2RST;
		}
	if constexpr (peripheral == Peripheral::I2c3)
		if (not Rcc::isEnabled<peripheral>()) {
			RCC->APB1LENR |= RCC_APB1LENR_I2C3EN; __DSB();
			RCC->APB1LRSTR |= RCC_APB1LRSTR_I2C3RST; __DSB();
			RCC->APB1LRSTR &= ~RCC_APB1LRSTR_I2C3RST;
		}
	if constexpr (peripheral == Peripheral::I2c4)
		if (not Rcc::isEnabled<peripheral>()) {
			RCC->APB4ENR |= RCC_APB4ENR_I2C4EN; __DSB();
			RCC->APB4RSTR |= RCC_APB4RSTR_I2C4RST; __DSB();
			RCC->APB4RSTR &= ~RCC_APB4RSTR_I2C4RST;
		}
	if constexpr (peripheral == Peripheral::I2c5)
		if (not Rcc::isEnabled<peripheral>()) {
			RCC->APB1LENR |= RCC_APB1LENR_I2C5EN; __DSB();
			RCC->APB1LRSTR |= RCC_APB1LRSTR_I2C5RST; __DSB();
			RCC->APB1LRSTR &= ~RCC_APB1LRSTR_I2C5RST;
		}
	if constexpr (peripheral == Peripheral::Lptim1)
		if (not Rcc::isEnabled<peripheral>()) {
			RCC->APB1LENR |= RCC_APB1LENR_LPTIM1EN; __DSB();
			RCC->APB1LRSTR |= RCC_APB1LRSTR_LPTIM1RST; __DSB();
			RCC->APB1LRSTR &= ~RCC_APB1LRSTR_LPTIM1RST;
		}
	if constexpr (peripheral == Peripheral::Lptim2)
		if (not Rcc::isEnabled<peripheral>()) {
			RCC->APB4ENR |= RCC_APB4ENR_LPTIM2EN; __DSB();
			RCC->APB4RSTR |= RCC_APB4RSTR_LPTIM2RST; __DSB();
			RCC->APB4RSTR &= ~RCC_APB4RSTR_LPTIM2RST;
		}
	if constexpr (peripheral == Peripheral::Lptim3)
		if (not Rcc::isEnabled<peripheral>()) {
			RCC->APB4ENR |= RCC_APB4ENR_LPTIM3EN; __DSB();
			RCC->APB4RSTR |= RCC_APB4RSTR_LPTIM3RST; __DSB();
			RCC->APB4RSTR &= ~RCC_APB4RSTR_LPTIM3RST;
		}
	if constexpr (peripheral == Peripheral::Lptim4)
		if (not Rcc::isEnabled<peripheral>()) {
			RCC->APB4ENR |= RCC_APB4ENR_LPTIM4EN; __DSB();
			RCC->APB4RSTR |= RCC_APB4RSTR_LPTIM4RST; __DSB();
			RCC->APB4RSTR &= ~RCC_APB4RSTR_LPTIM4RST;
		}
	if constexpr (peripheral == Peripheral::Lptim5)
		if (not Rcc::isEnabled<peripheral>()) {
			RCC->APB4ENR |= RCC_APB4ENR_LPTIM5EN; __DSB();
			RCC->APB4RSTR |= RCC_APB4RSTR_LPTIM5RST; __DSB();
			RCC->APB4RSTR &= ~RCC_APB4RSTR_LPTIM5RST;
		}
	if constexpr (peripheral == Peripheral::Lpuart1)
		if (not Rcc::isEnabled<peripheral>()) {
			RCC->APB4ENR |= RCC_APB4ENR_LPUART1EN; __DSB();
			RCC->APB4RSTR |= RCC_APB4RSTR_LPUART1RST; __DSB();
			RCC->APB4RSTR &= ~RCC_APB4RSTR_LPUART1RST;
		}
	if constexpr (peripheral == Peripheral::Ltdc)
		if (not Rcc::isEnabled<peripheral>()) {
			RCC->APB3ENR |= RCC_APB3ENR_LTDCEN; __DSB();
			RCC->APB3RSTR |= RCC_APB3RSTR_LTDCRST; __DSB();
			RCC->APB3RSTR &= ~RCC_APB3RSTR_LTDCRST;
		}
	if constexpr (peripheral == Peripheral::Mdios)
		if (not Rcc::isEnabled<peripheral>()) {
			RCC->APB1HENR |= RCC_APB1HENR_MDIOSEN; __DSB();
			RCC->APB1HRSTR |= RCC_APB1HRSTR_MDIOSRST; __DSB();
			RCC->APB1HRSTR &= ~RCC_APB1HRSTR_MDIOSRST;
		}
	if constexpr (peripheral == Peripheral::Mdma)
		if (not Rcc::isEnabled<peripheral>()) {
			RCC->AHB3ENR |= RCC_AHB3ENR_MDMAEN; __DSB();
			RCC->AHB3RSTR |= RCC_AHB3RSTR_MDMARST; __DSB();
			RCC->AHB3RSTR &= ~RCC_AHB3RSTR_MDMARST;
		}
	if constexpr (peripheral == Peripheral::Rng)
		if (not Rcc::isEnabled<peripheral>()) {
			RCC->AHB2ENR |= RCC_AHB2ENR_RNGEN; __DSB();
			RCC->AHB2RSTR |= RCC_AHB2RSTR_RNGRST; __DSB();
			RCC->AHB2RSTR &= ~RCC_AHB2RSTR_RNGRST;
		}
	if constexpr (peripheral == Peripheral::Rtc)
		{
			RCC->BDCR |= RCC_BDCR_RTCEN;
		}
	if constexpr (peripheral == Peripheral::Sai1)
		if (not Rcc::isEnabled<peripheral>()) {
			RCC->APB2ENR |= RCC_APB2ENR_SAI1EN; __DSB();
			RCC->APB2RSTR |= RCC_APB2RSTR_SAI1RST; __DSB();
			RCC->APB2RSTR &= ~RCC_APB2RSTR_SAI1RST;
		}
	if constexpr (peripheral == Peripheral::Sai4)
		if (not Rcc::isEnabled<peripheral>()) {
			RCC->APB4ENR |= RCC_APB4ENR_SAI4EN; __DSB();
			RCC->APB4RSTR |= RCC_APB4RSTR_SAI4RST; __DSB();
			RCC->APB4RSTR &= ~RCC_APB4RSTR_SAI4RST;
		}
	if constexpr (peripheral == Peripheral::Sdmmc1)
		if (not Rcc::isEnabled<peripheral>()) {
			RCC->AHB3ENR |= RCC_AHB3ENR_SDMMC1EN; __DSB();
			RCC->AHB3RSTR |= RCC_AHB3RSTR_SDMMC1RST; __DSB();
			RCC->AHB3RSTR &= ~RCC_AHB3RSTR_SDMMC1RST;
		}
	if constexpr (peripheral == Peripheral::Sdmmc2)
		if (not Rcc::isEnabled<peripheral>()) {
			RCC->AHB2ENR |= RCC_AHB2ENR_SDMMC2EN; __DSB();
			RCC->AHB2RSTR |= RCC_AHB2RSTR_SDMMC2RST; __DSB();
			RCC->AHB2RSTR &= ~RCC_AHB2RSTR_SDMMC2RST;
		}
	if constexpr (peripheral == Peripheral::Spdifrx)
		if (not Rcc::isEnabled<peripheral>()) {
			RCC->APB1LENR |= RCC_APB1LENR_SPDIFRXEN; __DSB();
			RCC->APB1LRSTR |= RCC_APB1LRSTR_SPDIFRXRST; __DSB();
			RCC->APB1LRSTR &= ~RCC_APB1LRSTR_SPDIFRXRST;
		}
	if constexpr (peripheral == Peripheral::Spi1)
		if (not Rcc::isEnabled<peripheral>()) {
			RCC->APB2ENR |= RCC_APB2ENR_SPI1EN; __DSB();
			RCC->APB2RSTR |= RCC_APB2RSTR_SPI1RST; __DSB();
			RCC->APB2RSTR &= ~RCC_APB2RSTR_SPI1RST;
		}
	if constexpr (peripheral == Peripheral::Spi2)
		if (not Rcc::isEnabled<peripheral>()) {
			RCC->APB1LENR |= RCC_APB1LENR_SPI2EN; __DSB();
			RCC->APB1LRSTR |= RCC_APB1LRSTR_SPI2RST; __DSB();
			RCC->APB1LRSTR &= ~RCC_APB1LRSTR_SPI2RST;
		}
	if constexpr (peripheral == Peripheral::Spi3)
		if (not Rcc::isEnabled<peripheral>()) {
			RCC->APB1LENR |= RCC_APB1LENR_SPI3EN; __DSB();
			RCC->APB1LRSTR |= RCC_APB1LRSTR_SPI3RST; __DSB();
			RCC->APB1LRSTR &= ~RCC_APB1LRSTR_SPI3RST;
		}
	if constexpr (peripheral == Peripheral::Spi4)
		if (not Rcc::isEnabled<peripheral>()) {
			RCC->APB2ENR |= RCC_APB2ENR_SPI4EN; __DSB();
			RCC->APB2RSTR |= RCC_APB2RSTR_SPI4RST; __DSB();
			RCC->APB2RSTR &= ~RCC_APB2RSTR_SPI4RST;
		}
	if constexpr (peripheral == Peripheral::Spi6)
		if (not Rcc::isEnabled<peripheral>()) {
			RCC->APB4ENR |= RCC_APB4ENR_SPI6EN; __DSB();
			RCC->APB4RSTR |= RCC_APB4RSTR_SPI6RST; __DSB();
			RCC->APB4RSTR &= ~RCC_APB4RSTR_SPI6RST;
		}
	if constexpr (peripheral == Peripheral::Tim1)
		if (not Rcc::isEnabled<peripheral>()) {
			RCC->APB2ENR |= RCC_APB2ENR_TIM1EN; __DSB();
			RCC->APB2RSTR |= RCC_APB2RSTR_TIM1RST; __DSB();
			RCC->APB2RSTR &= ~RCC_APB2RSTR_TIM1RST;
		}
	if constexpr (peripheral == Peripheral::Tim12)
		if (not Rcc::isEnabled<peripheral>()) {
			RCC->APB1LENR |= RCC_APB1LENR_TIM12EN; __DSB();
			RCC->APB1LRSTR |= RCC_APB1LRSTR_TIM12RST; __DSB();
			RCC->APB1LRSTR &= ~RCC_APB1LRSTR_TIM12RST;
		}
	if constexpr (peripheral == Peripheral::Tim13)
		if (not Rcc::isEnabled<peripheral>()) {
			RCC->APB1LENR |= RCC_APB1LENR_TIM13EN; __DSB();
			RCC->APB1LRSTR |= RCC_APB1LRSTR_TIM13RST; __DSB();
			RCC->APB1LRSTR &= ~RCC_APB1LRSTR_TIM13RST;
		}
	if constexpr (peripheral == Peripheral::Tim14)
		if (not Rcc::isEnabled<peripheral>()) {
			RCC->APB1LENR |= RCC_APB1LENR_TIM14EN; __DSB();
			RCC->APB1LRSTR |= RCC_APB1LRSTR_TIM14RST; __DSB();
			RCC->APB1LRSTR &= ~RCC_APB1LRSTR_TIM14RST;
		}
	if constexpr (peripheral == Peripheral::Tim15)
		if (not Rcc::isEnabled<peripheral>()) {
			RCC->APB2ENR |= RCC_APB2ENR_TIM15EN; __DSB();
			RCC->APB2RSTR |= RCC_APB2RSTR_TIM15RST; __DSB();
			RCC->APB2RSTR &= ~RCC_APB2RSTR_TIM15RST;
		}
	if constexpr (peripheral == Peripheral::Tim16)
		if (not Rcc::isEnabled<peripheral>()) {
			RCC->APB2ENR |= RCC_APB2ENR_TIM16EN; __DSB();
			RCC->APB2RSTR |= RCC_APB2RSTR_TIM16RST; __DSB();
			RCC->APB2RSTR &= ~RCC_APB2RSTR_TIM16RST;
		}
	if constexpr (peripheral == Peripheral::Tim17)
		if (not Rcc::isEnabled<peripheral>()) {
			RCC->APB2ENR |= RCC_APB2ENR_TIM17EN; __DSB();
			RCC->APB2RSTR |= RCC_APB2RSTR_TIM17RST; __DSB();
			RCC->APB2RSTR &= ~RCC_APB2RSTR_TIM17RST;
		}
	if constexpr (peripheral == Peripheral::Tim2)
		if (not Rcc::isEnabled<peripheral>()) {
			RCC->APB1LENR |= RCC_APB1LENR_TIM2EN; __DSB();
			RCC->APB1LRSTR |= RCC_APB1LRSTR_TIM2RST; __DSB();
			RCC->APB1LRSTR &= ~RCC_APB1LRSTR_TIM2RST;
		}
	if constexpr (peripheral == Peripheral::Tim23)
		if (not Rcc::isEnabled<peripheral>()) {
			RCC->APB1HENR |= RCC_APB1HENR_TIM23EN; __DSB();
			RCC->APB1HRSTR |= RCC_APB1HRSTR_TIM23RST; __DSB();
			RCC->APB1HRSTR &= ~RCC_APB1HRSTR_TIM23RST;
		}
	if constexpr (peripheral == Peripheral::Tim24)
		if (not Rcc::isEnabled<peripheral>()) {
			RCC->APB1HENR |= RCC_APB1HENR_TIM24EN; __DSB();
			RCC->APB1HRSTR |= RCC_APB1HRSTR_TIM24RST; __DSB();
			RCC->APB1HRSTR &= ~RCC_APB1HRSTR_TIM24RST;
		}
	if constexpr (peripheral == Peripheral::Tim3)
		if (not Rcc::isEnabled<peripheral>()) {
			RCC->APB1LENR |= RCC_APB1LENR_TIM3EN; __DSB();
			RCC->APB1LRSTR |= RCC_APB1LRSTR_TIM3RST; __DSB();
			RCC->APB1LRSTR &= ~RCC_APB1LRSTR_TIM3RST;
		}
	if constexpr (peripheral == Peripheral::Tim4)
		if (not Rcc::isEnabled<peripheral>()) {
			RCC->APB1LENR |= RCC_APB1LENR_TIM4EN; __DSB();
			RCC->APB1LRSTR |= RCC_APB1LRSTR_TIM4RST; __DSB();
			RCC->APB1LRSTR &= ~RCC_APB1LRSTR_TIM4RST;
		}
	if constexpr (peripheral == Peripheral::Tim5)
		if (not Rcc::isEnabled<peripheral>()) {
			RCC->APB1LENR |= RCC_APB1LENR_TIM5EN; __DSB();
			RCC->APB1LRSTR |= RCC_APB1LRSTR_TIM5RST; __DSB();
			RCC->APB1LRSTR &= ~RCC_APB1LRSTR_TIM5RST;
		}
	if constexpr (peripheral == Peripheral::Tim6)
		if (not Rcc::isEnabled<peripheral>()) {
			RCC->APB1LENR |= RCC_APB1LENR_TIM6EN; __DSB();
			RCC->APB1LRSTR |= RCC_APB1LRSTR_TIM6RST; __DSB();
			RCC->APB1LRSTR &= ~RCC_APB1LRSTR_TIM6RST;
		}
	if constexpr (peripheral == Peripheral::Tim7)
		if (not Rcc::isEnabled<peripheral>()) {
			RCC->APB1LENR |= RCC_APB1LENR_TIM7EN; __DSB();
			RCC->APB1LRSTR |= RCC_APB1LRSTR_TIM7RST; __DSB();
			RCC->APB1LRSTR &= ~RCC_APB1LRSTR_TIM7RST;
		}
	if constexpr (peripheral == Peripheral::Tim8)
		if (not Rcc::isEnabled<peripheral>()) {
			RCC->APB2ENR |= RCC_APB2ENR_TIM8EN; __DSB();
			RCC->APB2RSTR |= RCC_APB2RSTR_TIM8RST; __DSB();
			RCC->APB2RSTR &= ~RCC_APB2RSTR_TIM8RST;
		}
	if constexpr (peripheral == Peripheral::Uart4)
		if (not Rcc::isEnabled<peripheral>()) {
			RCC->APB1LENR |= RCC_APB1LENR_UART4EN; __DSB();
			RCC->APB1LRSTR |= RCC_APB1LRSTR_UART4RST; __DSB();
			RCC->APB1LRSTR &= ~RCC_APB1LRSTR_UART4RST;
		}
	if constexpr (peripheral == Peripheral::Uart5)
		if (not Rcc::isEnabled<peripheral>()) {
			RCC->APB1LENR |= RCC_APB1LENR_UART5EN; __DSB();
			RCC->APB1LRSTR |= RCC_APB1LRSTR_UART5RST; __DSB();
			RCC->APB1LRSTR &= ~RCC_APB1LRSTR_UART5RST;
		}
	if constexpr (peripheral == Peripheral::Uart7)
		if (not Rcc::isEnabled<peripheral>()) {
			RCC->APB1LENR |= RCC_APB1LENR_UART7EN; __DSB();
			RCC->APB1LRSTR |= RCC_APB1LRSTR_UART7RST; __DSB();
			RCC->APB1LRSTR &= ~RCC_APB1LRSTR_UART7RST;
		}
	if constexpr (peripheral == Peripheral::Uart8)
		if (not Rcc::isEnabled<peripheral>()) {
			RCC->APB1LENR |= RCC_APB1LENR_UART8EN; __DSB();
			RCC->APB1LRSTR |= RCC_APB1LRSTR_UART8RST; __DSB();
			RCC->APB1LRSTR &= ~RCC_APB1LRSTR_UART8RST;
		}
	if constexpr (peripheral == Peripheral::Uart9)
		if (not Rcc::isEnabled<peripheral>()) {
			RCC->APB2ENR |= RCC_APB2ENR_UART9EN; __DSB();
			RCC->APB2RSTR |= RCC_APB2RSTR_UART9RST; __DSB();
			RCC->APB2RSTR &= ~RCC_APB2RSTR_UART9RST;
		}
	if constexpr (peripheral == Peripheral::Usart1)
		if (not Rcc::isEnabled<peripheral>()) {
			RCC->APB2ENR |= RCC_APB2ENR_USART1EN; __DSB();
			RCC->APB2RSTR |= RCC_APB2RSTR_USART1RST; __DSB();
			RCC->APB2RSTR &= ~RCC_APB2RSTR_USART1RST;
		}
	if constexpr (peripheral == Peripheral::Usart10)
		if (not Rcc::isEnabled<peripheral>()) {
			RCC->APB2ENR |= RCC_APB2ENR_USART10EN; __DSB();
			RCC->APB2RSTR |= RCC_APB2RSTR_USART10RST; __DSB();
			RCC->APB2RSTR &= ~RCC_APB2RSTR_USART10RST;
		}
	if constexpr (peripheral == Peripheral::Usart2)
		if (not Rcc::isEnabled<peripheral>()) {
			RCC->APB1LENR |= RCC_APB1LENR_USART2EN; __DSB();
			RCC->APB1LRSTR |= RCC_APB1LRSTR_USART2RST; __DSB();
			RCC->APB1LRSTR &= ~RCC_APB1LRSTR_USART2RST;
		}
	if constexpr (peripheral == Peripheral::Usart3)
		if (not Rcc::isEnabled<peripheral>()) {
			RCC->APB1LENR |= RCC_APB1LENR_USART3EN; __DSB();
			RCC->APB1LRSTR |= RCC_APB1LRSTR_USART3RST; __DSB();
			RCC->APB1LRSTR &= ~RCC_APB1LRSTR_USART3RST;
		}
	if constexpr (peripheral == Peripheral::Usart6)
		if (not Rcc::isEnabled<peripheral>()) {
			RCC->APB2ENR |= RCC_APB2ENR_USART6EN; __DSB();
			RCC->APB2RSTR |= RCC_APB2RSTR_USART6RST; __DSB();
			RCC->APB2RSTR &= ~RCC_APB2RSTR_USART6RST;
		}
	if constexpr (peripheral == Peripheral::Usbotghs)
		if (not Rcc::isEnabled<peripheral>()) {
			RCC->AHB1ENR |= RCC_AHB1ENR_USB1OTGHSEN; __DSB();
			RCC->AHB1RSTR |= RCC_AHB1RSTR_USB1OTGHSRST; __DSB();
			RCC->AHB1RSTR &= ~RCC_AHB1RSTR_USB1OTGHSRST;
		}
	if constexpr (peripheral == Peripheral::Usbotghsulpi)
		{
			RCC->AHB1ENR |= RCC_AHB1ENR_USB1OTGHSULPIEN;
		}
	if constexpr (peripheral == Peripheral::Wwdg1)
		{
			RCC->APB3ENR |= RCC_APB3ENR_WWDG1EN;
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
	if constexpr (peripheral == Peripheral::Adc3) {
		RCC->AHB4ENR &= ~RCC_AHB4ENR_ADC3EN;
	}
	if constexpr (peripheral == Peripheral::Bdma) {
		RCC->AHB4ENR &= ~RCC_AHB4ENR_BDMAEN;
	}
	if constexpr (peripheral == Peripheral::Comp1) {
		RCC->APB4ENR &= ~RCC_APB4ENR_COMP12EN;
	}
	if constexpr (peripheral == Peripheral::Cordic) {
		RCC->AHB2ENR &= ~RCC_AHB2ENR_CORDICEN;
	}
	if constexpr (peripheral == Peripheral::Crc) {
		RCC->AHB4ENR &= ~RCC_AHB4ENR_CRCEN;
	}
	if constexpr (peripheral == Peripheral::Dac1) {
		RCC->APB1LENR &= ~RCC_APB1LENR_DAC12EN;
	}
	if constexpr (peripheral == Peripheral::Dcmi) {
		RCC->AHB2ENR &= ~RCC_AHB2ENR_DCMIEN;
	}
	if constexpr (peripheral == Peripheral::Dfsdm1) {
		RCC->APB2ENR &= ~RCC_APB2ENR_DFSDM1EN;
	}
	if constexpr (peripheral == Peripheral::Dma1) {
		RCC->AHB1ENR &= ~RCC_AHB1ENR_DMA1EN;
	}
	if constexpr (peripheral == Peripheral::Dma2) {
		RCC->AHB1ENR &= ~RCC_AHB1ENR_DMA2EN;
	}
	if constexpr (peripheral == Peripheral::Dma2d) {
		RCC->AHB3ENR &= ~RCC_AHB3ENR_DMA2DEN;
	}
	if constexpr (peripheral == Peripheral::Dts) {
		RCC->APB4ENR &= ~RCC_APB4ENR_DTSEN;
	}
	if constexpr (peripheral == Peripheral::Fdcan1) {
		RCC->APB1HENR &= ~RCC_APB1HENR_FDCANEN;
	}
	if constexpr (peripheral == Peripheral::Fmac) {
		RCC->AHB2ENR &= ~RCC_AHB2ENR_FMACEN;
	}
	if constexpr (peripheral == Peripheral::Fmc) {
		RCC->AHB3ENR &= ~RCC_AHB3ENR_FMCEN;
	}
	if constexpr (peripheral == Peripheral::Hdmicec) {
		RCC->APB1LLPENR &= ~RCC_APB1LLPENR_HDMICECEN;
	}
	if constexpr (peripheral == Peripheral::I2c1) {
		RCC->APB1LENR &= ~RCC_APB1LENR_I2C1EN;
	}
	if constexpr (peripheral == Peripheral::I2c2) {
		RCC->APB1LENR &= ~RCC_APB1LENR_I2C2EN;
	}
	if constexpr (peripheral == Peripheral::I2c3) {
		RCC->APB1LENR &= ~RCC_APB1LENR_I2C3EN;
	}
	if constexpr (peripheral == Peripheral::I2c4) {
		RCC->APB4ENR &= ~RCC_APB4ENR_I2C4EN;
	}
	if constexpr (peripheral == Peripheral::I2c5) {
		RCC->APB1LENR &= ~RCC_APB1LENR_I2C5EN;
	}
	if constexpr (peripheral == Peripheral::Lptim1) {
		RCC->APB1LENR &= ~RCC_APB1LENR_LPTIM1EN;
	}
	if constexpr (peripheral == Peripheral::Lptim2) {
		RCC->APB4ENR &= ~RCC_APB4ENR_LPTIM2EN;
	}
	if constexpr (peripheral == Peripheral::Lptim3) {
		RCC->APB4ENR &= ~RCC_APB4ENR_LPTIM3EN;
	}
	if constexpr (peripheral == Peripheral::Lptim4) {
		RCC->APB4ENR &= ~RCC_APB4ENR_LPTIM4EN;
	}
	if constexpr (peripheral == Peripheral::Lptim5) {
		RCC->APB4ENR &= ~RCC_APB4ENR_LPTIM5EN;
	}
	if constexpr (peripheral == Peripheral::Lpuart1) {
		RCC->APB4ENR &= ~RCC_APB4ENR_LPUART1EN;
	}
	if constexpr (peripheral == Peripheral::Ltdc) {
		RCC->APB3ENR &= ~RCC_APB3ENR_LTDCEN;
	}
	if constexpr (peripheral == Peripheral::Mdios) {
		RCC->APB1HENR &= ~RCC_APB1HENR_MDIOSEN;
	}
	if constexpr (peripheral == Peripheral::Mdma) {
		RCC->AHB3ENR &= ~RCC_AHB3ENR_MDMAEN;
	}
	if constexpr (peripheral == Peripheral::Rng) {
		RCC->AHB2ENR &= ~RCC_AHB2ENR_RNGEN;
	}
	if constexpr (peripheral == Peripheral::Rtc) {
		RCC->BDCR &= ~RCC_BDCR_RTCEN;
	}
	if constexpr (peripheral == Peripheral::Sai1) {
		RCC->APB2ENR &= ~RCC_APB2ENR_SAI1EN;
	}
	if constexpr (peripheral == Peripheral::Sai4) {
		RCC->APB4ENR &= ~RCC_APB4ENR_SAI4EN;
	}
	if constexpr (peripheral == Peripheral::Sdmmc1) {
		RCC->AHB3ENR &= ~RCC_AHB3ENR_SDMMC1EN;
	}
	if constexpr (peripheral == Peripheral::Sdmmc2) {
		RCC->AHB2ENR &= ~RCC_AHB2ENR_SDMMC2EN;
	}
	if constexpr (peripheral == Peripheral::Spdifrx) {
		RCC->APB1LENR &= ~RCC_APB1LENR_SPDIFRXEN;
	}
	if constexpr (peripheral == Peripheral::Spi1) {
		RCC->APB2ENR &= ~RCC_APB2ENR_SPI1EN;
	}
	if constexpr (peripheral == Peripheral::Spi2) {
		RCC->APB1LENR &= ~RCC_APB1LENR_SPI2EN;
	}
	if constexpr (peripheral == Peripheral::Spi3) {
		RCC->APB1LENR &= ~RCC_APB1LENR_SPI3EN;
	}
	if constexpr (peripheral == Peripheral::Spi4) {
		RCC->APB2ENR &= ~RCC_APB2ENR_SPI4EN;
	}
	if constexpr (peripheral == Peripheral::Spi6) {
		RCC->APB4ENR &= ~RCC_APB4ENR_SPI6EN;
	}
	if constexpr (peripheral == Peripheral::Tim1) {
		RCC->APB2ENR &= ~RCC_APB2ENR_TIM1EN;
	}
	if constexpr (peripheral == Peripheral::Tim12) {
		RCC->APB1LENR &= ~RCC_APB1LENR_TIM12EN;
	}
	if constexpr (peripheral == Peripheral::Tim13) {
		RCC->APB1LENR &= ~RCC_APB1LENR_TIM13EN;
	}
	if constexpr (peripheral == Peripheral::Tim14) {
		RCC->APB1LENR &= ~RCC_APB1LENR_TIM14EN;
	}
	if constexpr (peripheral == Peripheral::Tim15) {
		RCC->APB2ENR &= ~RCC_APB2ENR_TIM15EN;
	}
	if constexpr (peripheral == Peripheral::Tim16) {
		RCC->APB2ENR &= ~RCC_APB2ENR_TIM16EN;
	}
	if constexpr (peripheral == Peripheral::Tim17) {
		RCC->APB2ENR &= ~RCC_APB2ENR_TIM17EN;
	}
	if constexpr (peripheral == Peripheral::Tim2) {
		RCC->APB1LENR &= ~RCC_APB1LENR_TIM2EN;
	}
	if constexpr (peripheral == Peripheral::Tim23) {
		RCC->APB1HENR &= ~RCC_APB1HENR_TIM23EN;
	}
	if constexpr (peripheral == Peripheral::Tim24) {
		RCC->APB1HENR &= ~RCC_APB1HENR_TIM24EN;
	}
	if constexpr (peripheral == Peripheral::Tim3) {
		RCC->APB1LENR &= ~RCC_APB1LENR_TIM3EN;
	}
	if constexpr (peripheral == Peripheral::Tim4) {
		RCC->APB1LENR &= ~RCC_APB1LENR_TIM4EN;
	}
	if constexpr (peripheral == Peripheral::Tim5) {
		RCC->APB1LENR &= ~RCC_APB1LENR_TIM5EN;
	}
	if constexpr (peripheral == Peripheral::Tim6) {
		RCC->APB1LENR &= ~RCC_APB1LENR_TIM6EN;
	}
	if constexpr (peripheral == Peripheral::Tim7) {
		RCC->APB1LENR &= ~RCC_APB1LENR_TIM7EN;
	}
	if constexpr (peripheral == Peripheral::Tim8) {
		RCC->APB2ENR &= ~RCC_APB2ENR_TIM8EN;
	}
	if constexpr (peripheral == Peripheral::Uart4) {
		RCC->APB1LENR &= ~RCC_APB1LENR_UART4EN;
	}
	if constexpr (peripheral == Peripheral::Uart5) {
		RCC->APB1LENR &= ~RCC_APB1LENR_UART5EN;
	}
	if constexpr (peripheral == Peripheral::Uart7) {
		RCC->APB1LENR &= ~RCC_APB1LENR_UART7EN;
	}
	if constexpr (peripheral == Peripheral::Uart8) {
		RCC->APB1LENR &= ~RCC_APB1LENR_UART8EN;
	}
	if constexpr (peripheral == Peripheral::Uart9) {
		RCC->APB2ENR &= ~RCC_APB2ENR_UART9EN;
	}
	if constexpr (peripheral == Peripheral::Usart1) {
		RCC->APB2ENR &= ~RCC_APB2ENR_USART1EN;
	}
	if constexpr (peripheral == Peripheral::Usart10) {
		RCC->APB2ENR &= ~RCC_APB2ENR_USART10EN;
	}
	if constexpr (peripheral == Peripheral::Usart2) {
		RCC->APB1LENR &= ~RCC_APB1LENR_USART2EN;
	}
	if constexpr (peripheral == Peripheral::Usart3) {
		RCC->APB1LENR &= ~RCC_APB1LENR_USART3EN;
	}
	if constexpr (peripheral == Peripheral::Usart6) {
		RCC->APB2ENR &= ~RCC_APB2ENR_USART6EN;
	}
	if constexpr (peripheral == Peripheral::Usbotghs) {
		RCC->AHB1ENR &= ~RCC_AHB1ENR_USB1OTGHSEN;
	}
	if constexpr (peripheral == Peripheral::Usbotghsulpi) {
		RCC->AHB1ENR &= ~RCC_AHB1ENR_USB1OTGHSULPIEN;
	}
	if constexpr (peripheral == Peripheral::Wwdg1) {
		RCC->APB3ENR &= ~RCC_APB3ENR_WWDG1EN;
	}
	__DSB();
}

template< Peripheral peripheral >
bool
Rcc::isEnabled()
{
	static_assert(rcc_check_enable(peripheral),
		"Rcc::isEnabled() doesn't know this peripheral!");

	if constexpr (peripheral == Peripheral::Adc3)
		return RCC->AHB4ENR & RCC_AHB4ENR_ADC3EN;
	if constexpr (peripheral == Peripheral::Bdma)
		return RCC->AHB4ENR & RCC_AHB4ENR_BDMAEN;
	if constexpr (peripheral == Peripheral::Comp1)
		return RCC->APB4ENR & RCC_APB4ENR_COMP12EN;
	if constexpr (peripheral == Peripheral::Cordic)
		return RCC->AHB2ENR & RCC_AHB2ENR_CORDICEN;
	if constexpr (peripheral == Peripheral::Crc)
		return RCC->AHB4ENR & RCC_AHB4ENR_CRCEN;
	if constexpr (peripheral == Peripheral::Dac1)
		return RCC->APB1LENR & RCC_APB1LENR_DAC12EN;
	if constexpr (peripheral == Peripheral::Dcmi)
		return RCC->AHB2ENR & RCC_AHB2ENR_DCMIEN;
	if constexpr (peripheral == Peripheral::Dfsdm1)
		return RCC->APB2ENR & RCC_APB2ENR_DFSDM1EN;
	if constexpr (peripheral == Peripheral::Dma1)
		return RCC->AHB1ENR & RCC_AHB1ENR_DMA1EN;
	if constexpr (peripheral == Peripheral::Dma2)
		return RCC->AHB1ENR & RCC_AHB1ENR_DMA2EN;
	if constexpr (peripheral == Peripheral::Dma2d)
		return RCC->AHB3ENR & RCC_AHB3ENR_DMA2DEN;
	if constexpr (peripheral == Peripheral::Dts)
		return RCC->APB4ENR & RCC_APB4ENR_DTSEN;
	if constexpr (peripheral == Peripheral::Fdcan1)
		return RCC->APB1HENR & RCC_APB1HENR_FDCANEN;
	if constexpr (peripheral == Peripheral::Fmac)
		return RCC->AHB2ENR & RCC_AHB2ENR_FMACEN;
	if constexpr (peripheral == Peripheral::Fmc)
		return RCC->AHB3ENR & RCC_AHB3ENR_FMCEN;
	if constexpr (peripheral == Peripheral::Hdmicec)
		return RCC->APB1LLPENR & RCC_APB1LLPENR_HDMICECEN;
	if constexpr (peripheral == Peripheral::I2c1)
		return RCC->APB1LENR & RCC_APB1LENR_I2C1EN;
	if constexpr (peripheral == Peripheral::I2c2)
		return RCC->APB1LENR & RCC_APB1LENR_I2C2EN;
	if constexpr (peripheral == Peripheral::I2c3)
		return RCC->APB1LENR & RCC_APB1LENR_I2C3EN;
	if constexpr (peripheral == Peripheral::I2c4)
		return RCC->APB4ENR & RCC_APB4ENR_I2C4EN;
	if constexpr (peripheral == Peripheral::I2c5)
		return RCC->APB1LENR & RCC_APB1LENR_I2C5EN;
	if constexpr (peripheral == Peripheral::Lptim1)
		return RCC->APB1LENR & RCC_APB1LENR_LPTIM1EN;
	if constexpr (peripheral == Peripheral::Lptim2)
		return RCC->APB4ENR & RCC_APB4ENR_LPTIM2EN;
	if constexpr (peripheral == Peripheral::Lptim3)
		return RCC->APB4ENR & RCC_APB4ENR_LPTIM3EN;
	if constexpr (peripheral == Peripheral::Lptim4)
		return RCC->APB4ENR & RCC_APB4ENR_LPTIM4EN;
	if constexpr (peripheral == Peripheral::Lptim5)
		return RCC->APB4ENR & RCC_APB4ENR_LPTIM5EN;
	if constexpr (peripheral == Peripheral::Lpuart1)
		return RCC->APB4ENR & RCC_APB4ENR_LPUART1EN;
	if constexpr (peripheral == Peripheral::Ltdc)
		return RCC->APB3ENR & RCC_APB3ENR_LTDCEN;
	if constexpr (peripheral == Peripheral::Mdios)
		return RCC->APB1HENR & RCC_APB1HENR_MDIOSEN;
	if constexpr (peripheral == Peripheral::Mdma)
		return RCC->AHB3ENR & RCC_AHB3ENR_MDMAEN;
	if constexpr (peripheral == Peripheral::Rng)
		return RCC->AHB2ENR & RCC_AHB2ENR_RNGEN;
	if constexpr (peripheral == Peripheral::Rtc)
		return RCC->BDCR & RCC_BDCR_RTCEN;
	if constexpr (peripheral == Peripheral::Sai1)
		return RCC->APB2ENR & RCC_APB2ENR_SAI1EN;
	if constexpr (peripheral == Peripheral::Sai4)
		return RCC->APB4ENR & RCC_APB4ENR_SAI4EN;
	if constexpr (peripheral == Peripheral::Sdmmc1)
		return RCC->AHB3ENR & RCC_AHB3ENR_SDMMC1EN;
	if constexpr (peripheral == Peripheral::Sdmmc2)
		return RCC->AHB2ENR & RCC_AHB2ENR_SDMMC2EN;
	if constexpr (peripheral == Peripheral::Spdifrx)
		return RCC->APB1LENR & RCC_APB1LENR_SPDIFRXEN;
	if constexpr (peripheral == Peripheral::Spi1)
		return RCC->APB2ENR & RCC_APB2ENR_SPI1EN;
	if constexpr (peripheral == Peripheral::Spi2)
		return RCC->APB1LENR & RCC_APB1LENR_SPI2EN;
	if constexpr (peripheral == Peripheral::Spi3)
		return RCC->APB1LENR & RCC_APB1LENR_SPI3EN;
	if constexpr (peripheral == Peripheral::Spi4)
		return RCC->APB2ENR & RCC_APB2ENR_SPI4EN;
	if constexpr (peripheral == Peripheral::Spi6)
		return RCC->APB4ENR & RCC_APB4ENR_SPI6EN;
	if constexpr (peripheral == Peripheral::Tim1)
		return RCC->APB2ENR & RCC_APB2ENR_TIM1EN;
	if constexpr (peripheral == Peripheral::Tim12)
		return RCC->APB1LENR & RCC_APB1LENR_TIM12EN;
	if constexpr (peripheral == Peripheral::Tim13)
		return RCC->APB1LENR & RCC_APB1LENR_TIM13EN;
	if constexpr (peripheral == Peripheral::Tim14)
		return RCC->APB1LENR & RCC_APB1LENR_TIM14EN;
	if constexpr (peripheral == Peripheral::Tim15)
		return RCC->APB2ENR & RCC_APB2ENR_TIM15EN;
	if constexpr (peripheral == Peripheral::Tim16)
		return RCC->APB2ENR & RCC_APB2ENR_TIM16EN;
	if constexpr (peripheral == Peripheral::Tim17)
		return RCC->APB2ENR & RCC_APB2ENR_TIM17EN;
	if constexpr (peripheral == Peripheral::Tim2)
		return RCC->APB1LENR & RCC_APB1LENR_TIM2EN;
	if constexpr (peripheral == Peripheral::Tim23)
		return RCC->APB1HENR & RCC_APB1HENR_TIM23EN;
	if constexpr (peripheral == Peripheral::Tim24)
		return RCC->APB1HENR & RCC_APB1HENR_TIM24EN;
	if constexpr (peripheral == Peripheral::Tim3)
		return RCC->APB1LENR & RCC_APB1LENR_TIM3EN;
	if constexpr (peripheral == Peripheral::Tim4)
		return RCC->APB1LENR & RCC_APB1LENR_TIM4EN;
	if constexpr (peripheral == Peripheral::Tim5)
		return RCC->APB1LENR & RCC_APB1LENR_TIM5EN;
	if constexpr (peripheral == Peripheral::Tim6)
		return RCC->APB1LENR & RCC_APB1LENR_TIM6EN;
	if constexpr (peripheral == Peripheral::Tim7)
		return RCC->APB1LENR & RCC_APB1LENR_TIM7EN;
	if constexpr (peripheral == Peripheral::Tim8)
		return RCC->APB2ENR & RCC_APB2ENR_TIM8EN;
	if constexpr (peripheral == Peripheral::Uart4)
		return RCC->APB1LENR & RCC_APB1LENR_UART4EN;
	if constexpr (peripheral == Peripheral::Uart5)
		return RCC->APB1LENR & RCC_APB1LENR_UART5EN;
	if constexpr (peripheral == Peripheral::Uart7)
		return RCC->APB1LENR & RCC_APB1LENR_UART7EN;
	if constexpr (peripheral == Peripheral::Uart8)
		return RCC->APB1LENR & RCC_APB1LENR_UART8EN;
	if constexpr (peripheral == Peripheral::Uart9)
		return RCC->APB2ENR & RCC_APB2ENR_UART9EN;
	if constexpr (peripheral == Peripheral::Usart1)
		return RCC->APB2ENR & RCC_APB2ENR_USART1EN;
	if constexpr (peripheral == Peripheral::Usart10)
		return RCC->APB2ENR & RCC_APB2ENR_USART10EN;
	if constexpr (peripheral == Peripheral::Usart2)
		return RCC->APB1LENR & RCC_APB1LENR_USART2EN;
	if constexpr (peripheral == Peripheral::Usart3)
		return RCC->APB1LENR & RCC_APB1LENR_USART3EN;
	if constexpr (peripheral == Peripheral::Usart6)
		return RCC->APB2ENR & RCC_APB2ENR_USART6EN;
	if constexpr (peripheral == Peripheral::Usbotghs)
		return RCC->AHB1ENR & RCC_AHB1ENR_USB1OTGHSEN;
	if constexpr (peripheral == Peripheral::Usbotghsulpi)
		return RCC->AHB1ENR & RCC_AHB1ENR_USB1OTGHSULPIEN;
	if constexpr (peripheral == Peripheral::Wwdg1)
		return RCC->APB3ENR & RCC_APB3ENR_WWDG1EN;
}

}   // namespace modm::platform