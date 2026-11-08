/*
 * Copyright (c) 2012, 2016, Sascha Schade
 * Copyright (c) 2012, 2017, Fabian Greif
 * Copyright (c) 2012, 2014-2017, Niklas Hauser
 * Copyright (c) 2013-2014, Kevin Läufer
 * Copyright (c) 2018, 2021-2022, Christopher Durand
 *
 * This file is part of the modm project.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
// ----------------------------------------------------------------------------

#ifndef MODM_STM32_RCC_HPP
#define MODM_STM32_RCC_HPP

#include <cstdint>
#include "../device.hpp"
#include <modm/platform/core/peripherals.hpp>
#include <modm/platform/gpio/connector.hpp>
#include <modm/architecture/interface/delay.hpp>

namespace modm::platform
{

/**
 * Reset and Clock Control for STM32 devices.
 *
 * This class abstracts access to clock settings on the STM32.
 * You need to use this class to enable internal and external clock
 * sources & outputs, set PLL parameters and AHB & APB prescalers.
 * Don't forget to set the flash latencies.
 *
 * @author		Niklas Hauser
 * @ingroup		modm_platform_rcc
 */
class Rcc
{
public:
	static constexpr uint32_t LsiFrequency = 32'000;
	static constexpr uint32_t HsiFrequency = 64'000'000;
	static constexpr uint32_t BootFrequency = 64'000'000;
	static constexpr uint32_t MaxFrequency = 550'000'000;

	enum class
	PllSource : uint32_t
	{
		/// High speed internal clock (4-64 MHz)
		Hsi = RCC_PLLCKSELR_PLLSRC_HSI,
		InternalClock = Hsi,
		/// Low speed internal clock (4MHz)
		Cse = RCC_PLLCKSELR_PLLSRC_CSI,
		/// High speed external clock
		Hse = RCC_PLLCKSELR_PLLSRC_HSE,
		ExternalClock = Hse,
		ExternalCrystal = Hse,
	};
	enum class
	SystemClockSource : uint32_t
	{
		Hsi = RCC_CFGR_SW_HSI,
		Hse = RCC_CFGR_SW_HSE,
		InternalClock = Hsi,
		ExternalClock = Hse,
		ExternalCrystal = Hse,
		Pll1P = RCC_CFGR_SW_PLL1,
	};

	enum class
	RealTimeClockSource : uint32_t
	{
		Lsi = RCC_BDCR_RTCSEL_1,
		Lse = RCC_BDCR_RTCSEL_0,
		Hse = RCC_BDCR_RTCSEL_0 | RCC_BDCR_RTCSEL_1,

		ExternalClock = Hse,
		ExternalCrystal = Hse,
		LowSpeedInternalClock = Lsi,
		LowSpeedExternalClock = Lse,
		LowSpeedExternalCrystal = Lse
	};
	enum class
	WatchdogClockSource : uint32_t
	{
		LowSpeedInternalClock = 0
	};

	enum class
	AhbPrescaler : uint32_t
	{
		Div1   = RCC_D1CFGR_HPRE_DIV1,
		Div2   = RCC_D1CFGR_HPRE_DIV2,
		Div4   = RCC_D1CFGR_HPRE_DIV4,
		Div8   = RCC_D1CFGR_HPRE_DIV8,
		Div16  = RCC_D1CFGR_HPRE_DIV16,
		Div64  = RCC_D1CFGR_HPRE_DIV64,
		Div128 = RCC_D1CFGR_HPRE_DIV128,
		Div256 = RCC_D1CFGR_HPRE_DIV256,
		Div512 = RCC_D1CFGR_HPRE_DIV512
	};

	enum class
	Apb1Prescaler : uint32_t
	{
		Div1   = RCC_D2CFGR_D2PPRE1_DIV1,
		Div2   = RCC_D2CFGR_D2PPRE1_DIV2,
		Div4   = RCC_D2CFGR_D2PPRE1_DIV4,
		Div8   = RCC_D2CFGR_D2PPRE1_DIV8,
		Div16  = RCC_D2CFGR_D2PPRE1_DIV16
	};

	enum class
	Apb2Prescaler : uint32_t
	{
		Div1   = RCC_D2CFGR_D2PPRE2_DIV1,
		Div2   = RCC_D2CFGR_D2PPRE2_DIV2,
		Div4   = RCC_D2CFGR_D2PPRE2_DIV4,
		Div8   = RCC_D2CFGR_D2PPRE2_DIV8,
		Div16  = RCC_D2CFGR_D2PPRE2_DIV16
	};
	enum class
	Apb3Prescaler : uint32_t
	{
		Div1   = RCC_D1CFGR_D1PPRE_DIV1,
		Div2   = RCC_D1CFGR_D1PPRE_DIV2,
		Div4   = RCC_D1CFGR_D1PPRE_DIV4,
		Div8   = RCC_D1CFGR_D1PPRE_DIV8,
		Div16  = RCC_D1CFGR_D1PPRE_DIV16
	};

	enum class
	Apb4Prescaler : uint32_t
	{
		Div1   = RCC_D3CFGR_D3PPRE_DIV1,
		Div2   = RCC_D3CFGR_D3PPRE_DIV2,
		Div4   = RCC_D3CFGR_D3PPRE_DIV4,
		Div8   = RCC_D3CFGR_D3PPRE_DIV8,
		Div16  = RCC_D3CFGR_D3PPRE_DIV16
	};

	enum class
	PllInputRange : uint8_t
	{
		MHz1_2 = 0,
		MHz2_4 = 1,
		MHz4_8 = 2,
		MHz8_16 = 3,
	};
	enum class
	ClockOutput1Source : uint32_t
	{
		Hsi = 0,
		Lse = RCC_CFGR_MCO1_0,
		Hse = RCC_CFGR_MCO1_1,
		Pll1Q = RCC_CFGR_MCO1_1 | RCC_CFGR_MCO1_0,
		Hsi48 = RCC_CFGR_MCO1_2
	};

	enum class
	ClockOutput2Source : uint32_t
	{
		SystemClock = 0,
		Pll2P = RCC_CFGR_MCO2_0,
		Hse = RCC_CFGR_MCO2_1,
		Pll = RCC_CFGR_MCO2_1 | RCC_CFGR_MCO2_0,
		Csi = RCC_CFGR_MCO2_2,
		Lsi = RCC_CFGR_MCO2_2 | RCC_CFGR_MCO2_0
	};
	enum class
	CanClockSource : uint32_t
	{
		Hse = 0,
		Pll1Q = 0b01 << RCC_D2CCIP1R_FDCANSEL_Pos,
		Pll2Q = 0b10 << RCC_D2CCIP1R_FDCANSEL_Pos,
	};

	static void
	setCanClockSource(CanClockSource source)
	{
		RCC->D2CCIP1R = (RCC->D2CCIP1R & ~RCC_D2CCIP1R_FDCANSEL_Msk) | uint32_t(source);
	}

	/// FDCAN subsystem prescaler common to all FDCAN instances
	enum class
	CanPrescaler : uint8_t
	{
		Div1  = 0b0000,
		Div2  = 0b0001,
		Div4  = 0b0010,
		Div6  = 0b0011,
		Div8  = 0b0100,
		Div10 = 0b0101,
		Div12 = 0b0110,
		Div14 = 0b0111,
		Div16 = 0b1000,
		Div18 = 0b1001,
		Div20 = 0b1010,
		Div22 = 0b1011,
		Div24 = 0b1100,
		Div26 = 0b1101,
		Div28 = 0b1110,
		Div30 = 0b1111,
	};

	/// Configure CAN subsystem prescaler
	/// \warning Configure the prescaler before enabling the CAN peripherals
	/// \returns true if setting the prescaler was successful
	static bool
	setCanPrescaler(CanPrescaler prescaler);
public:
	// sources
	static bool
	enableInternalClock(uint32_t waitCycles = 2048);

	static bool
	enableInternalClockMHz48(uint32_t waitCycles = 2048);
	static bool
	enableExternalClock(uint32_t waitCycles = 2048);

	static bool
	enableExternalCrystal(uint32_t waitCycles = 2048);

	static bool
	enableLowSpeedInternalClock(uint32_t waitCycles = 2048);

	static bool
	enableLowSpeedExternalClock(uint32_t waitCycles = 2048);

	static bool
	enableLowSpeedExternalCrystal(uint32_t waitCycles = 2048);

	struct PllFactors
	{
		PllInputRange range;
		uint8_t pllM;
		uint16_t pllN;
		uint8_t pllP;
		uint8_t pllQ;
		uint8_t pllR;
		uint16_t pllFrac = 0;
	};
	/**
	 * Enable PLL1.
	 *
	 * \param	source
	 * 		Source select for PLL. If you are using HSE you must
	 * 		enable it first (see enableHse()).
	 *
	 * \param	factors
	 * 		Struct with all pll factors. \see PllFactors.
	 *
	 * \param	waitCycles
	 * 		Number of cycles to wait for the pll to stabilise. Default: 2048.
	 */
	static bool
	enablePll1(PllSource source, const PllFactors& pllFactors, uint32_t waitCycles = 2048);

	/**
	 * Disable PLL1.
	 *
	 * \param	waitCycles
	 * 		Number of cycles to wait for the pll to stabilise. Default: 2048.
	 */
	static bool
	disablePll1(uint32_t waitCycles = 2048);
	/**
	 * Enable PLL2.
	 *
	 * \param	source
	 * 		Source select for PLL. If you are using HSE you must
	 * 		enable it first (see enableHse()).
	 *
	 * \param	factors
	 * 		Struct with all pll factors. \see PllFactors.
	 *
	 * \param	waitCycles
	 * 		Number of cycles to wait for the pll to stabilise. Default: 2048.
	 */
	static bool
	enablePll2(PllSource source, const PllFactors& pllFactors, uint32_t waitCycles = 2048);

	/**
	 * Disable PLL2.
	 *
	 * \param	waitCycles
	 * 		Number of cycles to wait for the pll to stabilise. Default: 2048.
	 */
	static bool
	disablePll2(uint32_t waitCycles = 2048);
	/**
	 * Enable PLL3.
	 *
	 * \param	source
	 * 		Source select for PLL. If you are using HSE you must
	 * 		enable it first (see enableHse()).
	 *
	 * \param	factors
	 * 		Struct with all pll factors. \see PllFactors.
	 *
	 * \param	waitCycles
	 * 		Number of cycles to wait for the pll to stabilise. Default: 2048.
	 */
	static bool
	enablePll3(PllSource source, const PllFactors& pllFactors, uint32_t waitCycles = 2048);

	/**
	 * Disable PLL3.
	 *
	 * \param	waitCycles
	 * 		Number of cycles to wait for the pll to stabilise. Default: 2048.
	 */
	static bool
	disablePll3(uint32_t waitCycles = 2048);
	// sinks
	static bool
	enableSystemClock(SystemClockSource src, uint32_t waitCycles = 2048);

	static inline bool
	enableRealTimeClock(RealTimeClockSource src, uint8_t hseDiv = 0xff)
	{
		RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_RTCPRE_Msk) | ((uint8_t(hseDiv) << RCC_CFGR_RTCPRE_Pos) & RCC_CFGR_RTCPRE_Msk);
		RCC->BDCR = (RCC->BDCR & ~RCC_BDCR_RTCSEL) | RCC_BDCR_RTCEN | uint32_t(src);
		return true;
	}

	static inline bool
	enableWatchdogClock(WatchdogClockSource /*src*/)
	{ return true; }

	enum class
	UsbClockSource : uint32_t
	{
		Disabled = 0,
		Pll1Q = RCC_D2CCIP2R_USBSEL_0,
		Pll3Q = RCC_D2CCIP2R_USBSEL_1,
		Hsi48 = RCC_D2CCIP2R_USBSEL_1 | RCC_D2CCIP2R_USBSEL_0,
	};
	static inline void
	enableUsbClockSource(UsbClockSource source)
	{
		RCC->D2CCIP2R = (RCC->D2CCIP2R & ~RCC_D2CCIP2R_USBSEL_Msk) | uint32_t(source);
	}
	static inline bool
	enableClockOutput1(ClockOutput1Source src, uint8_t div)
	{
		uint32_t tmp = RCC->CFGR & ~(RCC_CFGR_MCO1 | RCC_CFGR_MCO1PRE);
		if (div > 1) tmp |= (div << 18);
		RCC->CFGR = tmp | uint32_t(src);
		return true;
	}

	static inline bool
	enableClockOutput2(ClockOutput2Source src, uint8_t div)
	{
		uint32_t tmp = RCC->CFGR & ~(RCC_CFGR_MCO2 | RCC_CFGR_MCO2PRE);
		if (div > 1) tmp |= (div << 25);
		RCC->CFGR = tmp | uint32_t(src);
		return true;
	}
public:
	static inline bool
	setAhbPrescaler(AhbPrescaler prescaler)
	{
		RCC->D1CFGR = (RCC->D1CFGR & ~RCC_D1CFGR_HPRE) | uint32_t(prescaler);
		return true;
	}

	static inline bool
	setApb1Prescaler(Apb1Prescaler prescaler)
	{
		RCC->D2CFGR = (RCC->D2CFGR & ~RCC_D2CFGR_D2PPRE1) | uint32_t(prescaler);
		return true;
	}

	static inline bool
	setApb2Prescaler(Apb2Prescaler prescaler)
	{
		RCC->D2CFGR = (RCC->D2CFGR & ~RCC_D2CFGR_D2PPRE2) | uint32_t(prescaler);
		return true;
	}
	static inline bool
	setApb3Prescaler(Apb3Prescaler prescaler)
	{
		RCC->D1CFGR = (RCC->D1CFGR & ~RCC_D1CFGR_D1PPRE) | uint32_t(prescaler);
		return true;
	}
	static inline bool
	setApb4Prescaler(Apb4Prescaler prescaler)
	{
		RCC->D3CFGR = (RCC->D3CFGR & ~RCC_D3CFGR_D3PPRE) | uint32_t(prescaler);
		return true;
	}
	enum class
	VoltageScaling : uint32_t
	{
		Scale3 = PWR_D3CR_VOS_0,
		Scale2 = PWR_D3CR_VOS_1,
		Scale1 = PWR_D3CR_VOS_1 | PWR_D3CR_VOS_0,
		Scale0 = 0
	};

	enum class
	PowerSource : uint32_t
	{
		/// Vcore supplied from LDO
		Ldo							= PWR_CR3_LDOEN,
		/// Vcore supplied from external source
		External					= PWR_CR3_BYPASS,
	};

	/// Configure power source, has to be called exactly once early on start-up
	static bool
	configurePowerSource(PowerSource source, uint32_t waitCycles = 2048);
	static bool
	setVoltageScaling(VoltageScaling voltage, uint32_t waitCycles = 2048);
public:
	/** Set flash latency for CPU frequency and voltage.
	 * Does nothing if CPU frequency is too high for the available
	 * voltage.
	 *
	 * @returns maximum CPU frequency for voltage.
	 * @retval	<=CPU_Frequency flash latency has been set correctly.
	 * @retval	>CPU_Frequency requested frequency too high for voltage.
	 */
	template< uint32_t Core_Hz, uint16_t Core_mV = 3300>
	static uint32_t
	setFlashLatency();

	template< uint32_t Core_Hz >
	static void
	updateCoreFrequency();

	template< class... Signals >
	static void
	connect()
	{
		using Connector = GpioConnector<Peripheral::Rcc, Signals...>;
		Connector::connect();
	}

public:
	template< Peripheral peripheral >
	static void
	enable();

	template< Peripheral peripheral >
	static bool
	isEnabled();

	template< Peripheral peripheral >
	static void
	disable();

private:
	struct flash_latency
	{
		uint32_t latency;
		uint32_t max_frequency;
	};
	static constexpr flash_latency
	computeFlashLatency(uint32_t Core_Hz, uint16_t Core_mV);
};

}   // namespace modm::platform


#include "rcc_impl.hpp"

#endif	//  MODM_STM32_RCC_HPP