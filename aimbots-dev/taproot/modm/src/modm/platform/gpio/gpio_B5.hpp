/*
 * Copyright (c) 2017-2018, Niklas Hauser
 *
 * This file is part of the modm project.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
// ----------------------------------------------------------------------------

#ifndef MODM_STM32_GPIO_PIN_B5_HPP
#define MODM_STM32_GPIO_PIN_B5_HPP

#include "../device.hpp"
#include "base.hpp"
#include "set.hpp"

namespace modm::platform
{

/// @cond
class GpioB5;
using GpioOutputB5 = GpioB5;
using GpioInputB5  = GpioB5;
/// @endcond

/// IO class for Pin B5
/// @ingroup	modm_platform_gpio
class GpioB5 : public Gpio, public ::modm::GpioIO
{
	template<class... Gpios>
	friend class GpioSet;
	using PinSet = GpioSet<GpioB5>;
	friend class Adc;
	friend class Adc1; friend class Adc2;
	friend class Adc3; friend class Adc4;
public:
	using Output = GpioB5;
	using Input = GpioB5;
	using IO = GpioB5;
	using Type = GpioB5;
	static constexpr bool isInverted = false;
	static constexpr Port port = Port::B; ///< Port name
	static constexpr uint8_t pin = 5; ///< Pin number
	static constexpr IRQn_Type ExternalInterruptIRQ = EXTI9_5_IRQn;

protected:
	/// Bitmask for registers that contain a 1bit value for every pin.
	static constexpr uint16_t mask  = 0x1 << pin;
	/// Bitmask for registers that contain a 2bit value for every pin.
	static constexpr uint32_t mask2 = 0x3 << (pin * 2);
	/// Port Number.
	static constexpr uint8_t port_nr = uint8_t(port);
	/// Alternate Function register id. 0 for pin 0-7. 1 for pin 8-15.
	static constexpr uint8_t af_id  = pin / 8;
	/// Alternate Function offset.
	static constexpr uint8_t af_offset = (pin * 4) % 32;
	/// Alternate Function register mask.
	static constexpr uint32_t af_mask  = 0xf << af_offset;

public:
	/// @cond
	inline static void setAlternateFunction(uint8_t af) {
		GPIOB->AFR[af_id] = (GPIOB->AFR[af_id] & ~af_mask) | ((af & 0xf) << af_offset);
		GPIOB->MODER = (GPIOB->MODER & ~mask2) | (i(Mode::AlternateFunction) << (pin * 2));
	}

	/// Enable Analog Mode which is needed to use this pin as an ADC input.
	inline static void setAnalogInput() { PinSet::setAnalogInput(); }
	/// @endcond

public:
	// GpioOutput
	// start documentation inherited
	inline static void setOutput() { PinSet::setOutput(); }
	inline static void setOutput(bool status) { PinSet::setOutput(status); }
	inline static void set() { PinSet::set(); }
	inline static void set(bool status) { PinSet::set(status); }
	inline static void reset() { PinSet::reset(); }
	inline static bool toggle() {
		if (isSet()) { reset(); return true; }
		else         { set();   return false; }
	}
	inline static bool isSet() { return (GPIOB->ODR & mask); }
	// stop documentation inherited
	inline static void configure(OutputType type, OutputSpeed speed = OutputSpeed::MHz50) { PinSet::configure(type, speed); }
	inline static void setOutput(OutputType type, OutputSpeed speed = OutputSpeed::MHz50) { PinSet::setOutput(type, speed); }
	// GpioInput
	// start documentation inherited
	inline static void setInput() { PinSet::setInput(); }
	inline static bool read() { return (GPIOB->IDR & mask); }
	// end documentation inherited
	inline static void configure(InputType type) { PinSet::configure(type); }
	inline static void setInput(InputType type) { PinSet::setInput(type); }
	// External Interrupts
	// Warning: This will disable any previously enabled interrupt which is
	// routed to the same interupt line, e.g. PA3 will disable PB3.
	// This is a hardware limitation by the STM32 EXTI.
	inline static void enableExternalInterrupt()
	{
		// PA[x], x =  0 ..  3 maps to EXTICR[0]
		// PA[x], x =  4 ..  7 maps to EXTICR[1]
		// PA[x], x =  8 .. 11 maps to EXTICR[2]
		// PA[x], x = 12 .. 15 maps to EXTICR[3]
		// => bit3 and bit2 (mask 0x0c) specify the register
		// => bit1 and bit0 (mask 0x03) specify the bit position
		constexpr uint8_t index   = (pin & 0b1100) >> 2;
		constexpr uint8_t bit_pos = (pin & 0b0011) << 2;
		constexpr uint16_t syscfg_mask = (0b1111) << bit_pos;
		constexpr uint16_t syscfg_value = (port_nr & (0b1111)) << bit_pos;
		SYSCFG->EXTICR[index] = (SYSCFG->EXTICR[index] & ~syscfg_mask) | syscfg_value;
		EXTI->IMR |= mask;
	}
	inline static void disableExternalInterrupt() { EXTI->IMR &= ~mask; }
	inline static void enableExternalInterruptVector(const uint32_t priority)
	{
		NVIC_SetPriority(ExternalInterruptIRQ, priority);
		NVIC_EnableIRQ(ExternalInterruptIRQ);
	}
	inline static void disableExternalInterruptVector() { NVIC_DisableIRQ(ExternalInterruptIRQ); }
	inline static void setInputTrigger(const InputTrigger trigger)
	{
		switch (trigger)
		{
		case InputTrigger::RisingEdge:
			EXTI->RTSR |=  mask;
			EXTI->FTSR &= ~mask;
			break;
		case InputTrigger::FallingEdge:
			EXTI->RTSR &= ~mask;
			EXTI->FTSR |=  mask;
			break;
		case InputTrigger::BothEdges:
			EXTI->RTSR |=  mask;
			EXTI->FTSR |=  mask;
			break;
		}
	}
	inline static bool getExternalInterruptFlag() { return (EXTI->PR & mask); }
	inline static void acknowledgeExternalInterruptFlag() { EXTI->PR = mask; }
	// GpioIO
	// start documentation inherited
	inline static Direction getDirection() {
		uint32_t mode = (GPIOB->MODER & mask2);
		if (mode == (i(Mode::Input) << pin * 2)) {
			return Direction::In;
		}
		if (mode == (i(Mode::Output) << pin * 2)) {
			return Direction::Out;
		}
		return Direction::Special;
	}
	// end documentation inherited
	inline static void lock() { PinSet::lock(); }
	inline static void disconnect() {
		setInput(InputType::Floating);
		GPIOB->AFR[af_id] &= ~af_mask;
	}

public:
#ifdef __DOXYGEN__
	/// @{
	/// Connect to any software peripheral
	using BitBang = GpioSignal;
	/// Connect to Tim3
	using Ch2 = GpioSignal;
	/// Connect to Dcmi
	using D10 = GpioSignal;
	/// Connect to Spi1 or Spi3
	using Mosi = GpioSignal;
	/// Connect to Eth
	using Ppsout = GpioSignal;
	/// Connect to Can2
	using Rx = GpioSignal;
	/// Connect to I2s3
	using Sd = GpioSignal;
	/// Connect to Fmc
	using Sdcke1 = GpioSignal;
	/// Connect to I2c1
	using Smba = GpioSignal;
	/// Connect to Usbotghs
	using Ulpid7 = GpioSignal;
	/// @}
#endif
	/// @cond
	template< Peripheral peripheral >
	struct BitBang { static void connect();
		static_assert(
			(peripheral == Peripheral::BitBang),
			"GpioB5::BitBang only connects to software drivers!");
	};
	template< Peripheral peripheral >
	struct Ch2 { static void connect();
		static_assert(
			(peripheral == Peripheral::Tim3),
			"GpioB5::Ch2 only connects to Tim3!");
	};
	template< Peripheral peripheral >
	struct D10 { static void connect();
		static_assert(
			(peripheral == Peripheral::Dcmi),
			"GpioB5::D10 only connects to Dcmi!");
	};
	template< Peripheral peripheral >
	struct Mosi { static void connect();
		static_assert(
			(peripheral == Peripheral::Spi1) ||
			(peripheral == Peripheral::Spi3),
			"GpioB5::Mosi only connects to Spi1 or Spi3!");
	};
	template< Peripheral peripheral >
	struct Ppsout { static void connect();
		static_assert(
			(peripheral == Peripheral::Eth),
			"GpioB5::Ppsout only connects to Eth!");
	};
	template< Peripheral peripheral >
	struct Rx { static void connect();
		static_assert(
			(peripheral == Peripheral::Can2),
			"GpioB5::Rx only connects to Can2!");
	};
	template< Peripheral peripheral >
	struct Sd { static void connect();
		static_assert(
			(peripheral == Peripheral::I2s3),
			"GpioB5::Sd only connects to I2s3!");
	};
	template< Peripheral peripheral >
	struct Sdcke1 { static void connect();
		static_assert(
			(peripheral == Peripheral::Fmc),
			"GpioB5::Sdcke1 only connects to Fmc!");
	};
	template< Peripheral peripheral >
	struct Smba { static void connect();
		static_assert(
			(peripheral == Peripheral::I2c1),
			"GpioB5::Smba only connects to I2c1!");
	};
	template< Peripheral peripheral >
	struct Ulpid7 { static void connect();
		static_assert(
			(peripheral == Peripheral::Usbotghs),
			"GpioB5::Ulpid7 only connects to Usbotghs!");
	};
	/// @endcond
private:
	template< Peripheral peripheral >
	static constexpr int8_t AdcChannel = -1;
	template< Peripheral peripheral >
	static constexpr int8_t DacChannel = -1;
};

/// @cond
template<>
struct GpioB5::BitBang<Peripheral::BitBang>
{
	using Gpio = GpioB5;
	static constexpr Gpio::Signal Signal = Gpio::Signal::BitBang;
	static constexpr int af = -1;
	inline static void connect() {}
};
template<>
struct GpioB5::Ch2<Peripheral::Tim3>
{
	using Gpio = GpioB5;
	static constexpr Gpio::Signal Signal = Gpio::Signal::Ch2;
	static constexpr int af = 2;
	inline static void
	connect()
	{
		setAlternateFunction(2);
	}
};
template<>
struct GpioB5::D10<Peripheral::Dcmi>
{
	using Gpio = GpioB5;
	static constexpr Gpio::Signal Signal = Gpio::Signal::D10;
	static constexpr int af = 13;
	inline static void
	connect()
	{
		setAlternateFunction(13);
	}
};
template<>
struct GpioB5::Mosi<Peripheral::Spi1>
{
	using Gpio = GpioB5;
	static constexpr Gpio::Signal Signal = Gpio::Signal::Mosi;
	static constexpr int af = 5;
	inline static void
	connect()
	{
		setAlternateFunction(5);
	}
};
template<>
struct GpioB5::Mosi<Peripheral::Spi3>
{
	using Gpio = GpioB5;
	static constexpr Gpio::Signal Signal = Gpio::Signal::Mosi;
	static constexpr int af = 6;
	inline static void
	connect()
	{
		setAlternateFunction(6);
	}
};
template<>
struct GpioB5::Ppsout<Peripheral::Eth>
{
	using Gpio = GpioB5;
	static constexpr Gpio::Signal Signal = Gpio::Signal::Ppsout;
	static constexpr int af = 11;
	inline static void
	connect()
	{
		setAlternateFunction(11);
	}
};
template<>
struct GpioB5::Rx<Peripheral::Can2>
{
	using Gpio = GpioB5;
	static constexpr Gpio::Signal Signal = Gpio::Signal::Rx;
	static constexpr int af = 9;
	inline static void
	connect()
	{
		setAlternateFunction(9);
	}
};
template<>
struct GpioB5::Sd<Peripheral::I2s3>
{
	using Gpio = GpioB5;
	static constexpr Gpio::Signal Signal = Gpio::Signal::Sd;
	static constexpr int af = 6;
	inline static void
	connect()
	{
		setAlternateFunction(6);
	}
};
template<>
struct GpioB5::Sdcke1<Peripheral::Fmc>
{
	using Gpio = GpioB5;
	static constexpr Gpio::Signal Signal = Gpio::Signal::Sdcke1;
	static constexpr int af = 12;
	inline static void
	connect()
	{
		setAlternateFunction(12);
	}
};
template<>
struct GpioB5::Smba<Peripheral::I2c1>
{
	using Gpio = GpioB5;
	static constexpr Gpio::Signal Signal = Gpio::Signal::Smba;
	static constexpr int af = 4;
	inline static void
	connect()
	{
		setAlternateFunction(4);
	}
};
template<>
struct GpioB5::Ulpid7<Peripheral::Usbotghs>
{
	using Gpio = GpioB5;
	static constexpr Gpio::Signal Signal = Gpio::Signal::Ulpid7;
	static constexpr int af = 10;
	inline static void
	connect()
	{
		setAlternateFunction(10);
	}
};
/// @endcond

} // namespace modm::platform

#endif // MODM_STM32_GPIO_PIN_B5_HPP