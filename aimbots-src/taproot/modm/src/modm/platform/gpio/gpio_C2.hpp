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

#ifndef MODM_STM32_GPIO_PIN_C2_HPP
#define MODM_STM32_GPIO_PIN_C2_HPP

#include "../device.hpp"
#include "base.hpp"
#include "set.hpp"

namespace modm::platform
{

/// @cond
class GpioC2;
using GpioOutputC2 = GpioC2;
using GpioInputC2  = GpioC2;
/// @endcond

/// IO class for Pin C2
/// @ingroup	modm_platform_gpio
class GpioC2 : public Gpio, public ::modm::GpioIO
{
	template<class... Gpios>
	friend class GpioSet;
	using PinSet = GpioSet<GpioC2>;
	friend class Adc;
	friend class Adc1; friend class Adc2;
	friend class Adc3; friend class Adc4;
public:
	using Output = GpioC2;
	using Input = GpioC2;
	using IO = GpioC2;
	using Type = GpioC2;
	static constexpr bool isInverted = false;
	static constexpr Port port = Port::C; ///< Port name
	static constexpr uint8_t pin = 2; ///< Pin number
	static constexpr IRQn_Type ExternalInterruptIRQ = EXTI2_IRQn;

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
		GPIOC->AFR[af_id] = (GPIOC->AFR[af_id] & ~af_mask) | ((af & 0xf) << af_offset);
		GPIOC->MODER = (GPIOC->MODER & ~mask2) | (i(Mode::AlternateFunction) << (pin * 2));
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
	inline static bool isSet() { return (GPIOC->ODR & mask); }
	// stop documentation inherited
	inline static void configure(OutputType type, OutputSpeed speed = OutputSpeed::MHz50) { PinSet::configure(type, speed); }
	inline static void setOutput(OutputType type, OutputSpeed speed = OutputSpeed::MHz50) { PinSet::setOutput(type, speed); }
	// GpioInput
	// start documentation inherited
	inline static void setInput() { PinSet::setInput(); }
	inline static bool read() { return (GPIOC->IDR & mask); }
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
		uint32_t mode = (GPIOC->MODER & mask2);
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
		GPIOC->AFR[af_id] &= ~af_mask;
	}

public:
#ifdef __DOXYGEN__
	/// @{
	/// Connect to any software peripheral
	using BitBang = GpioSignal;
	/// Connect to I2s2
	using Extsd = GpioSignal;
	/// Connect to Adc1 or Adc2 or Adc3
	using In12 = GpioSignal;
	/// Connect to Spi2
	using Miso = GpioSignal;
	/// Connect to Fmc
	using Sdne0 = GpioSignal;
	/// Connect to Eth
	using Txd2 = GpioSignal;
	/// Connect to Usbotghs
	using Ulpidir = GpioSignal;
	/// @}
#endif
	/// @cond
	template< Peripheral peripheral >
	struct BitBang { static void connect();
		static_assert(
			(peripheral == Peripheral::BitBang),
			"GpioC2::BitBang only connects to software drivers!");
	};
	template< Peripheral peripheral >
	struct Extsd { static void connect();
		static_assert(
			(peripheral == Peripheral::I2s2),
			"GpioC2::Extsd only connects to I2s2!");
	};
	template< Peripheral peripheral >
	struct In12 { static void connect();
		static_assert(
			(peripheral == Peripheral::Adc1) ||
			(peripheral == Peripheral::Adc2) ||
			(peripheral == Peripheral::Adc3),
			"GpioC2::In12 only connects to Adc1 or Adc2 or Adc3!");
	};
	template< Peripheral peripheral >
	struct Miso { static void connect();
		static_assert(
			(peripheral == Peripheral::Spi2),
			"GpioC2::Miso only connects to Spi2!");
	};
	template< Peripheral peripheral >
	struct Sdne0 { static void connect();
		static_assert(
			(peripheral == Peripheral::Fmc),
			"GpioC2::Sdne0 only connects to Fmc!");
	};
	template< Peripheral peripheral >
	struct Txd2 { static void connect();
		static_assert(
			(peripheral == Peripheral::Eth),
			"GpioC2::Txd2 only connects to Eth!");
	};
	template< Peripheral peripheral >
	struct Ulpidir { static void connect();
		static_assert(
			(peripheral == Peripheral::Usbotghs),
			"GpioC2::Ulpidir only connects to Usbotghs!");
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
struct GpioC2::BitBang<Peripheral::BitBang>
{
	using Gpio = GpioC2;
	static constexpr Gpio::Signal Signal = Gpio::Signal::BitBang;
	static constexpr int af = -1;
	inline static void connect() {}
};
template<>
struct GpioC2::Extsd<Peripheral::I2s2>
{
	using Gpio = GpioC2;
	static constexpr Gpio::Signal Signal = Gpio::Signal::Extsd;
	static constexpr int af = 6;
	inline static void
	connect()
	{
		setAlternateFunction(6);
	}
};
template<>
struct GpioC2::In12<Peripheral::Adc1>
{
	using Gpio = GpioC2;
	static constexpr Gpio::Signal Signal = Gpio::Signal::In12;
	static constexpr int af = -1;
	inline static void
	connect()
	{
		disconnect();
		setAnalogInput();
	}
};
template<>
constexpr int8_t
GpioC2::AdcChannel<Peripheral::Adc1> = 12;
template<>
struct GpioC2::In12<Peripheral::Adc2>
{
	using Gpio = GpioC2;
	static constexpr Gpio::Signal Signal = Gpio::Signal::In12;
	static constexpr int af = -1;
	inline static void
	connect()
	{
		disconnect();
		setAnalogInput();
	}
};
template<>
constexpr int8_t
GpioC2::AdcChannel<Peripheral::Adc2> = 12;
template<>
struct GpioC2::In12<Peripheral::Adc3>
{
	using Gpio = GpioC2;
	static constexpr Gpio::Signal Signal = Gpio::Signal::In12;
	static constexpr int af = -1;
	inline static void
	connect()
	{
		disconnect();
		setAnalogInput();
	}
};
template<>
constexpr int8_t
GpioC2::AdcChannel<Peripheral::Adc3> = 12;
template<>
struct GpioC2::Miso<Peripheral::Spi2>
{
	using Gpio = GpioC2;
	static constexpr Gpio::Signal Signal = Gpio::Signal::Miso;
	static constexpr int af = 5;
	inline static void
	connect()
	{
		setAlternateFunction(5);
	}
};
template<>
struct GpioC2::Sdne0<Peripheral::Fmc>
{
	using Gpio = GpioC2;
	static constexpr Gpio::Signal Signal = Gpio::Signal::Sdne0;
	static constexpr int af = 12;
	inline static void
	connect()
	{
		setAlternateFunction(12);
	}
};
template<>
struct GpioC2::Txd2<Peripheral::Eth>
{
	using Gpio = GpioC2;
	static constexpr Gpio::Signal Signal = Gpio::Signal::Txd2;
	static constexpr int af = 11;
	inline static void
	connect()
	{
		setAlternateFunction(11);
	}
};
template<>
struct GpioC2::Ulpidir<Peripheral::Usbotghs>
{
	using Gpio = GpioC2;
	static constexpr Gpio::Signal Signal = Gpio::Signal::Ulpidir;
	static constexpr int af = 10;
	inline static void
	connect()
	{
		setAlternateFunction(10);
	}
};
/// @endcond

} // namespace modm::platform

#endif // MODM_STM32_GPIO_PIN_C2_HPP