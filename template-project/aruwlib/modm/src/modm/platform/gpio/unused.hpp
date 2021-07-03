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

#ifndef MODM_STM32_GPIO_PIN_UNUSED_HPP
#define MODM_STM32_GPIO_PIN_UNUSED_HPP

#include "base.hpp"
#include <modm/architecture/interface/gpio.hpp>

namespace modm
{

namespace platform
{

/**
 * Dummy implementation of an I/O pin.
 *
 * This class can be used when a pin is not required. All functions
 * are dummy functions which do nothing. `read()` will always
 * return `false`.
 *
 * For example when creating a software SPI with the modm::SoftwareSimpleSpi
 * class and the return channel (MISO - Master In Slave Out) is not needed,
 * a good way is to use this class as a parameter when defining the
 * SPI class.
 *
 * Example:
 * @code
 * #include <modm/architecture/platform.hpp>
 *
 * namespace pin
 * {
 *     typedef GpioOutputD7 Clk;
 *     typedef GpioOutputD5 Mosi;
 * }
 *
 * modm::SoftwareSpiMaster< pin::Clk, pin::Mosi, GpioUnused > Spi;
 *
 * ...
 * Spi::write(0xaa);
 * @endcode
 *
 * @author	Fabian Greif
 * @author	Niklas Hauser
 * @ingroup	modm_platform_gpio
 */
class GpioUnused : public Gpio, public ::modm::GpioIO
{
public:
	using Output = GpioUnused;
	using Input = GpioUnused;
	using IO = GpioUnused;
	using Type = GpioUnused;
	static constexpr bool isInverted = false;
	static constexpr Port port = Port(-1);
	static constexpr uint8_t pin = uint8_t(-1);
	static constexpr uint16_t mask = 0;

protected:
	/// @cond
	static void setAlternateFunction(uint8_t) {}
	static void setAnalogInput() {}
	/// @endcond

public:
	// GpioOutput
	// start documentation inherited
	static void setOutput() {}
	static void setOutput(bool) {}
	static void set() {}
	static void set(bool) {}
	static void reset() {}
	static void toggle() {}
	static bool isSet() { return false; }
	// stop documentation inherited
	static void configure(OutputType, OutputSpeed = OutputSpeed::MHz50) {}
	static void setOutput(OutputType, OutputSpeed = OutputSpeed::MHz50) {}

	// GpioInput
	// start documentation inherited
	static void setInput() {}
	static bool read() { return false; }
	// end documentation inherited
	static void configure(InputType) {}
	static void setInput(InputType) {}
	// External Interrupts
	static void enableExternalInterrupt() {}
	static void disableExternalInterrupt() {}
	static void enableExternalInterruptVector(const uint32_t) {}
	static void disableExternalInterruptVector() {}
	static void setInputTrigger(const InputTrigger) {}
	static bool getExternalInterruptFlag() { return false; }
	/// Reset the interrupt flag in the interrupt routine.
	static void acknowledgeExternalInterruptFlag() {}

	// GpioIO
	// start documentation inherited
	static Direction getDirection() { return Direction::Special; }
	// end documentation inherited
	static void lock() {}
	static void disconnect() {}

public:
	/// @cond
	template< Peripheral _ >
	struct A0
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::A0;
		static void connect() {}
	};
	template< Peripheral _ >
	struct A1
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::A1;
		static void connect() {}
	};
	template< Peripheral _ >
	struct A10
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::A10;
		static void connect() {}
	};
	template< Peripheral _ >
	struct A11
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::A11;
		static void connect() {}
	};
	template< Peripheral _ >
	struct A12
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::A12;
		static void connect() {}
	};
	template< Peripheral _ >
	struct A13
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::A13;
		static void connect() {}
	};
	template< Peripheral _ >
	struct A14
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::A14;
		static void connect() {}
	};
	template< Peripheral _ >
	struct A15
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::A15;
		static void connect() {}
	};
	template< Peripheral _ >
	struct A16
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::A16;
		static void connect() {}
	};
	template< Peripheral _ >
	struct A17
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::A17;
		static void connect() {}
	};
	template< Peripheral _ >
	struct A18
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::A18;
		static void connect() {}
	};
	template< Peripheral _ >
	struct A19
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::A19;
		static void connect() {}
	};
	template< Peripheral _ >
	struct A2
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::A2;
		static void connect() {}
	};
	template< Peripheral _ >
	struct A20
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::A20;
		static void connect() {}
	};
	template< Peripheral _ >
	struct A21
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::A21;
		static void connect() {}
	};
	template< Peripheral _ >
	struct A22
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::A22;
		static void connect() {}
	};
	template< Peripheral _ >
	struct A23
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::A23;
		static void connect() {}
	};
	template< Peripheral _ >
	struct A24
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::A24;
		static void connect() {}
	};
	template< Peripheral _ >
	struct A25
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::A25;
		static void connect() {}
	};
	template< Peripheral _ >
	struct A3
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::A3;
		static void connect() {}
	};
	template< Peripheral _ >
	struct A4
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::A4;
		static void connect() {}
	};
	template< Peripheral _ >
	struct A5
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::A5;
		static void connect() {}
	};
	template< Peripheral _ >
	struct A6
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::A6;
		static void connect() {}
	};
	template< Peripheral _ >
	struct A7
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::A7;
		static void connect() {}
	};
	template< Peripheral _ >
	struct A8
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::A8;
		static void connect() {}
	};
	template< Peripheral _ >
	struct A9
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::A9;
		static void connect() {}
	};
	template< Peripheral _ >
	struct Af1
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::Af1;
		static void connect() {}
	};
	template< Peripheral _ >
	struct Af2
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::Af2;
		static void connect() {}
	};
	template< Peripheral _ >
	struct Ale
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::Ale;
		static void connect() {}
	};
	template< Peripheral _ >
	struct Ba0
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::Ba0;
		static void connect() {}
	};
	template< Peripheral _ >
	struct Ba1
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::Ba1;
		static void connect() {}
	};
	template< Peripheral _ >
	struct Bkin
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::Bkin;
		static void connect() {}
	};
	template< Peripheral _ >
	struct Cd
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::Cd;
		static void connect() {}
	};
	template< Peripheral _ >
	struct Ch1
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::Ch1;
		static void connect() {}
	};
	template< Peripheral _ >
	struct Ch1n
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::Ch1n;
		static void connect() {}
	};
	template< Peripheral _ >
	struct Ch2
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::Ch2;
		static void connect() {}
	};
	template< Peripheral _ >
	struct Ch2n
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::Ch2n;
		static void connect() {}
	};
	template< Peripheral _ >
	struct Ch3
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::Ch3;
		static void connect() {}
	};
	template< Peripheral _ >
	struct Ch3n
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::Ch3n;
		static void connect() {}
	};
	template< Peripheral _ >
	struct Ch4
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::Ch4;
		static void connect() {}
	};
	template< Peripheral _ >
	struct Ck
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::Ck;
		static void connect() {}
	};
	template< Peripheral _ >
	struct Ckin
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::Ckin;
		static void connect() {}
	};
	template< Peripheral _ >
	struct Cle
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::Cle;
		static void connect() {}
	};
	template< Peripheral _ >
	struct Clk
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::Clk;
		static void connect() {}
	};
	template< Peripheral _ >
	struct Cmd
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::Cmd;
		static void connect() {}
	};
	template< Peripheral _ >
	struct Col
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::Col;
		static void connect() {}
	};
	template< Peripheral _ >
	struct Crs
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::Crs;
		static void connect() {}
	};
	template< Peripheral _ >
	struct Cts
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::Cts;
		static void connect() {}
	};
	template< Peripheral _ >
	struct D0
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::D0;
		static void connect() {}
	};
	template< Peripheral _ >
	struct D1
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::D1;
		static void connect() {}
	};
	template< Peripheral _ >
	struct D10
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::D10;
		static void connect() {}
	};
	template< Peripheral _ >
	struct D11
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::D11;
		static void connect() {}
	};
	template< Peripheral _ >
	struct D12
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::D12;
		static void connect() {}
	};
	template< Peripheral _ >
	struct D13
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::D13;
		static void connect() {}
	};
	template< Peripheral _ >
	struct D14
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::D14;
		static void connect() {}
	};
	template< Peripheral _ >
	struct D15
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::D15;
		static void connect() {}
	};
	template< Peripheral _ >
	struct D16
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::D16;
		static void connect() {}
	};
	template< Peripheral _ >
	struct D17
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::D17;
		static void connect() {}
	};
	template< Peripheral _ >
	struct D18
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::D18;
		static void connect() {}
	};
	template< Peripheral _ >
	struct D19
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::D19;
		static void connect() {}
	};
	template< Peripheral _ >
	struct D2
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::D2;
		static void connect() {}
	};
	template< Peripheral _ >
	struct D20
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::D20;
		static void connect() {}
	};
	template< Peripheral _ >
	struct D21
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::D21;
		static void connect() {}
	};
	template< Peripheral _ >
	struct D22
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::D22;
		static void connect() {}
	};
	template< Peripheral _ >
	struct D23
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::D23;
		static void connect() {}
	};
	template< Peripheral _ >
	struct D24
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::D24;
		static void connect() {}
	};
	template< Peripheral _ >
	struct D25
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::D25;
		static void connect() {}
	};
	template< Peripheral _ >
	struct D26
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::D26;
		static void connect() {}
	};
	template< Peripheral _ >
	struct D27
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::D27;
		static void connect() {}
	};
	template< Peripheral _ >
	struct D28
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::D28;
		static void connect() {}
	};
	template< Peripheral _ >
	struct D29
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::D29;
		static void connect() {}
	};
	template< Peripheral _ >
	struct D3
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::D3;
		static void connect() {}
	};
	template< Peripheral _ >
	struct D30
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::D30;
		static void connect() {}
	};
	template< Peripheral _ >
	struct D31
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::D31;
		static void connect() {}
	};
	template< Peripheral _ >
	struct D4
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::D4;
		static void connect() {}
	};
	template< Peripheral _ >
	struct D5
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::D5;
		static void connect() {}
	};
	template< Peripheral _ >
	struct D6
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::D6;
		static void connect() {}
	};
	template< Peripheral _ >
	struct D7
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::D7;
		static void connect() {}
	};
	template< Peripheral _ >
	struct D8
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::D8;
		static void connect() {}
	};
	template< Peripheral _ >
	struct D9
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::D9;
		static void connect() {}
	};
	template< Peripheral _ >
	struct Da0
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::Da0;
		static void connect() {}
	};
	template< Peripheral _ >
	struct Da1
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::Da1;
		static void connect() {}
	};
	template< Peripheral _ >
	struct Da10
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::Da10;
		static void connect() {}
	};
	template< Peripheral _ >
	struct Da11
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::Da11;
		static void connect() {}
	};
	template< Peripheral _ >
	struct Da12
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::Da12;
		static void connect() {}
	};
	template< Peripheral _ >
	struct Da13
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::Da13;
		static void connect() {}
	};
	template< Peripheral _ >
	struct Da14
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::Da14;
		static void connect() {}
	};
	template< Peripheral _ >
	struct Da15
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::Da15;
		static void connect() {}
	};
	template< Peripheral _ >
	struct Da2
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::Da2;
		static void connect() {}
	};
	template< Peripheral _ >
	struct Da3
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::Da3;
		static void connect() {}
	};
	template< Peripheral _ >
	struct Da4
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::Da4;
		static void connect() {}
	};
	template< Peripheral _ >
	struct Da5
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::Da5;
		static void connect() {}
	};
	template< Peripheral _ >
	struct Da6
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::Da6;
		static void connect() {}
	};
	template< Peripheral _ >
	struct Da7
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::Da7;
		static void connect() {}
	};
	template< Peripheral _ >
	struct Da8
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::Da8;
		static void connect() {}
	};
	template< Peripheral _ >
	struct Da9
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::Da9;
		static void connect() {}
	};
	template< Peripheral _ >
	struct Dm
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::Dm;
		static void connect() {}
	};
	template< Peripheral _ >
	struct Dp
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::Dp;
		static void connect() {}
	};
	template< Peripheral _ >
	struct Etr
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::Etr;
		static void connect() {}
	};
	template< Peripheral _ >
	struct ExtSd
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::ExtSd;
		static void connect() {}
	};
	template< Peripheral _ >
	struct FsA
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::FsA;
		static void connect() {}
	};
	template< Peripheral _ >
	struct FsB
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::FsB;
		static void connect() {}
	};
	template< Peripheral _ >
	struct Hsync
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::Hsync;
		static void connect() {}
	};
	template< Peripheral _ >
	struct Id
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::Id;
		static void connect() {}
	};
	template< Peripheral _ >
	struct In0
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::In0;
		static void connect() {}
	};
	template< Peripheral _ >
	struct In1
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::In1;
		static void connect() {}
	};
	template< Peripheral _ >
	struct In10
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::In10;
		static void connect() {}
	};
	template< Peripheral _ >
	struct In11
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::In11;
		static void connect() {}
	};
	template< Peripheral _ >
	struct In12
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::In12;
		static void connect() {}
	};
	template< Peripheral _ >
	struct In13
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::In13;
		static void connect() {}
	};
	template< Peripheral _ >
	struct In14
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::In14;
		static void connect() {}
	};
	template< Peripheral _ >
	struct In15
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::In15;
		static void connect() {}
	};
	template< Peripheral _ >
	struct In2
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::In2;
		static void connect() {}
	};
	template< Peripheral _ >
	struct In3
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::In3;
		static void connect() {}
	};
	template< Peripheral _ >
	struct In4
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::In4;
		static void connect() {}
	};
	template< Peripheral _ >
	struct In5
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::In5;
		static void connect() {}
	};
	template< Peripheral _ >
	struct In6
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::In6;
		static void connect() {}
	};
	template< Peripheral _ >
	struct In7
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::In7;
		static void connect() {}
	};
	template< Peripheral _ >
	struct In8
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::In8;
		static void connect() {}
	};
	template< Peripheral _ >
	struct In9
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::In9;
		static void connect() {}
	};
	template< Peripheral _ >
	struct Int2
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::Int2;
		static void connect() {}
	};
	template< Peripheral _ >
	struct Int3
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::Int3;
		static void connect() {}
	};
	template< Peripheral _ >
	struct Intr
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::Intr;
		static void connect() {}
	};
	template< Peripheral _ >
	struct Jtck
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::Jtck;
		static void connect() {}
	};
	template< Peripheral _ >
	struct Jtdi
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::Jtdi;
		static void connect() {}
	};
	template< Peripheral _ >
	struct Jtdo
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::Jtdo;
		static void connect() {}
	};
	template< Peripheral _ >
	struct Jtms
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::Jtms;
		static void connect() {}
	};
	template< Peripheral _ >
	struct Jtrst
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::Jtrst;
		static void connect() {}
	};
	template< Peripheral _ >
	struct Mck
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::Mck;
		static void connect() {}
	};
	template< Peripheral _ >
	struct MclkA
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::MclkA;
		static void connect() {}
	};
	template< Peripheral _ >
	struct MclkB
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::MclkB;
		static void connect() {}
	};
	template< Peripheral _ >
	struct Mco1
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::Mco1;
		static void connect() {}
	};
	template< Peripheral _ >
	struct Mco2
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::Mco2;
		static void connect() {}
	};
	template< Peripheral _ >
	struct Mdc
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::Mdc;
		static void connect() {}
	};
	template< Peripheral _ >
	struct Mdio
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::Mdio;
		static void connect() {}
	};
	template< Peripheral _ >
	struct Miso
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::Miso;
		static void connect() {}
	};
	template< Peripheral _ >
	struct Mosi
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::Mosi;
		static void connect() {}
	};
	template< Peripheral _ >
	struct Nbl0
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::Nbl0;
		static void connect() {}
	};
	template< Peripheral _ >
	struct Nbl1
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::Nbl1;
		static void connect() {}
	};
	template< Peripheral _ >
	struct Nbl2
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::Nbl2;
		static void connect() {}
	};
	template< Peripheral _ >
	struct Nbl3
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::Nbl3;
		static void connect() {}
	};
	template< Peripheral _ >
	struct Nce2
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::Nce2;
		static void connect() {}
	};
	template< Peripheral _ >
	struct Nce3
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::Nce3;
		static void connect() {}
	};
	template< Peripheral _ >
	struct Nce41
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::Nce41;
		static void connect() {}
	};
	template< Peripheral _ >
	struct Nce42
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::Nce42;
		static void connect() {}
	};
	template< Peripheral _ >
	struct Ne1
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::Ne1;
		static void connect() {}
	};
	template< Peripheral _ >
	struct Ne2
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::Ne2;
		static void connect() {}
	};
	template< Peripheral _ >
	struct Ne3
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::Ne3;
		static void connect() {}
	};
	template< Peripheral _ >
	struct Ne4
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::Ne4;
		static void connect() {}
	};
	template< Peripheral _ >
	struct Niord
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::Niord;
		static void connect() {}
	};
	template< Peripheral _ >
	struct Niowr
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::Niowr;
		static void connect() {}
	};
	template< Peripheral _ >
	struct Nl
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::Nl;
		static void connect() {}
	};
	template< Peripheral _ >
	struct Noe
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::Noe;
		static void connect() {}
	};
	template< Peripheral _ >
	struct Nreg
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::Nreg;
		static void connect() {}
	};
	template< Peripheral _ >
	struct Nss
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::Nss;
		static void connect() {}
	};
	template< Peripheral _ >
	struct Nwait
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::Nwait;
		static void connect() {}
	};
	template< Peripheral _ >
	struct Nwe
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::Nwe;
		static void connect() {}
	};
	template< Peripheral _ >
	struct Osc32In
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::Osc32In;
		static void connect() {}
	};
	template< Peripheral _ >
	struct Osc32Out
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::Osc32Out;
		static void connect() {}
	};
	template< Peripheral _ >
	struct OscIn
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::OscIn;
		static void connect() {}
	};
	template< Peripheral _ >
	struct OscOut
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::OscOut;
		static void connect() {}
	};
	template< Peripheral _ >
	struct Out1
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::Out1;
		static void connect() {}
	};
	template< Peripheral _ >
	struct Out2
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::Out2;
		static void connect() {}
	};
	template< Peripheral _ >
	struct Pixclk
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::Pixclk;
		static void connect() {}
	};
	template< Peripheral _ >
	struct PpsOut
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::PpsOut;
		static void connect() {}
	};
	template< Peripheral _ >
	struct RccCrsDv
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::RccCrsDv;
		static void connect() {}
	};
	template< Peripheral _ >
	struct RefClk
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::RefClk;
		static void connect() {}
	};
	template< Peripheral _ >
	struct Refin
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::Refin;
		static void connect() {}
	};
	template< Peripheral _ >
	struct Rts
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::Rts;
		static void connect() {}
	};
	template< Peripheral _ >
	struct Rx
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::Rx;
		static void connect() {}
	};
	template< Peripheral _ >
	struct RxClk
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::RxClk;
		static void connect() {}
	};
	template< Peripheral _ >
	struct RxDv
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::RxDv;
		static void connect() {}
	};
	template< Peripheral _ >
	struct RxEr
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::RxEr;
		static void connect() {}
	};
	template< Peripheral _ >
	struct Rxd0
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::Rxd0;
		static void connect() {}
	};
	template< Peripheral _ >
	struct Rxd1
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::Rxd1;
		static void connect() {}
	};
	template< Peripheral _ >
	struct Rxd2
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::Rxd2;
		static void connect() {}
	};
	template< Peripheral _ >
	struct Rxd3
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::Rxd3;
		static void connect() {}
	};
	template< Peripheral _ >
	struct Sck
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::Sck;
		static void connect() {}
	};
	template< Peripheral _ >
	struct SckA
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::SckA;
		static void connect() {}
	};
	template< Peripheral _ >
	struct SckB
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::SckB;
		static void connect() {}
	};
	template< Peripheral _ >
	struct Scl
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::Scl;
		static void connect() {}
	};
	template< Peripheral _ >
	struct Sd
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::Sd;
		static void connect() {}
	};
	template< Peripheral _ >
	struct SdA
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::SdA;
		static void connect() {}
	};
	template< Peripheral _ >
	struct SdB
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::SdB;
		static void connect() {}
	};
	template< Peripheral _ >
	struct Sda
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::Sda;
		static void connect() {}
	};
	template< Peripheral _ >
	struct Sdcke0
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::Sdcke0;
		static void connect() {}
	};
	template< Peripheral _ >
	struct Sdcke1
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::Sdcke1;
		static void connect() {}
	};
	template< Peripheral _ >
	struct Sdclk
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::Sdclk;
		static void connect() {}
	};
	template< Peripheral _ >
	struct Sdncas
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::Sdncas;
		static void connect() {}
	};
	template< Peripheral _ >
	struct Sdne0
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::Sdne0;
		static void connect() {}
	};
	template< Peripheral _ >
	struct Sdne1
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::Sdne1;
		static void connect() {}
	};
	template< Peripheral _ >
	struct Sdnras
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::Sdnras;
		static void connect() {}
	};
	template< Peripheral _ >
	struct Sdnwe
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::Sdnwe;
		static void connect() {}
	};
	template< Peripheral _ >
	struct Smba
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::Smba;
		static void connect() {}
	};
	template< Peripheral _ >
	struct Sof
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::Sof;
		static void connect() {}
	};
	template< Peripheral _ >
	struct Swclk
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::Swclk;
		static void connect() {}
	};
	template< Peripheral _ >
	struct Swdio
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::Swdio;
		static void connect() {}
	};
	template< Peripheral _ >
	struct Swo
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::Swo;
		static void connect() {}
	};
	template< Peripheral _ >
	struct Traceclk
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::Traceclk;
		static void connect() {}
	};
	template< Peripheral _ >
	struct Traced0
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::Traced0;
		static void connect() {}
	};
	template< Peripheral _ >
	struct Traced1
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::Traced1;
		static void connect() {}
	};
	template< Peripheral _ >
	struct Traced2
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::Traced2;
		static void connect() {}
	};
	template< Peripheral _ >
	struct Traced3
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::Traced3;
		static void connect() {}
	};
	template< Peripheral _ >
	struct Tx
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::Tx;
		static void connect() {}
	};
	template< Peripheral _ >
	struct TxClk
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::TxClk;
		static void connect() {}
	};
	template< Peripheral _ >
	struct TxEn
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::TxEn;
		static void connect() {}
	};
	template< Peripheral _ >
	struct Txd0
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::Txd0;
		static void connect() {}
	};
	template< Peripheral _ >
	struct Txd1
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::Txd1;
		static void connect() {}
	};
	template< Peripheral _ >
	struct Txd2
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::Txd2;
		static void connect() {}
	};
	template< Peripheral _ >
	struct Txd3
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::Txd3;
		static void connect() {}
	};
	template< Peripheral _ >
	struct UlpiCk
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::UlpiCk;
		static void connect() {}
	};
	template< Peripheral _ >
	struct UlpiD0
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::UlpiD0;
		static void connect() {}
	};
	template< Peripheral _ >
	struct UlpiD1
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::UlpiD1;
		static void connect() {}
	};
	template< Peripheral _ >
	struct UlpiD2
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::UlpiD2;
		static void connect() {}
	};
	template< Peripheral _ >
	struct UlpiD3
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::UlpiD3;
		static void connect() {}
	};
	template< Peripheral _ >
	struct UlpiD4
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::UlpiD4;
		static void connect() {}
	};
	template< Peripheral _ >
	struct UlpiD5
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::UlpiD5;
		static void connect() {}
	};
	template< Peripheral _ >
	struct UlpiD6
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::UlpiD6;
		static void connect() {}
	};
	template< Peripheral _ >
	struct UlpiD7
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::UlpiD7;
		static void connect() {}
	};
	template< Peripheral _ >
	struct UlpiDir
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::UlpiDir;
		static void connect() {}
	};
	template< Peripheral _ >
	struct UlpiNxt
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::UlpiNxt;
		static void connect() {}
	};
	template< Peripheral _ >
	struct UlpiStp
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::UlpiStp;
		static void connect() {}
	};
	template< Peripheral _ >
	struct Vbus
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::Vbus;
		static void connect() {}
	};
	template< Peripheral _ >
	struct Vsync
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::Vsync;
		static void connect() {}
	};
	template< Peripheral _ >
	struct Wkup
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::Wkup;
		static void connect() {}
	};
	template< Peripheral _ >
	struct Ws
	{
		using Gpio = GpioUnused;
		static constexpr Gpio::Signal Signal = Gpio::Signal::Ws;
		static void connect() {}
	};
	/// @endcond
};

} // namespace platform

} // namespace modm

#endif // MODM_STM32_GPIO_PIN_UNUSED_HPP