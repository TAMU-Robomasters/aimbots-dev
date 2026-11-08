/*
 * Copyright (c) 2017-2026, Niklas Hauser
 * Copyright (c) 2022, Andrey Kunitsyn
 *
 * This file is part of the modm project.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
// ----------------------------------------------------------------------------

#pragma once

#include "base.hpp"
#include "data.hpp"
#include "static.hpp"

namespace modm::platform
{

/// @ingroup modm_platform_gpio
using GpioUnused = GpioStatic<detail::DataUnused>;
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
template<>
class GpioStatic<detail::DataUnused> : public Gpio, public ::modm::GpioIO
{
public:
	using Output = GpioUnused;
	using Input = GpioUnused;
	using IO = GpioUnused;
	using Type = GpioUnused;
	using Data = detail::DataUnused;
	static constexpr bool isInverted = false;
	static constexpr Port port = Port(-1);
	static constexpr uint8_t pin = uint8_t(-1);
	static constexpr uint16_t mask = 0;
protected:
	static void setAlternateFunction(uint8_t) {}
	static void setAlternateFunction() {}
	static void setAnalogInput() {}
public:
	// GpioOutput
	static void setOutput() {}
	static void setOutput(bool) {}
	static void setOutput(OutputType, OutputSpeed = OutputSpeed::MHz50) {}
	static void configure(OutputType, OutputSpeed = OutputSpeed::MHz50) {}
	static void set() {}
	static void set(bool) {}
	static void reset() {}
	static bool isSet() { return false; }
	static void toggle() {}

	// GpioInput
	static void setInput() {}
	static void setInput(InputType) {}
	static void configure(InputType) {}
	static bool read() { return false; }

	// GpioIO
	static Direction getDirection() { return Direction::In; }
	static void lock() {}
	static void disconnect() {}

public:
	struct BitBang
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::BitBang;
	};
	struct A16
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::A16;
	};
	struct A17
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::A17;
	};
	struct A18
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::A18;
	};
	struct A19
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::A19;
	};
	struct A20
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::A20;
	};
	struct A21
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::A21;
	};
	struct A22
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::A22;
	};
	struct A23
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::A23;
	};
	struct A25
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::A25;
	};
	struct Ale
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Ale;
	};
	struct B0
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::B0;
	};
	struct B1
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::B1;
	};
	struct B2
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::B2;
	};
	struct B3
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::B3;
	};
	struct B4
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::B4;
	};
	struct B5
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::B5;
	};
	struct B6
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::B6;
	};
	struct B7
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::B7;
	};
	struct Bkin
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Bkin;
	};
	struct Bkin2
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Bkin2;
	};
	struct Bkin2comp1
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Bkin2comp1;
	};
	struct Bkin2comp2
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Bkin2comp2;
	};
	struct Bkincomp1
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Bkincomp1;
	};
	struct Bkincomp2
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Bkincomp2;
	};
	struct Cdir
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Cdir;
	};
	struct Cec
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Cec;
	};
	struct Ch1
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Ch1;
	};
	struct Ch1n
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Ch1n;
	};
	struct Ch2
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Ch2;
	};
	struct Ch2n
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Ch2n;
	};
	struct Ch3
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Ch3;
	};
	struct Ch3n
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Ch3n;
	};
	struct Ch4
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Ch4;
	};
	struct Ck
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Ck;
	};
	struct Ck1
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Ck1;
	};
	struct Ck2
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Ck2;
	};
	struct Ckin
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Ckin;
	};
	struct Ckin0
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Ckin0;
	};
	struct Ckin1
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Ckin1;
	};
	struct Ckin2
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Ckin2;
	};
	struct Ckin3
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Ckin3;
	};
	struct Ckin4
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Ckin4;
	};
	struct Ckin5
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Ckin5;
	};
	struct Ckin6
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Ckin6;
	};
	struct Ckin7
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Ckin7;
	};
	struct Ckout
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Ckout;
	};
	struct Cle
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Cle;
	};
	struct Clk
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Clk;
	};
	struct Cmd
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Cmd;
	};
	struct Col
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Col;
	};
	struct Crs
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Crs;
	};
	struct Crssync
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Crssync;
	};
	struct Csleep
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Csleep;
	};
	struct Cstop
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Cstop;
	};
	struct Cts
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Cts;
	};
	struct D0
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::D0;
	};
	struct D0dir
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::D0dir;
	};
	struct D1
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::D1;
	};
	struct D10
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::D10;
	};
	struct D11
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::D11;
	};
	struct D12
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::D12;
	};
	struct D123dir
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::D123dir;
	};
	struct D13
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::D13;
	};
	struct D14
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::D14;
	};
	struct D15
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::D15;
	};
	struct D2
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::D2;
	};
	struct D3
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::D3;
	};
	struct D4
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::D4;
	};
	struct D5
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::D5;
	};
	struct D6
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::D6;
	};
	struct D7
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::D7;
	};
	struct D8
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::D8;
	};
	struct D9
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::D9;
	};
	struct Da0
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Da0;
	};
	struct Da1
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Da1;
	};
	struct Da10
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Da10;
	};
	struct Da11
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Da11;
	};
	struct Da12
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Da12;
	};
	struct Da13
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Da13;
	};
	struct Da14
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Da14;
	};
	struct Da15
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Da15;
	};
	struct Da2
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Da2;
	};
	struct Da3
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Da3;
	};
	struct Da4
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Da4;
	};
	struct Da5
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Da5;
	};
	struct Da6
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Da6;
	};
	struct Da7
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Da7;
	};
	struct Da8
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Da8;
	};
	struct Da9
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Da9;
	};
	struct Datin0
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Datin0;
	};
	struct Datin1
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Datin1;
	};
	struct Datin2
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Datin2;
	};
	struct Datin3
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Datin3;
	};
	struct Datin4
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Datin4;
	};
	struct Datin5
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Datin5;
	};
	struct Datin6
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Datin6;
	};
	struct Datin7
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Datin7;
	};
	struct De
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::De;
	};
	struct Dm
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Dm;
	};
	struct Dp
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Dp;
	};
	struct Etr
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Etr;
	};
	struct Fsa
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Fsa;
	};
	struct Fsb
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Fsb;
	};
	struct G0
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::G0;
	};
	struct G1
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::G1;
	};
	struct G2
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::G2;
	};
	struct G3
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::G3;
	};
	struct G4
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::G4;
	};
	struct G5
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::G5;
	};
	struct G6
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::G6;
	};
	struct G7
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::G7;
	};
	struct Hsync
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Hsync;
	};
	struct Id
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Id;
	};
	struct In0
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::In0;
	};
	struct In1
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::In1;
	};
	struct In2
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::In2;
	};
	struct In3
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::In3;
	};
	struct Inm
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Inm;
	};
	struct Inn1
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Inn1;
	};
	struct Inn10
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Inn10;
	};
	struct Inn16
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Inn16;
	};
	struct Inn18
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Inn18;
	};
	struct Inn3
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Inn3;
	};
	struct Inn4
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Inn4;
	};
	struct Inn5
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Inn5;
	};
	struct Inp
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Inp;
	};
	struct Inp0
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Inp0;
	};
	struct Inp1
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Inp1;
	};
	struct Inp10
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Inp10;
	};
	struct Inp11
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Inp11;
	};
	struct Inp14
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Inp14;
	};
	struct Inp15
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Inp15;
	};
	struct Inp16
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Inp16;
	};
	struct Inp17
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Inp17;
	};
	struct Inp18
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Inp18;
	};
	struct Inp19
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Inp19;
	};
	struct Inp3
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Inp3;
	};
	struct Inp4
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Inp4;
	};
	struct Inp5
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Inp5;
	};
	struct Inp7
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Inp7;
	};
	struct Inp8
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Inp8;
	};
	struct Inp9
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Inp9;
	};
	struct Int
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Int;
	};
	struct Io
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Io;
	};
	struct Jtck
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Jtck;
	};
	struct Jtdi
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Jtdi;
	};
	struct Jtdo
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Jtdo;
	};
	struct Jtms
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Jtms;
	};
	struct Jtrst
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Jtrst;
	};
	struct Mck
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Mck;
	};
	struct Mclka
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Mclka;
	};
	struct Mclkb
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Mclkb;
	};
	struct Mco1
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Mco1;
	};
	struct Mco2
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Mco2;
	};
	struct Mdc
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Mdc;
	};
	struct Mdio
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Mdio;
	};
	struct Miso
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Miso;
	};
	struct Mosi
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Mosi;
	};
	struct Nbl0
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Nbl0;
	};
	struct Nbl1
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Nbl1;
	};
	struct Nce
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Nce;
	};
	struct Ndstop2
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Ndstop2;
	};
	struct Ne1
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Ne1;
	};
	struct Ne2
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Ne2;
	};
	struct Nl
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Nl;
	};
	struct Noe
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Noe;
	};
	struct Nss
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Nss;
	};
	struct Nwait
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Nwait;
	};
	struct Nwe
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Nwe;
	};
	struct Osc32in
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Osc32in;
	};
	struct Osc32out
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Osc32out;
	};
	struct Oscin
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Oscin;
	};
	struct Oscout
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Oscout;
	};
	struct Out
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Out;
	};
	struct Out1
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Out1;
	};
	struct Out2
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Out2;
	};
	struct Outalarm
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Outalarm;
	};
	struct Outcalib
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Outcalib;
	};
	struct P1clk
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::P1clk;
	};
	struct P1dqs
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::P1dqs;
	};
	struct P1io0
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::P1io0;
	};
	struct P1io1
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::P1io1;
	};
	struct P1io2
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::P1io2;
	};
	struct P1io3
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::P1io3;
	};
	struct P1io4
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::P1io4;
	};
	struct P1io5
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::P1io5;
	};
	struct P1io6
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::P1io6;
	};
	struct P1io7
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::P1io7;
	};
	struct P1nclk
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::P1nclk;
	};
	struct P1ncs
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::P1ncs;
	};
	struct Pdck
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Pdck;
	};
	struct Pixclk
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Pixclk;
	};
	struct Ppsout
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Ppsout;
	};
	struct Pvdin
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Pvdin;
	};
	struct R0
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::R0;
	};
	struct R1
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::R1;
	};
	struct R2
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::R2;
	};
	struct R3
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::R3;
	};
	struct R4
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::R4;
	};
	struct R5
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::R5;
	};
	struct R6
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::R6;
	};
	struct R7
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::R7;
	};
	struct Rcccrsdv
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Rcccrsdv;
	};
	struct Rdy
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Rdy;
	};
	struct Refclk
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Refclk;
	};
	struct Refin
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Refin;
	};
	struct Rts
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Rts;
	};
	struct Rx
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Rx;
	};
	struct Rxclk
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Rxclk;
	};
	struct Rxd0
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Rxd0;
	};
	struct Rxd1
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Rxd1;
	};
	struct Rxd2
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Rxd2;
	};
	struct Rxd3
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Rxd3;
	};
	struct Rxdv
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Rxdv;
	};
	struct Rxer
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Rxer;
	};
	struct Sck
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Sck;
	};
	struct Scka
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Scka;
	};
	struct Sckb
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Sckb;
	};
	struct Scl
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Scl;
	};
	struct Sda
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Sda;
	};
	struct Sdb
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Sdb;
	};
	struct Sdcke0
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Sdcke0;
	};
	struct Sdcke1
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Sdcke1;
	};
	struct Sdi
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Sdi;
	};
	struct Sdne0
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Sdne0;
	};
	struct Sdne1
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Sdne1;
	};
	struct Sdnwe
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Sdnwe;
	};
	struct Sdo
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Sdo;
	};
	struct Smba
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Smba;
	};
	struct Sof
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Sof;
	};
	struct Suspend
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Suspend;
	};
	struct Swclk
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Swclk;
	};
	struct Swdio
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Swdio;
	};
	struct Swo
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Swo;
	};
	struct Tamp1
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Tamp1;
	};
	struct Tamp3
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Tamp3;
	};
	struct Traceclk
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Traceclk;
	};
	struct Traced0
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Traced0;
	};
	struct Traced1
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Traced1;
	};
	struct Traced2
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Traced2;
	};
	struct Traced3
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Traced3;
	};
	struct Trgio
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Trgio;
	};
	struct Ts
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Ts;
	};
	struct Tx
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Tx;
	};
	struct Txclk
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Txclk;
	};
	struct Txd0
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Txd0;
	};
	struct Txd1
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Txd1;
	};
	struct Txd2
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Txd2;
	};
	struct Txd3
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Txd3;
	};
	struct Txen
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Txen;
	};
	struct Txer
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Txer;
	};
	struct Ulpick
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Ulpick;
	};
	struct Ulpid0
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Ulpid0;
	};
	struct Ulpid1
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Ulpid1;
	};
	struct Ulpid2
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Ulpid2;
	};
	struct Ulpid3
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Ulpid3;
	};
	struct Ulpid4
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Ulpid4;
	};
	struct Ulpid5
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Ulpid5;
	};
	struct Ulpid6
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Ulpid6;
	};
	struct Ulpid7
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Ulpid7;
	};
	struct Ulpidir
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Ulpidir;
	};
	struct Ulpinxt
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Ulpinxt;
	};
	struct Ulpistp
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Ulpistp;
	};
	struct Vbus
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Vbus;
	};
	struct Vinm
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Vinm;
	};
	struct Vinp
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Vinp;
	};
	struct Vout
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Vout;
	};
	struct Vsync
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Vsync;
	};
	struct Wkup1
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Wkup1;
	};
	struct Wkup2
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Wkup2;
	};
	struct Wkup4
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Wkup4;
	};
	struct Wkup6
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Wkup6;
	};
	struct Ws
	{
		using Data = detail::DataUnused;
		static constexpr platform::Gpio::Signal Signal = platform::Gpio::Signal::Ws;
	};
};

} // namespace modm::platform