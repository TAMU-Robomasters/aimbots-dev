/*
 * Copyright (c) 2016-2018, Niklas Hauser
 *
 * This file is part of the modm project.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
// ----------------------------------------------------------------------------

#ifndef MODM_STM32_GPIO_BASE_HPP
#define MODM_STM32_GPIO_BASE_HPP

#include "../device.hpp"
#include <modm/architecture/interface/gpio.hpp>
#include <modm/math/utils/bit_operation.hpp>
#include <modm/platform/core/peripherals.hpp>

namespace modm::platform
{

/// @ingroup modm_platform_gpio
struct Gpio
{
	enum class
	InputType
	{
		Floating = 0x0,	///< floating on input
		PullUp = 0x1,	///< pull-up on input
		PullDown = 0x2,	///< pull-down on input
	};

	enum class
	OutputType
	{
		PushPull = 0x0,		///< push-pull on output
		OpenDrain = 0x1,	///< open-drain on output
	};

	enum class
	OutputSpeed
	{
		Low      = 0,
		Medium   = 0x1,
		High     = 0x2,
		VeryHigh = 0x3,		///< 30 pF (80 MHz Output max speed on 15 pF)
		MHz2     = Low,
		MHz25    = Medium,
		MHz50    = High,
		MHz100   = VeryHigh,
	};

	enum class
	InputTrigger
	{
		RisingEdge,
		FallingEdge,
		BothEdges,
	};

	/// The Port a Gpio Pin is connected to.
	enum class
	Port
	{
		A = 0,
		B = 1,
		C = 2,
		D = 3,
		E = 4,
		F = 5,
		G = 6,
		H = 7,
		I = 8,
	};

	/// @cond
	enum class
	Signal
	{
		BitBang,
		A0,
		A1,
		A10,
		A11,
		A12,
		A13,
		A14,
		A15,
		A16,
		A17,
		A18,
		A19,
		A2,
		A20,
		A21,
		A22,
		A23,
		A24,
		A25,
		A3,
		A4,
		A5,
		A6,
		A7,
		A8,
		A9,
		Af1,
		Af2,
		Ale,
		Ba0,
		Ba1,
		Bkin,
		Cd,
		Ch1,
		Ch1n,
		Ch2,
		Ch2n,
		Ch3,
		Ch3n,
		Ch4,
		Ck,
		Ckin,
		Cle,
		Clk,
		Cmd,
		Col,
		Crs,
		Cts,
		D0,
		D1,
		D10,
		D11,
		D12,
		D13,
		D14,
		D15,
		D16,
		D17,
		D18,
		D19,
		D2,
		D20,
		D21,
		D22,
		D23,
		D24,
		D25,
		D26,
		D27,
		D28,
		D29,
		D3,
		D30,
		D31,
		D4,
		D5,
		D6,
		D7,
		D8,
		D9,
		Da0,
		Da1,
		Da10,
		Da11,
		Da12,
		Da13,
		Da14,
		Da15,
		Da2,
		Da3,
		Da4,
		Da5,
		Da6,
		Da7,
		Da8,
		Da9,
		Dm,
		Dp,
		Etr,
		Extsd,
		Fsa,
		Fsb,
		Hsync,
		Id,
		In0,
		In1,
		In10,
		In11,
		In12,
		In13,
		In14,
		In15,
		In2,
		In3,
		In4,
		In5,
		In6,
		In7,
		In8,
		In9,
		Int2,
		Int3,
		Intr,
		Jtck,
		Jtdi,
		Jtdo,
		Jtms,
		Jtrst,
		Mck,
		Mclka,
		Mclkb,
		Mco1,
		Mco2,
		Mdc,
		Mdio,
		Miso,
		Mosi,
		Nbl0,
		Nbl1,
		Nbl2,
		Nbl3,
		Nce2,
		Nce3,
		Nce41,
		Nce42,
		Ne1,
		Ne2,
		Ne3,
		Ne4,
		Niord,
		Niowr,
		Nl,
		Noe,
		Nreg,
		Nss,
		Nwait,
		Nwe,
		Osc32in,
		Osc32out,
		Oscin,
		Oscout,
		Out1,
		Out2,
		Pixclk,
		Ppsout,
		Rcccrsdv,
		Refclk,
		Refin,
		Rts,
		Rx,
		Rxclk,
		Rxd0,
		Rxd1,
		Rxd2,
		Rxd3,
		Rxdv,
		Rxer,
		Sck,
		Scka,
		Sckb,
		Scl,
		Sd,
		Sda,
		Sdb,
		Sdcke0,
		Sdcke1,
		Sdclk,
		Sdncas,
		Sdne0,
		Sdne1,
		Sdnras,
		Sdnwe,
		Smba,
		Sof,
		Swclk,
		Swdio,
		Swo,
		Traceclk,
		Traced0,
		Traced1,
		Traced2,
		Traced3,
		Tx,
		Txclk,
		Txd0,
		Txd1,
		Txd2,
		Txd3,
		Txen,
		Ulpick,
		Ulpid0,
		Ulpid1,
		Ulpid2,
		Ulpid3,
		Ulpid4,
		Ulpid5,
		Ulpid6,
		Ulpid7,
		Ulpidir,
		Ulpinxt,
		Ulpistp,
		Vbus,
		Vsync,
		Wkup,
		Ws,
	};
	/// @endcond

protected:
	/// @cond
	/// I/O Direction Mode values for this specific pin.
	enum class
	Mode
	{
		Input  = 0x0,
		Output = 0x1,
		AlternateFunction = 0x2,
		Analog = 0x3,
		Mask   = 0x3,
	};

	static constexpr uint32_t
	i(Mode mode) { return uint32_t(mode); }
	// Enum Class To Integer helper functions.
	static constexpr uint32_t
	i(InputType pull) { return uint32_t(pull); }
	static constexpr uint32_t
	i(OutputType out) { return uint32_t(out); }
	static constexpr uint32_t
	i(OutputSpeed speed) { return uint32_t(speed); }
	/// @endcond
};

} // namespace modm::platform

#endif // MODM_STM32_GPIO_BASE_HPP