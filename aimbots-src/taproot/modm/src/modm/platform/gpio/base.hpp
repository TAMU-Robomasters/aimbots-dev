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

	/// The Port a Gpio Pin is connected to.
	enum class
	Port
	{
		A = 0,
		B = 1,
		C = 2,
		D = 3,
		E = 4,
		H = 7,
	};

	/// @cond
	enum class
	Signal
	{
		BitBang,
		A16,
		A17,
		A18,
		A19,
		A20,
		A21,
		A22,
		A23,
		A25,
		Ale,
		B0,
		B1,
		B2,
		B3,
		B4,
		B5,
		B6,
		B7,
		Bkin,
		Bkin2,
		Bkin2comp1,
		Bkin2comp2,
		Bkincomp1,
		Bkincomp2,
		Cdir,
		Cec,
		Ch1,
		Ch1n,
		Ch2,
		Ch2n,
		Ch3,
		Ch3n,
		Ch4,
		Ck,
		Ck1,
		Ck2,
		Ckin,
		Ckin0,
		Ckin1,
		Ckin2,
		Ckin3,
		Ckin4,
		Ckin5,
		Ckin6,
		Ckin7,
		Ckout,
		Cle,
		Clk,
		Cmd,
		Col,
		Crs,
		Crssync,
		Csleep,
		Cstop,
		Cts,
		D0,
		D0dir,
		D1,
		D10,
		D11,
		D12,
		D123dir,
		D13,
		D14,
		D15,
		D2,
		D3,
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
		Datin0,
		Datin1,
		Datin2,
		Datin3,
		Datin4,
		Datin5,
		Datin6,
		Datin7,
		De,
		Dm,
		Dp,
		Etr,
		Fsa,
		Fsb,
		G0,
		G1,
		G2,
		G3,
		G4,
		G5,
		G6,
		G7,
		Hsync,
		Id,
		In0,
		In1,
		In2,
		In3,
		Inm,
		Inn1,
		Inn10,
		Inn16,
		Inn18,
		Inn3,
		Inn4,
		Inn5,
		Inp,
		Inp0,
		Inp1,
		Inp10,
		Inp11,
		Inp14,
		Inp15,
		Inp16,
		Inp17,
		Inp18,
		Inp19,
		Inp3,
		Inp4,
		Inp5,
		Inp7,
		Inp8,
		Inp9,
		Int,
		Io,
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
		Nce,
		Ndstop2,
		Ne1,
		Ne2,
		Nl,
		Noe,
		Nss,
		Nwait,
		Nwe,
		Osc32in,
		Osc32out,
		Oscin,
		Oscout,
		Out,
		Out1,
		Out2,
		Outalarm,
		Outcalib,
		P1clk,
		P1dqs,
		P1io0,
		P1io1,
		P1io2,
		P1io3,
		P1io4,
		P1io5,
		P1io6,
		P1io7,
		P1nclk,
		P1ncs,
		Pdck,
		Pixclk,
		Ppsout,
		Pvdin,
		R0,
		R1,
		R2,
		R3,
		R4,
		R5,
		R6,
		R7,
		Rcccrsdv,
		Rdy,
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
		Sda,
		Sdb,
		Sdcke0,
		Sdcke1,
		Sdi,
		Sdne0,
		Sdne1,
		Sdnwe,
		Sdo,
		Smba,
		Sof,
		Suspend,
		Swclk,
		Swdio,
		Swo,
		Tamp1,
		Tamp3,
		Traceclk,
		Traced0,
		Traced1,
		Traced2,
		Traced3,
		Trgio,
		Ts,
		Tx,
		Txclk,
		Txd0,
		Txd1,
		Txd2,
		Txd3,
		Txen,
		Txer,
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
		Vinm,
		Vinp,
		Vout,
		Vsync,
		Wkup1,
		Wkup2,
		Wkup4,
		Wkup6,
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