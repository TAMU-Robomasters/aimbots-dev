/*
 * Copyright (c) 2021, Niklas Hauser
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

/// @cond
namespace modm::platform::detail
{

enum class
AdcPolarity
{
	Positive,
	Negative
};
template<class Signal, Peripheral p> struct SignalConnection;
template<class Data, Peripheral p, AdcPolarity ap> constexpr int8_t AdcChannel = -1;
template<class Data, Peripheral p> constexpr int8_t DacChannel = -1;
struct DataUnused {};

struct DataA0 {
	static constexpr Gpio::Port port = Gpio::Port::A;
	static constexpr uint8_t pin = 0;
	struct BitBang { using Data = DataA0; static constexpr Gpio::Signal Signal = Gpio::Signal::BitBang; };
	struct A19 { using Data = DataA0; static constexpr Gpio::Signal Signal = Gpio::Signal::A19; };
	struct Bkin { using Data = DataA0; static constexpr Gpio::Signal Signal = Gpio::Signal::Bkin; };
	struct Ch1 { using Data = DataA0; static constexpr Gpio::Signal Signal = Gpio::Signal::Ch1; };
	struct Cmd { using Data = DataA0; static constexpr Gpio::Signal Signal = Gpio::Signal::Cmd; };
	struct Crs { using Data = DataA0; static constexpr Gpio::Signal Signal = Gpio::Signal::Crs; };
	struct Cts { using Data = DataA0; static constexpr Gpio::Signal Signal = Gpio::Signal::Cts; };
	struct Etr { using Data = DataA0; static constexpr Gpio::Signal Signal = Gpio::Signal::Etr; };
	struct Inp16 { using Data = DataA0; static constexpr Gpio::Signal Signal = Gpio::Signal::Inp16; };
	struct Nss { using Data = DataA0; static constexpr Gpio::Signal Signal = Gpio::Signal::Nss; };
	struct Sdb { using Data = DataA0; static constexpr Gpio::Signal Signal = Gpio::Signal::Sdb; };
	struct Tx { using Data = DataA0; static constexpr Gpio::Signal Signal = Gpio::Signal::Tx; };
	struct Wkup1 { using Data = DataA0; static constexpr Gpio::Signal Signal = Gpio::Signal::Wkup1; };
	struct Ws { using Data = DataA0; static constexpr Gpio::Signal Signal = Gpio::Signal::Ws; };
};
template<Peripheral p> struct SignalConnection<DataA0::BitBang, p> {
	static_assert(p == Peripheral::BitBang, "GpioA0::BitBang only connects to software drivers!"); };
template<Peripheral p> struct SignalConnection<DataA0::A19, p> {
	static_assert((p == Peripheral::Fmc),"GpioA0::A19 only connects to Fmc!"); };
template<Peripheral p> struct SignalConnection<DataA0::Bkin, p> {
	static_assert((p == Peripheral::Tim15),"GpioA0::Bkin only connects to Tim15!"); };
template<Peripheral p> struct SignalConnection<DataA0::Ch1, p> {
	static_assert((p == Peripheral::Tim2) or (p == Peripheral::Tim5),
		"GpioA0::Ch1 only connects to Tim2 or Tim5!"); };
template<Peripheral p> struct SignalConnection<DataA0::Cmd, p> {
	static_assert((p == Peripheral::Sdmmc2),"GpioA0::Cmd only connects to Sdmmc2!"); };
template<Peripheral p> struct SignalConnection<DataA0::Crs, p> {
	static_assert((p == Peripheral::Eth),"GpioA0::Crs only connects to Eth!"); };
template<Peripheral p> struct SignalConnection<DataA0::Cts, p> {
	static_assert((p == Peripheral::Usart2),"GpioA0::Cts only connects to Usart2!"); };
template<Peripheral p> struct SignalConnection<DataA0::Etr, p> {
	static_assert((p == Peripheral::Tim2) or (p == Peripheral::Tim8),
		"GpioA0::Etr only connects to Tim2 or Tim8!"); };
template<Peripheral p> struct SignalConnection<DataA0::Inp16, p> {
	static_assert((p == Peripheral::Adc1),"GpioA0::Inp16 only connects to Adc1!"); };
template<Peripheral p> struct SignalConnection<DataA0::Nss, p> {
	static_assert((p == Peripheral::Spi6) or (p == Peripheral::Usart2),
		"GpioA0::Nss only connects to Spi6 or Usart2!"); };
template<Peripheral p> struct SignalConnection<DataA0::Sdb, p> {
	static_assert((p == Peripheral::Sai4),"GpioA0::Sdb only connects to Sai4!"); };
template<Peripheral p> struct SignalConnection<DataA0::Tx, p> {
	static_assert((p == Peripheral::Uart4),"GpioA0::Tx only connects to Uart4!"); };
template<Peripheral p> struct SignalConnection<DataA0::Wkup1, p> {
	static_assert((p == Peripheral::Pwr),"GpioA0::Wkup1 only connects to Pwr!"); };
template<Peripheral p> struct SignalConnection<DataA0::Ws, p> {
	static_assert((p == Peripheral::I2s6),"GpioA0::Ws only connects to I2s6!"); };
template<> struct SignalConnection<DataA0::BitBang, Peripheral::BitBang> { static constexpr int8_t af = -1; };
template<> struct SignalConnection<DataA0::A19, Peripheral::Fmc> { static constexpr int8_t af = 12; };
template<> struct SignalConnection<DataA0::Bkin, Peripheral::Tim15> { static constexpr int8_t af = 4; };
template<> struct SignalConnection<DataA0::Ch1, Peripheral::Tim2> { static constexpr int8_t af = 1; };
template<> struct SignalConnection<DataA0::Ch1, Peripheral::Tim5> { static constexpr int8_t af = 2; };
template<> struct SignalConnection<DataA0::Cmd, Peripheral::Sdmmc2> { static constexpr int8_t af = 9; };
template<> struct SignalConnection<DataA0::Crs, Peripheral::Eth> { static constexpr int8_t af = 11; };
template<> struct SignalConnection<DataA0::Cts, Peripheral::Usart2> { static constexpr int8_t af = 7; };
template<> struct SignalConnection<DataA0::Etr, Peripheral::Tim2> { static constexpr int8_t af = 1; };
template<> struct SignalConnection<DataA0::Etr, Peripheral::Tim8> { static constexpr int8_t af = 3; };
template<> struct SignalConnection<DataA0::Inp16, Peripheral::Adc1> { static constexpr int8_t af = -2; };
template<> struct SignalConnection<DataA0::Nss, Peripheral::Spi6> { static constexpr int8_t af = 5; };
template<> struct SignalConnection<DataA0::Nss, Peripheral::Usart2> { static constexpr int8_t af = 7; };
template<> struct SignalConnection<DataA0::Sdb, Peripheral::Sai4> { static constexpr int8_t af = 10; };
template<> struct SignalConnection<DataA0::Tx, Peripheral::Uart4> { static constexpr int8_t af = 8; };
template<> struct SignalConnection<DataA0::Wkup1, Peripheral::Pwr> { static constexpr int8_t af = -1; };
template<> struct SignalConnection<DataA0::Ws, Peripheral::I2s6> { static constexpr int8_t af = 5; };
template<> constexpr inline int8_t AdcChannel<DataA0, Peripheral::Adc1, AdcPolarity::Positive> = 16;

struct DataA1 {
	static constexpr Gpio::Port port = Gpio::Port::A;
	static constexpr uint8_t pin = 1;
	struct BitBang { using Data = DataA1; static constexpr Gpio::Signal Signal = Gpio::Signal::BitBang; };
	struct Ch1n { using Data = DataA1; static constexpr Gpio::Signal Signal = Gpio::Signal::Ch1n; };
	struct Ch2 { using Data = DataA1; static constexpr Gpio::Signal Signal = Gpio::Signal::Ch2; };
	struct De { using Data = DataA1; static constexpr Gpio::Signal Signal = Gpio::Signal::De; };
	struct Inn16 { using Data = DataA1; static constexpr Gpio::Signal Signal = Gpio::Signal::Inn16; };
	struct Inp17 { using Data = DataA1; static constexpr Gpio::Signal Signal = Gpio::Signal::Inp17; };
	struct Mclkb { using Data = DataA1; static constexpr Gpio::Signal Signal = Gpio::Signal::Mclkb; };
	struct Out { using Data = DataA1; static constexpr Gpio::Signal Signal = Gpio::Signal::Out; };
	struct P1dqs { using Data = DataA1; static constexpr Gpio::Signal Signal = Gpio::Signal::P1dqs; };
	struct P1io3 { using Data = DataA1; static constexpr Gpio::Signal Signal = Gpio::Signal::P1io3; };
	struct R2 { using Data = DataA1; static constexpr Gpio::Signal Signal = Gpio::Signal::R2; };
	struct Refclk { using Data = DataA1; static constexpr Gpio::Signal Signal = Gpio::Signal::Refclk; };
	struct Rts { using Data = DataA1; static constexpr Gpio::Signal Signal = Gpio::Signal::Rts; };
	struct Rx { using Data = DataA1; static constexpr Gpio::Signal Signal = Gpio::Signal::Rx; };
	struct Rxclk { using Data = DataA1; static constexpr Gpio::Signal Signal = Gpio::Signal::Rxclk; };
};
template<Peripheral p> struct SignalConnection<DataA1::BitBang, p> {
	static_assert(p == Peripheral::BitBang, "GpioA1::BitBang only connects to software drivers!"); };
template<Peripheral p> struct SignalConnection<DataA1::Ch1n, p> {
	static_assert((p == Peripheral::Tim15),"GpioA1::Ch1n only connects to Tim15!"); };
template<Peripheral p> struct SignalConnection<DataA1::Ch2, p> {
	static_assert((p == Peripheral::Tim2) or (p == Peripheral::Tim5),
		"GpioA1::Ch2 only connects to Tim2 or Tim5!"); };
template<Peripheral p> struct SignalConnection<DataA1::De, p> {
	static_assert((p == Peripheral::Usart2),"GpioA1::De only connects to Usart2!"); };
template<Peripheral p> struct SignalConnection<DataA1::Inn16, p> {
	static_assert((p == Peripheral::Adc1),"GpioA1::Inn16 only connects to Adc1!"); };
template<Peripheral p> struct SignalConnection<DataA1::Inp17, p> {
	static_assert((p == Peripheral::Adc1),"GpioA1::Inp17 only connects to Adc1!"); };
template<Peripheral p> struct SignalConnection<DataA1::Mclkb, p> {
	static_assert((p == Peripheral::Sai4),"GpioA1::Mclkb only connects to Sai4!"); };
template<Peripheral p> struct SignalConnection<DataA1::Out, p> {
	static_assert((p == Peripheral::Lptim3),"GpioA1::Out only connects to Lptim3!"); };
template<Peripheral p> struct SignalConnection<DataA1::P1dqs, p> {
	static_assert((p == Peripheral::Octospim),"GpioA1::P1dqs only connects to Octospim!"); };
template<Peripheral p> struct SignalConnection<DataA1::P1io3, p> {
	static_assert((p == Peripheral::Octospim),"GpioA1::P1io3 only connects to Octospim!"); };
template<Peripheral p> struct SignalConnection<DataA1::R2, p> {
	static_assert((p == Peripheral::Ltdc),"GpioA1::R2 only connects to Ltdc!"); };
template<Peripheral p> struct SignalConnection<DataA1::Refclk, p> {
	static_assert((p == Peripheral::Eth),"GpioA1::Refclk only connects to Eth!"); };
template<Peripheral p> struct SignalConnection<DataA1::Rts, p> {
	static_assert((p == Peripheral::Usart2),"GpioA1::Rts only connects to Usart2!"); };
template<Peripheral p> struct SignalConnection<DataA1::Rx, p> {
	static_assert((p == Peripheral::Uart4),"GpioA1::Rx only connects to Uart4!"); };
template<Peripheral p> struct SignalConnection<DataA1::Rxclk, p> {
	static_assert((p == Peripheral::Eth),"GpioA1::Rxclk only connects to Eth!"); };
template<> struct SignalConnection<DataA1::BitBang, Peripheral::BitBang> { static constexpr int8_t af = -1; };
template<> struct SignalConnection<DataA1::Ch1n, Peripheral::Tim15> { static constexpr int8_t af = 4; };
template<> struct SignalConnection<DataA1::Ch2, Peripheral::Tim2> { static constexpr int8_t af = 1; };
template<> struct SignalConnection<DataA1::Ch2, Peripheral::Tim5> { static constexpr int8_t af = 2; };
template<> struct SignalConnection<DataA1::De, Peripheral::Usart2> { static constexpr int8_t af = 7; };
template<> struct SignalConnection<DataA1::Inn16, Peripheral::Adc1> { static constexpr int8_t af = -2; };
template<> struct SignalConnection<DataA1::Inp17, Peripheral::Adc1> { static constexpr int8_t af = -2; };
template<> struct SignalConnection<DataA1::Mclkb, Peripheral::Sai4> { static constexpr int8_t af = 10; };
template<> struct SignalConnection<DataA1::Out, Peripheral::Lptim3> { static constexpr int8_t af = 3; };
template<> struct SignalConnection<DataA1::P1dqs, Peripheral::Octospim> { static constexpr int8_t af = 12; };
template<> struct SignalConnection<DataA1::P1io3, Peripheral::Octospim> { static constexpr int8_t af = 9; };
template<> struct SignalConnection<DataA1::R2, Peripheral::Ltdc> { static constexpr int8_t af = 14; };
template<> struct SignalConnection<DataA1::Refclk, Peripheral::Eth> { static constexpr int8_t af = 11; };
template<> struct SignalConnection<DataA1::Rts, Peripheral::Usart2> { static constexpr int8_t af = 7; };
template<> struct SignalConnection<DataA1::Rx, Peripheral::Uart4> { static constexpr int8_t af = 8; };
template<> struct SignalConnection<DataA1::Rxclk, Peripheral::Eth> { static constexpr int8_t af = 11; };
template<> constexpr inline int8_t AdcChannel<DataA1, Peripheral::Adc1, AdcPolarity::Negative> = 16;
template<> constexpr inline int8_t AdcChannel<DataA1, Peripheral::Adc1, AdcPolarity::Positive> = 17;

struct DataA2 {
	static constexpr Gpio::Port port = Gpio::Port::A;
	static constexpr uint8_t pin = 2;
	struct BitBang { using Data = DataA2; static constexpr Gpio::Signal Signal = Gpio::Signal::BitBang; };
	struct Ch1 { using Data = DataA2; static constexpr Gpio::Signal Signal = Gpio::Signal::Ch1; };
	struct Ch3 { using Data = DataA2; static constexpr Gpio::Signal Signal = Gpio::Signal::Ch3; };
	struct Inp14 { using Data = DataA2; static constexpr Gpio::Signal Signal = Gpio::Signal::Inp14; };
	struct Mdio { using Data = DataA2; static constexpr Gpio::Signal Signal = Gpio::Signal::Mdio; };
	struct Out { using Data = DataA2; static constexpr Gpio::Signal Signal = Gpio::Signal::Out; };
	struct P1io0 { using Data = DataA2; static constexpr Gpio::Signal Signal = Gpio::Signal::P1io0; };
	struct R1 { using Data = DataA2; static constexpr Gpio::Signal Signal = Gpio::Signal::R1; };
	struct Sckb { using Data = DataA2; static constexpr Gpio::Signal Signal = Gpio::Signal::Sckb; };
	struct Tx { using Data = DataA2; static constexpr Gpio::Signal Signal = Gpio::Signal::Tx; };
	struct Wkup2 { using Data = DataA2; static constexpr Gpio::Signal Signal = Gpio::Signal::Wkup2; };
};
template<Peripheral p> struct SignalConnection<DataA2::BitBang, p> {
	static_assert(p == Peripheral::BitBang, "GpioA2::BitBang only connects to software drivers!"); };
template<Peripheral p> struct SignalConnection<DataA2::Ch1, p> {
	static_assert((p == Peripheral::Tim15),"GpioA2::Ch1 only connects to Tim15!"); };
template<Peripheral p> struct SignalConnection<DataA2::Ch3, p> {
	static_assert((p == Peripheral::Tim2) or (p == Peripheral::Tim5),
		"GpioA2::Ch3 only connects to Tim2 or Tim5!"); };
template<Peripheral p> struct SignalConnection<DataA2::Inp14, p> {
	static_assert((p == Peripheral::Adc1) or (p == Peripheral::Adc2),
		"GpioA2::Inp14 only connects to Adc1 or Adc2!"); };
template<Peripheral p> struct SignalConnection<DataA2::Mdio, p> {
	static_assert((p == Peripheral::Eth) or (p == Peripheral::Mdios),
		"GpioA2::Mdio only connects to Eth or Mdios!"); };
template<Peripheral p> struct SignalConnection<DataA2::Out, p> {
	static_assert((p == Peripheral::Lptim4),"GpioA2::Out only connects to Lptim4!"); };
template<Peripheral p> struct SignalConnection<DataA2::P1io0, p> {
	static_assert((p == Peripheral::Octospim),"GpioA2::P1io0 only connects to Octospim!"); };
template<Peripheral p> struct SignalConnection<DataA2::R1, p> {
	static_assert((p == Peripheral::Ltdc),"GpioA2::R1 only connects to Ltdc!"); };
template<Peripheral p> struct SignalConnection<DataA2::Sckb, p> {
	static_assert((p == Peripheral::Sai4),"GpioA2::Sckb only connects to Sai4!"); };
template<Peripheral p> struct SignalConnection<DataA2::Tx, p> {
	static_assert((p == Peripheral::Usart2),"GpioA2::Tx only connects to Usart2!"); };
template<Peripheral p> struct SignalConnection<DataA2::Wkup2, p> {
	static_assert((p == Peripheral::Pwr),"GpioA2::Wkup2 only connects to Pwr!"); };
template<> struct SignalConnection<DataA2::BitBang, Peripheral::BitBang> { static constexpr int8_t af = -1; };
template<> struct SignalConnection<DataA2::Ch1, Peripheral::Tim15> { static constexpr int8_t af = 4; };
template<> struct SignalConnection<DataA2::Ch3, Peripheral::Tim2> { static constexpr int8_t af = 1; };
template<> struct SignalConnection<DataA2::Ch3, Peripheral::Tim5> { static constexpr int8_t af = 2; };
template<> struct SignalConnection<DataA2::Inp14, Peripheral::Adc1> { static constexpr int8_t af = -2; };
template<> struct SignalConnection<DataA2::Inp14, Peripheral::Adc2> { static constexpr int8_t af = -2; };
template<> struct SignalConnection<DataA2::Mdio, Peripheral::Eth> { static constexpr int8_t af = 11; };
template<> struct SignalConnection<DataA2::Mdio, Peripheral::Mdios> { static constexpr int8_t af = 12; };
template<> struct SignalConnection<DataA2::Out, Peripheral::Lptim4> { static constexpr int8_t af = 3; };
template<> struct SignalConnection<DataA2::P1io0, Peripheral::Octospim> { static constexpr int8_t af = 6; };
template<> struct SignalConnection<DataA2::R1, Peripheral::Ltdc> { static constexpr int8_t af = 14; };
template<> struct SignalConnection<DataA2::Sckb, Peripheral::Sai4> { static constexpr int8_t af = 8; };
template<> struct SignalConnection<DataA2::Tx, Peripheral::Usart2> { static constexpr int8_t af = 7; };
template<> struct SignalConnection<DataA2::Wkup2, Peripheral::Pwr> { static constexpr int8_t af = -1; };
template<> constexpr inline int8_t AdcChannel<DataA2, Peripheral::Adc1, AdcPolarity::Positive> = 14;
template<> constexpr inline int8_t AdcChannel<DataA2, Peripheral::Adc2, AdcPolarity::Positive> = 14;

struct DataA3 {
	static constexpr Gpio::Port port = Gpio::Port::A;
	static constexpr uint8_t pin = 3;
	struct BitBang { using Data = DataA3; static constexpr Gpio::Signal Signal = Gpio::Signal::BitBang; };
	struct B2 { using Data = DataA3; static constexpr Gpio::Signal Signal = Gpio::Signal::B2; };
	struct B5 { using Data = DataA3; static constexpr Gpio::Signal Signal = Gpio::Signal::B5; };
	struct Ch2 { using Data = DataA3; static constexpr Gpio::Signal Signal = Gpio::Signal::Ch2; };
	struct Ch4 { using Data = DataA3; static constexpr Gpio::Signal Signal = Gpio::Signal::Ch4; };
	struct Col { using Data = DataA3; static constexpr Gpio::Signal Signal = Gpio::Signal::Col; };
	struct Inp15 { using Data = DataA3; static constexpr Gpio::Signal Signal = Gpio::Signal::Inp15; };
	struct Mck { using Data = DataA3; static constexpr Gpio::Signal Signal = Gpio::Signal::Mck; };
	struct Out { using Data = DataA3; static constexpr Gpio::Signal Signal = Gpio::Signal::Out; };
	struct P1clk { using Data = DataA3; static constexpr Gpio::Signal Signal = Gpio::Signal::P1clk; };
	struct P1io2 { using Data = DataA3; static constexpr Gpio::Signal Signal = Gpio::Signal::P1io2; };
	struct Rx { using Data = DataA3; static constexpr Gpio::Signal Signal = Gpio::Signal::Rx; };
	struct Ulpid0 { using Data = DataA3; static constexpr Gpio::Signal Signal = Gpio::Signal::Ulpid0; };
};
template<Peripheral p> struct SignalConnection<DataA3::BitBang, p> {
	static_assert(p == Peripheral::BitBang, "GpioA3::BitBang only connects to software drivers!"); };
template<Peripheral p> struct SignalConnection<DataA3::B2, p> {
	static_assert((p == Peripheral::Ltdc),"GpioA3::B2 only connects to Ltdc!"); };
template<Peripheral p> struct SignalConnection<DataA3::B5, p> {
	static_assert((p == Peripheral::Ltdc),"GpioA3::B5 only connects to Ltdc!"); };
template<Peripheral p> struct SignalConnection<DataA3::Ch2, p> {
	static_assert((p == Peripheral::Tim15),"GpioA3::Ch2 only connects to Tim15!"); };
template<Peripheral p> struct SignalConnection<DataA3::Ch4, p> {
	static_assert((p == Peripheral::Tim2) or (p == Peripheral::Tim5),
		"GpioA3::Ch4 only connects to Tim2 or Tim5!"); };
template<Peripheral p> struct SignalConnection<DataA3::Col, p> {
	static_assert((p == Peripheral::Eth),"GpioA3::Col only connects to Eth!"); };
template<Peripheral p> struct SignalConnection<DataA3::Inp15, p> {
	static_assert((p == Peripheral::Adc1) or (p == Peripheral::Adc2),
		"GpioA3::Inp15 only connects to Adc1 or Adc2!"); };
template<Peripheral p> struct SignalConnection<DataA3::Mck, p> {
	static_assert((p == Peripheral::I2s6),"GpioA3::Mck only connects to I2s6!"); };
template<Peripheral p> struct SignalConnection<DataA3::Out, p> {
	static_assert((p == Peripheral::Lptim5),"GpioA3::Out only connects to Lptim5!"); };
template<Peripheral p> struct SignalConnection<DataA3::P1clk, p> {
	static_assert((p == Peripheral::Octospim),"GpioA3::P1clk only connects to Octospim!"); };
template<Peripheral p> struct SignalConnection<DataA3::P1io2, p> {
	static_assert((p == Peripheral::Octospim),"GpioA3::P1io2 only connects to Octospim!"); };
template<Peripheral p> struct SignalConnection<DataA3::Rx, p> {
	static_assert((p == Peripheral::Usart2),"GpioA3::Rx only connects to Usart2!"); };
template<Peripheral p> struct SignalConnection<DataA3::Ulpid0, p> {
	static_assert((p == Peripheral::Usbotghs),"GpioA3::Ulpid0 only connects to Usbotghs!"); };
template<> struct SignalConnection<DataA3::BitBang, Peripheral::BitBang> { static constexpr int8_t af = -1; };
template<> struct SignalConnection<DataA3::B2, Peripheral::Ltdc> { static constexpr int8_t af = 9; };
template<> struct SignalConnection<DataA3::B5, Peripheral::Ltdc> { static constexpr int8_t af = 14; };
template<> struct SignalConnection<DataA3::Ch2, Peripheral::Tim15> { static constexpr int8_t af = 4; };
template<> struct SignalConnection<DataA3::Ch4, Peripheral::Tim2> { static constexpr int8_t af = 1; };
template<> struct SignalConnection<DataA3::Ch4, Peripheral::Tim5> { static constexpr int8_t af = 2; };
template<> struct SignalConnection<DataA3::Col, Peripheral::Eth> { static constexpr int8_t af = 11; };
template<> struct SignalConnection<DataA3::Inp15, Peripheral::Adc1> { static constexpr int8_t af = -2; };
template<> struct SignalConnection<DataA3::Inp15, Peripheral::Adc2> { static constexpr int8_t af = -2; };
template<> struct SignalConnection<DataA3::Mck, Peripheral::I2s6> { static constexpr int8_t af = 5; };
template<> struct SignalConnection<DataA3::Out, Peripheral::Lptim5> { static constexpr int8_t af = 3; };
template<> struct SignalConnection<DataA3::P1clk, Peripheral::Octospim> { static constexpr int8_t af = 12; };
template<> struct SignalConnection<DataA3::P1io2, Peripheral::Octospim> { static constexpr int8_t af = 6; };
template<> struct SignalConnection<DataA3::Rx, Peripheral::Usart2> { static constexpr int8_t af = 7; };
template<> struct SignalConnection<DataA3::Ulpid0, Peripheral::Usbotghs> { static constexpr int8_t af = 10; };
template<> constexpr inline int8_t AdcChannel<DataA3, Peripheral::Adc1, AdcPolarity::Positive> = 15;
template<> constexpr inline int8_t AdcChannel<DataA3, Peripheral::Adc2, AdcPolarity::Positive> = 15;

struct DataA4 {
	static constexpr Gpio::Port port = Gpio::Port::A;
	static constexpr uint8_t pin = 4;
	struct BitBang { using Data = DataA4; static constexpr Gpio::Signal Signal = Gpio::Signal::BitBang; };
	struct Ck { using Data = DataA4; static constexpr Gpio::Signal Signal = Gpio::Signal::Ck; };
	struct D8 { using Data = DataA4; static constexpr Gpio::Signal Signal = Gpio::Signal::D8; };
	struct Da8 { using Data = DataA4; static constexpr Gpio::Signal Signal = Gpio::Signal::Da8; };
	struct De { using Data = DataA4; static constexpr Gpio::Signal Signal = Gpio::Signal::De; };
	struct Etr { using Data = DataA4; static constexpr Gpio::Signal Signal = Gpio::Signal::Etr; };
	struct Hsync { using Data = DataA4; static constexpr Gpio::Signal Signal = Gpio::Signal::Hsync; };
	struct Inp18 { using Data = DataA4; static constexpr Gpio::Signal Signal = Gpio::Signal::Inp18; };
	struct Nss { using Data = DataA4; static constexpr Gpio::Signal Signal = Gpio::Signal::Nss; };
	struct Out1 { using Data = DataA4; static constexpr Gpio::Signal Signal = Gpio::Signal::Out1; };
	struct Vsync { using Data = DataA4; static constexpr Gpio::Signal Signal = Gpio::Signal::Vsync; };
	struct Ws { using Data = DataA4; static constexpr Gpio::Signal Signal = Gpio::Signal::Ws; };
};
template<Peripheral p> struct SignalConnection<DataA4::BitBang, p> {
	static_assert(p == Peripheral::BitBang, "GpioA4::BitBang only connects to software drivers!"); };
template<Peripheral p> struct SignalConnection<DataA4::Ck, p> {
	static_assert((p == Peripheral::Usart2),"GpioA4::Ck only connects to Usart2!"); };
template<Peripheral p> struct SignalConnection<DataA4::D8, p> {
	static_assert((p == Peripheral::Fmc),"GpioA4::D8 only connects to Fmc!"); };
template<Peripheral p> struct SignalConnection<DataA4::Da8, p> {
	static_assert((p == Peripheral::Fmc),"GpioA4::Da8 only connects to Fmc!"); };
template<Peripheral p> struct SignalConnection<DataA4::De, p> {
	static_assert((p == Peripheral::Pssi),"GpioA4::De only connects to Pssi!"); };
template<Peripheral p> struct SignalConnection<DataA4::Etr, p> {
	static_assert((p == Peripheral::Tim5),"GpioA4::Etr only connects to Tim5!"); };
template<Peripheral p> struct SignalConnection<DataA4::Hsync, p> {
	static_assert((p == Peripheral::Dcmi),"GpioA4::Hsync only connects to Dcmi!"); };
template<Peripheral p> struct SignalConnection<DataA4::Inp18, p> {
	static_assert((p == Peripheral::Adc1) or (p == Peripheral::Adc2),
		"GpioA4::Inp18 only connects to Adc1 or Adc2!"); };
template<Peripheral p> struct SignalConnection<DataA4::Nss, p> {
	static_assert((p == Peripheral::Spi1) or (p == Peripheral::Spi3) or (p == Peripheral::Spi6),
		"GpioA4::Nss only connects to Spi1 or Spi3 or Spi6!"); };
template<Peripheral p> struct SignalConnection<DataA4::Out1, p> {
	static_assert((p == Peripheral::Dac1),"GpioA4::Out1 only connects to Dac1!"); };
template<Peripheral p> struct SignalConnection<DataA4::Vsync, p> {
	static_assert((p == Peripheral::Ltdc),"GpioA4::Vsync only connects to Ltdc!"); };
template<Peripheral p> struct SignalConnection<DataA4::Ws, p> {
	static_assert((p == Peripheral::I2s1) or (p == Peripheral::I2s3) or (p == Peripheral::I2s6),
		"GpioA4::Ws only connects to I2s1 or I2s3 or I2s6!"); };
template<> struct SignalConnection<DataA4::BitBang, Peripheral::BitBang> { static constexpr int8_t af = -1; };
template<> struct SignalConnection<DataA4::Ck, Peripheral::Usart2> { static constexpr int8_t af = 7; };
template<> struct SignalConnection<DataA4::D8, Peripheral::Fmc> { static constexpr int8_t af = 12; };
template<> struct SignalConnection<DataA4::Da8, Peripheral::Fmc> { static constexpr int8_t af = 12; };
template<> struct SignalConnection<DataA4::De, Peripheral::Pssi> { static constexpr int8_t af = 13; };
template<> struct SignalConnection<DataA4::Etr, Peripheral::Tim5> { static constexpr int8_t af = 2; };
template<> struct SignalConnection<DataA4::Hsync, Peripheral::Dcmi> { static constexpr int8_t af = 13; };
template<> struct SignalConnection<DataA4::Inp18, Peripheral::Adc1> { static constexpr int8_t af = -2; };
template<> struct SignalConnection<DataA4::Inp18, Peripheral::Adc2> { static constexpr int8_t af = -2; };
template<> struct SignalConnection<DataA4::Nss, Peripheral::Spi1> { static constexpr int8_t af = 5; };
template<> struct SignalConnection<DataA4::Nss, Peripheral::Spi3> { static constexpr int8_t af = 6; };
template<> struct SignalConnection<DataA4::Nss, Peripheral::Spi6> { static constexpr int8_t af = 8; };
template<> struct SignalConnection<DataA4::Out1, Peripheral::Dac1> { static constexpr int8_t af = -2; };
template<> struct SignalConnection<DataA4::Vsync, Peripheral::Ltdc> { static constexpr int8_t af = 14; };
template<> struct SignalConnection<DataA4::Ws, Peripheral::I2s1> { static constexpr int8_t af = 5; };
template<> struct SignalConnection<DataA4::Ws, Peripheral::I2s3> { static constexpr int8_t af = 6; };
template<> struct SignalConnection<DataA4::Ws, Peripheral::I2s6> { static constexpr int8_t af = 8; };
template<> constexpr inline int8_t AdcChannel<DataA4, Peripheral::Adc1, AdcPolarity::Positive> = 18;
template<> constexpr inline int8_t AdcChannel<DataA4, Peripheral::Adc2, AdcPolarity::Positive> = 18;
template<> constexpr inline int8_t DacChannel<DataA4, Peripheral::Dac1> = 1;

struct DataA5 {
	static constexpr Gpio::Port port = Gpio::Port::A;
	static constexpr uint8_t pin = 5;
	struct BitBang { using Data = DataA5; static constexpr Gpio::Signal Signal = Gpio::Signal::BitBang; };
	struct Ch1 { using Data = DataA5; static constexpr Gpio::Signal Signal = Gpio::Signal::Ch1; };
	struct Ch1n { using Data = DataA5; static constexpr Gpio::Signal Signal = Gpio::Signal::Ch1n; };
	struct Ck { using Data = DataA5; static constexpr Gpio::Signal Signal = Gpio::Signal::Ck; };
	struct D14 { using Data = DataA5; static constexpr Gpio::Signal Signal = Gpio::Signal::D14; };
	struct D9 { using Data = DataA5; static constexpr Gpio::Signal Signal = Gpio::Signal::D9; };
	struct Da9 { using Data = DataA5; static constexpr Gpio::Signal Signal = Gpio::Signal::Da9; };
	struct Etr { using Data = DataA5; static constexpr Gpio::Signal Signal = Gpio::Signal::Etr; };
	struct Inn18 { using Data = DataA5; static constexpr Gpio::Signal Signal = Gpio::Signal::Inn18; };
	struct Inp19 { using Data = DataA5; static constexpr Gpio::Signal Signal = Gpio::Signal::Inp19; };
	struct Ndstop2 { using Data = DataA5; static constexpr Gpio::Signal Signal = Gpio::Signal::Ndstop2; };
	struct Out2 { using Data = DataA5; static constexpr Gpio::Signal Signal = Gpio::Signal::Out2; };
	struct R4 { using Data = DataA5; static constexpr Gpio::Signal Signal = Gpio::Signal::R4; };
	struct Sck { using Data = DataA5; static constexpr Gpio::Signal Signal = Gpio::Signal::Sck; };
	struct Ulpick { using Data = DataA5; static constexpr Gpio::Signal Signal = Gpio::Signal::Ulpick; };
};
template<Peripheral p> struct SignalConnection<DataA5::BitBang, p> {
	static_assert(p == Peripheral::BitBang, "GpioA5::BitBang only connects to software drivers!"); };
template<Peripheral p> struct SignalConnection<DataA5::Ch1, p> {
	static_assert((p == Peripheral::Tim2),"GpioA5::Ch1 only connects to Tim2!"); };
template<Peripheral p> struct SignalConnection<DataA5::Ch1n, p> {
	static_assert((p == Peripheral::Tim8),"GpioA5::Ch1n only connects to Tim8!"); };
template<Peripheral p> struct SignalConnection<DataA5::Ck, p> {
	static_assert((p == Peripheral::I2s1) or (p == Peripheral::I2s6),
		"GpioA5::Ck only connects to I2s1 or I2s6!"); };
template<Peripheral p> struct SignalConnection<DataA5::D14, p> {
	static_assert((p == Peripheral::Pssi),"GpioA5::D14 only connects to Pssi!"); };
template<Peripheral p> struct SignalConnection<DataA5::D9, p> {
	static_assert((p == Peripheral::Fmc),"GpioA5::D9 only connects to Fmc!"); };
template<Peripheral p> struct SignalConnection<DataA5::Da9, p> {
	static_assert((p == Peripheral::Fmc),"GpioA5::Da9 only connects to Fmc!"); };
template<Peripheral p> struct SignalConnection<DataA5::Etr, p> {
	static_assert((p == Peripheral::Tim2),"GpioA5::Etr only connects to Tim2!"); };
template<Peripheral p> struct SignalConnection<DataA5::Inn18, p> {
	static_assert((p == Peripheral::Adc1) or (p == Peripheral::Adc2),
		"GpioA5::Inn18 only connects to Adc1 or Adc2!"); };
template<Peripheral p> struct SignalConnection<DataA5::Inp19, p> {
	static_assert((p == Peripheral::Adc1) or (p == Peripheral::Adc2),
		"GpioA5::Inp19 only connects to Adc1 or Adc2!"); };
template<Peripheral p> struct SignalConnection<DataA5::Ndstop2, p> {
	static_assert((p == Peripheral::Pwr),"GpioA5::Ndstop2 only connects to Pwr!"); };
template<Peripheral p> struct SignalConnection<DataA5::Out2, p> {
	static_assert((p == Peripheral::Dac1),"GpioA5::Out2 only connects to Dac1!"); };
template<Peripheral p> struct SignalConnection<DataA5::R4, p> {
	static_assert((p == Peripheral::Ltdc),"GpioA5::R4 only connects to Ltdc!"); };
template<Peripheral p> struct SignalConnection<DataA5::Sck, p> {
	static_assert((p == Peripheral::Spi1) or (p == Peripheral::Spi6),
		"GpioA5::Sck only connects to Spi1 or Spi6!"); };
template<Peripheral p> struct SignalConnection<DataA5::Ulpick, p> {
	static_assert((p == Peripheral::Usbotghs),"GpioA5::Ulpick only connects to Usbotghs!"); };
template<> struct SignalConnection<DataA5::BitBang, Peripheral::BitBang> { static constexpr int8_t af = -1; };
template<> struct SignalConnection<DataA5::Ch1, Peripheral::Tim2> { static constexpr int8_t af = 1; };
template<> struct SignalConnection<DataA5::Ch1n, Peripheral::Tim8> { static constexpr int8_t af = 3; };
template<> struct SignalConnection<DataA5::Ck, Peripheral::I2s1> { static constexpr int8_t af = 5; };
template<> struct SignalConnection<DataA5::Ck, Peripheral::I2s6> { static constexpr int8_t af = 8; };
template<> struct SignalConnection<DataA5::D14, Peripheral::Pssi> { static constexpr int8_t af = 13; };
template<> struct SignalConnection<DataA5::D9, Peripheral::Fmc> { static constexpr int8_t af = 12; };
template<> struct SignalConnection<DataA5::Da9, Peripheral::Fmc> { static constexpr int8_t af = 12; };
template<> struct SignalConnection<DataA5::Etr, Peripheral::Tim2> { static constexpr int8_t af = 1; };
template<> struct SignalConnection<DataA5::Inn18, Peripheral::Adc1> { static constexpr int8_t af = -2; };
template<> struct SignalConnection<DataA5::Inn18, Peripheral::Adc2> { static constexpr int8_t af = -2; };
template<> struct SignalConnection<DataA5::Inp19, Peripheral::Adc1> { static constexpr int8_t af = -2; };
template<> struct SignalConnection<DataA5::Inp19, Peripheral::Adc2> { static constexpr int8_t af = -2; };
template<> struct SignalConnection<DataA5::Ndstop2, Peripheral::Pwr> { static constexpr int8_t af = -1; };
template<> struct SignalConnection<DataA5::Out2, Peripheral::Dac1> { static constexpr int8_t af = -2; };
template<> struct SignalConnection<DataA5::R4, Peripheral::Ltdc> { static constexpr int8_t af = 14; };
template<> struct SignalConnection<DataA5::Sck, Peripheral::Spi1> { static constexpr int8_t af = 5; };
template<> struct SignalConnection<DataA5::Sck, Peripheral::Spi6> { static constexpr int8_t af = 8; };
template<> struct SignalConnection<DataA5::Ulpick, Peripheral::Usbotghs> { static constexpr int8_t af = 10; };
template<> constexpr inline int8_t AdcChannel<DataA5, Peripheral::Adc1, AdcPolarity::Negative> = 18;
template<> constexpr inline int8_t AdcChannel<DataA5, Peripheral::Adc2, AdcPolarity::Negative> = 18;
template<> constexpr inline int8_t AdcChannel<DataA5, Peripheral::Adc1, AdcPolarity::Positive> = 19;
template<> constexpr inline int8_t AdcChannel<DataA5, Peripheral::Adc2, AdcPolarity::Positive> = 19;
template<> constexpr inline int8_t DacChannel<DataA5, Peripheral::Dac1> = 2;

struct DataA6 {
	static constexpr Gpio::Port port = Gpio::Port::A;
	static constexpr uint8_t pin = 6;
	struct BitBang { using Data = DataA6; static constexpr Gpio::Signal Signal = Gpio::Signal::BitBang; };
	struct Bkin { using Data = DataA6; static constexpr Gpio::Signal Signal = Gpio::Signal::Bkin; };
	struct Bkincomp1 { using Data = DataA6; static constexpr Gpio::Signal Signal = Gpio::Signal::Bkincomp1; };
	struct Bkincomp2 { using Data = DataA6; static constexpr Gpio::Signal Signal = Gpio::Signal::Bkincomp2; };
	struct Ch1 { using Data = DataA6; static constexpr Gpio::Signal Signal = Gpio::Signal::Ch1; };
	struct G2 { using Data = DataA6; static constexpr Gpio::Signal Signal = Gpio::Signal::G2; };
	struct Inp3 { using Data = DataA6; static constexpr Gpio::Signal Signal = Gpio::Signal::Inp3; };
	struct Mdc { using Data = DataA6; static constexpr Gpio::Signal Signal = Gpio::Signal::Mdc; };
	struct Miso { using Data = DataA6; static constexpr Gpio::Signal Signal = Gpio::Signal::Miso; };
	struct P1io3 { using Data = DataA6; static constexpr Gpio::Signal Signal = Gpio::Signal::P1io3; };
	struct Pdck { using Data = DataA6; static constexpr Gpio::Signal Signal = Gpio::Signal::Pdck; };
	struct Pixclk { using Data = DataA6; static constexpr Gpio::Signal Signal = Gpio::Signal::Pixclk; };
	struct Sdi { using Data = DataA6; static constexpr Gpio::Signal Signal = Gpio::Signal::Sdi; };
};
template<Peripheral p> struct SignalConnection<DataA6::BitBang, p> {
	static_assert(p == Peripheral::BitBang, "GpioA6::BitBang only connects to software drivers!"); };
template<Peripheral p> struct SignalConnection<DataA6::Bkin, p> {
	static_assert((p == Peripheral::Tim1) or (p == Peripheral::Tim8),
		"GpioA6::Bkin only connects to Tim1 or Tim8!"); };
template<Peripheral p> struct SignalConnection<DataA6::Bkincomp1, p> {
	static_assert((p == Peripheral::Tim8) or (p == Peripheral::Tim1),
		"GpioA6::Bkincomp1 only connects to Tim8 or Tim1!"); };
template<Peripheral p> struct SignalConnection<DataA6::Bkincomp2, p> {
	static_assert((p == Peripheral::Tim8) or (p == Peripheral::Tim1),
		"GpioA6::Bkincomp2 only connects to Tim8 or Tim1!"); };
template<Peripheral p> struct SignalConnection<DataA6::Ch1, p> {
	static_assert((p == Peripheral::Tim3) or (p == Peripheral::Tim13),
		"GpioA6::Ch1 only connects to Tim3 or Tim13!"); };
template<Peripheral p> struct SignalConnection<DataA6::G2, p> {
	static_assert((p == Peripheral::Ltdc),"GpioA6::G2 only connects to Ltdc!"); };
template<Peripheral p> struct SignalConnection<DataA6::Inp3, p> {
	static_assert((p == Peripheral::Adc1) or (p == Peripheral::Adc2),
		"GpioA6::Inp3 only connects to Adc1 or Adc2!"); };
template<Peripheral p> struct SignalConnection<DataA6::Mdc, p> {
	static_assert((p == Peripheral::Mdios),"GpioA6::Mdc only connects to Mdios!"); };
template<Peripheral p> struct SignalConnection<DataA6::Miso, p> {
	static_assert((p == Peripheral::Spi1) or (p == Peripheral::Spi6),
		"GpioA6::Miso only connects to Spi1 or Spi6!"); };
template<Peripheral p> struct SignalConnection<DataA6::P1io3, p> {
	static_assert((p == Peripheral::Octospim),"GpioA6::P1io3 only connects to Octospim!"); };
template<Peripheral p> struct SignalConnection<DataA6::Pdck, p> {
	static_assert((p == Peripheral::Pssi),"GpioA6::Pdck only connects to Pssi!"); };
template<Peripheral p> struct SignalConnection<DataA6::Pixclk, p> {
	static_assert((p == Peripheral::Dcmi),"GpioA6::Pixclk only connects to Dcmi!"); };
template<Peripheral p> struct SignalConnection<DataA6::Sdi, p> {
	static_assert((p == Peripheral::I2s1) or (p == Peripheral::I2s6),
		"GpioA6::Sdi only connects to I2s1 or I2s6!"); };
template<> struct SignalConnection<DataA6::BitBang, Peripheral::BitBang> { static constexpr int8_t af = -1; };
template<> struct SignalConnection<DataA6::Bkin, Peripheral::Tim1> { static constexpr int8_t af = 1; };
template<> struct SignalConnection<DataA6::Bkin, Peripheral::Tim8> { static constexpr int8_t af = 3; };
template<> struct SignalConnection<DataA6::Bkincomp1, Peripheral::Tim8> { static constexpr int8_t af = 10; };
template<> struct SignalConnection<DataA6::Bkincomp1, Peripheral::Tim1> { static constexpr int8_t af = 12; };
template<> struct SignalConnection<DataA6::Bkincomp2, Peripheral::Tim8> { static constexpr int8_t af = 10; };
template<> struct SignalConnection<DataA6::Bkincomp2, Peripheral::Tim1> { static constexpr int8_t af = 12; };
template<> struct SignalConnection<DataA6::Ch1, Peripheral::Tim3> { static constexpr int8_t af = 2; };
template<> struct SignalConnection<DataA6::Ch1, Peripheral::Tim13> { static constexpr int8_t af = 9; };
template<> struct SignalConnection<DataA6::G2, Peripheral::Ltdc> { static constexpr int8_t af = 14; };
template<> struct SignalConnection<DataA6::Inp3, Peripheral::Adc1> { static constexpr int8_t af = -2; };
template<> struct SignalConnection<DataA6::Inp3, Peripheral::Adc2> { static constexpr int8_t af = -2; };
template<> struct SignalConnection<DataA6::Mdc, Peripheral::Mdios> { static constexpr int8_t af = 11; };
template<> struct SignalConnection<DataA6::Miso, Peripheral::Spi1> { static constexpr int8_t af = 5; };
template<> struct SignalConnection<DataA6::Miso, Peripheral::Spi6> { static constexpr int8_t af = 8; };
template<> struct SignalConnection<DataA6::P1io3, Peripheral::Octospim> { static constexpr int8_t af = 6; };
template<> struct SignalConnection<DataA6::Pdck, Peripheral::Pssi> { static constexpr int8_t af = 13; };
template<> struct SignalConnection<DataA6::Pixclk, Peripheral::Dcmi> { static constexpr int8_t af = 13; };
template<> struct SignalConnection<DataA6::Sdi, Peripheral::I2s1> { static constexpr int8_t af = 5; };
template<> struct SignalConnection<DataA6::Sdi, Peripheral::I2s6> { static constexpr int8_t af = 8; };
template<> constexpr inline int8_t AdcChannel<DataA6, Peripheral::Adc1, AdcPolarity::Positive> = 3;
template<> constexpr inline int8_t AdcChannel<DataA6, Peripheral::Adc2, AdcPolarity::Positive> = 3;

struct DataA7 {
	static constexpr Gpio::Port port = Gpio::Port::A;
	static constexpr uint8_t pin = 7;
	struct BitBang { using Data = DataA7; static constexpr Gpio::Signal Signal = Gpio::Signal::BitBang; };
	struct Ch1 { using Data = DataA7; static constexpr Gpio::Signal Signal = Gpio::Signal::Ch1; };
	struct Ch1n { using Data = DataA7; static constexpr Gpio::Signal Signal = Gpio::Signal::Ch1n; };
	struct Ch2 { using Data = DataA7; static constexpr Gpio::Signal Signal = Gpio::Signal::Ch2; };
	struct Inn3 { using Data = DataA7; static constexpr Gpio::Signal Signal = Gpio::Signal::Inn3; };
	struct Inp7 { using Data = DataA7; static constexpr Gpio::Signal Signal = Gpio::Signal::Inp7; };
	struct Mosi { using Data = DataA7; static constexpr Gpio::Signal Signal = Gpio::Signal::Mosi; };
	struct P1io2 { using Data = DataA7; static constexpr Gpio::Signal Signal = Gpio::Signal::P1io2; };
	struct Rcccrsdv { using Data = DataA7; static constexpr Gpio::Signal Signal = Gpio::Signal::Rcccrsdv; };
	struct Rxdv { using Data = DataA7; static constexpr Gpio::Signal Signal = Gpio::Signal::Rxdv; };
	struct Sdnwe { using Data = DataA7; static constexpr Gpio::Signal Signal = Gpio::Signal::Sdnwe; };
	struct Sdo { using Data = DataA7; static constexpr Gpio::Signal Signal = Gpio::Signal::Sdo; };
	struct Vinm { using Data = DataA7; static constexpr Gpio::Signal Signal = Gpio::Signal::Vinm; };
	struct Vsync { using Data = DataA7; static constexpr Gpio::Signal Signal = Gpio::Signal::Vsync; };
};
template<Peripheral p> struct SignalConnection<DataA7::BitBang, p> {
	static_assert(p == Peripheral::BitBang, "GpioA7::BitBang only connects to software drivers!"); };
template<Peripheral p> struct SignalConnection<DataA7::Ch1, p> {
	static_assert((p == Peripheral::Tim14),"GpioA7::Ch1 only connects to Tim14!"); };
template<Peripheral p> struct SignalConnection<DataA7::Ch1n, p> {
	static_assert((p == Peripheral::Tim1) or (p == Peripheral::Tim8),
		"GpioA7::Ch1n only connects to Tim1 or Tim8!"); };
template<Peripheral p> struct SignalConnection<DataA7::Ch2, p> {
	static_assert((p == Peripheral::Tim3),"GpioA7::Ch2 only connects to Tim3!"); };
template<Peripheral p> struct SignalConnection<DataA7::Inn3, p> {
	static_assert((p == Peripheral::Adc1) or (p == Peripheral::Adc2),
		"GpioA7::Inn3 only connects to Adc1 or Adc2!"); };
template<Peripheral p> struct SignalConnection<DataA7::Inp7, p> {
	static_assert((p == Peripheral::Adc1) or (p == Peripheral::Adc2),
		"GpioA7::Inp7 only connects to Adc1 or Adc2!"); };
template<Peripheral p> struct SignalConnection<DataA7::Mosi, p> {
	static_assert((p == Peripheral::Spi1) or (p == Peripheral::Spi6),
		"GpioA7::Mosi only connects to Spi1 or Spi6!"); };
template<Peripheral p> struct SignalConnection<DataA7::P1io2, p> {
	static_assert((p == Peripheral::Octospim),"GpioA7::P1io2 only connects to Octospim!"); };
template<Peripheral p> struct SignalConnection<DataA7::Rcccrsdv, p> {
	static_assert((p == Peripheral::Eth),"GpioA7::Rcccrsdv only connects to Eth!"); };
template<Peripheral p> struct SignalConnection<DataA7::Rxdv, p> {
	static_assert((p == Peripheral::Eth),"GpioA7::Rxdv only connects to Eth!"); };
template<Peripheral p> struct SignalConnection<DataA7::Sdnwe, p> {
	static_assert((p == Peripheral::Fmc),"GpioA7::Sdnwe only connects to Fmc!"); };
template<Peripheral p> struct SignalConnection<DataA7::Sdo, p> {
	static_assert((p == Peripheral::I2s1) or (p == Peripheral::I2s6),
		"GpioA7::Sdo only connects to I2s1 or I2s6!"); };
template<Peripheral p> struct SignalConnection<DataA7::Vinm, p> {
	static_assert((p == Peripheral::Opamp1),"GpioA7::Vinm only connects to Opamp1!"); };
template<Peripheral p> struct SignalConnection<DataA7::Vsync, p> {
	static_assert((p == Peripheral::Ltdc),"GpioA7::Vsync only connects to Ltdc!"); };
template<> struct SignalConnection<DataA7::BitBang, Peripheral::BitBang> { static constexpr int8_t af = -1; };
template<> struct SignalConnection<DataA7::Ch1, Peripheral::Tim14> { static constexpr int8_t af = 9; };
template<> struct SignalConnection<DataA7::Ch1n, Peripheral::Tim1> { static constexpr int8_t af = 1; };
template<> struct SignalConnection<DataA7::Ch1n, Peripheral::Tim8> { static constexpr int8_t af = 3; };
template<> struct SignalConnection<DataA7::Ch2, Peripheral::Tim3> { static constexpr int8_t af = 2; };
template<> struct SignalConnection<DataA7::Inn3, Peripheral::Adc1> { static constexpr int8_t af = -2; };
template<> struct SignalConnection<DataA7::Inn3, Peripheral::Adc2> { static constexpr int8_t af = -2; };
template<> struct SignalConnection<DataA7::Inp7, Peripheral::Adc1> { static constexpr int8_t af = -2; };
template<> struct SignalConnection<DataA7::Inp7, Peripheral::Adc2> { static constexpr int8_t af = -2; };
template<> struct SignalConnection<DataA7::Mosi, Peripheral::Spi1> { static constexpr int8_t af = 5; };
template<> struct SignalConnection<DataA7::Mosi, Peripheral::Spi6> { static constexpr int8_t af = 8; };
template<> struct SignalConnection<DataA7::P1io2, Peripheral::Octospim> { static constexpr int8_t af = 10; };
template<> struct SignalConnection<DataA7::Rcccrsdv, Peripheral::Eth> { static constexpr int8_t af = 11; };
template<> struct SignalConnection<DataA7::Rxdv, Peripheral::Eth> { static constexpr int8_t af = 11; };
template<> struct SignalConnection<DataA7::Sdnwe, Peripheral::Fmc> { static constexpr int8_t af = 12; };
template<> struct SignalConnection<DataA7::Sdo, Peripheral::I2s1> { static constexpr int8_t af = 5; };
template<> struct SignalConnection<DataA7::Sdo, Peripheral::I2s6> { static constexpr int8_t af = 8; };
template<> struct SignalConnection<DataA7::Vinm, Peripheral::Opamp1> { static constexpr int8_t af = -1; };
template<> struct SignalConnection<DataA7::Vsync, Peripheral::Ltdc> { static constexpr int8_t af = 14; };
template<> constexpr inline int8_t AdcChannel<DataA7, Peripheral::Adc1, AdcPolarity::Negative> = 3;
template<> constexpr inline int8_t AdcChannel<DataA7, Peripheral::Adc2, AdcPolarity::Negative> = 3;
template<> constexpr inline int8_t AdcChannel<DataA7, Peripheral::Adc1, AdcPolarity::Positive> = 7;
template<> constexpr inline int8_t AdcChannel<DataA7, Peripheral::Adc2, AdcPolarity::Positive> = 7;

struct DataA8 {
	static constexpr Gpio::Port port = Gpio::Port::A;
	static constexpr uint8_t pin = 8;
	struct BitBang { using Data = DataA8; static constexpr Gpio::Signal Signal = Gpio::Signal::BitBang; };
	struct B3 { using Data = DataA8; static constexpr Gpio::Signal Signal = Gpio::Signal::B3; };
	struct Bkin2 { using Data = DataA8; static constexpr Gpio::Signal Signal = Gpio::Signal::Bkin2; };
	struct Bkin2comp1 { using Data = DataA8; static constexpr Gpio::Signal Signal = Gpio::Signal::Bkin2comp1; };
	struct Bkin2comp2 { using Data = DataA8; static constexpr Gpio::Signal Signal = Gpio::Signal::Bkin2comp2; };
	struct Ch1 { using Data = DataA8; static constexpr Gpio::Signal Signal = Gpio::Signal::Ch1; };
	struct Ck { using Data = DataA8; static constexpr Gpio::Signal Signal = Gpio::Signal::Ck; };
	struct Mco1 { using Data = DataA8; static constexpr Gpio::Signal Signal = Gpio::Signal::Mco1; };
	struct R6 { using Data = DataA8; static constexpr Gpio::Signal Signal = Gpio::Signal::R6; };
	struct Rx { using Data = DataA8; static constexpr Gpio::Signal Signal = Gpio::Signal::Rx; };
	struct Scl { using Data = DataA8; static constexpr Gpio::Signal Signal = Gpio::Signal::Scl; };
	struct Sof { using Data = DataA8; static constexpr Gpio::Signal Signal = Gpio::Signal::Sof; };
};
template<Peripheral p> struct SignalConnection<DataA8::BitBang, p> {
	static_assert(p == Peripheral::BitBang, "GpioA8::BitBang only connects to software drivers!"); };
template<Peripheral p> struct SignalConnection<DataA8::B3, p> {
	static_assert((p == Peripheral::Ltdc),"GpioA8::B3 only connects to Ltdc!"); };
template<Peripheral p> struct SignalConnection<DataA8::Bkin2, p> {
	static_assert((p == Peripheral::Tim8),"GpioA8::Bkin2 only connects to Tim8!"); };
template<Peripheral p> struct SignalConnection<DataA8::Bkin2comp1, p> {
	static_assert((p == Peripheral::Tim8),"GpioA8::Bkin2comp1 only connects to Tim8!"); };
template<Peripheral p> struct SignalConnection<DataA8::Bkin2comp2, p> {
	static_assert((p == Peripheral::Tim8),"GpioA8::Bkin2comp2 only connects to Tim8!"); };
template<Peripheral p> struct SignalConnection<DataA8::Ch1, p> {
	static_assert((p == Peripheral::Tim1),"GpioA8::Ch1 only connects to Tim1!"); };
template<Peripheral p> struct SignalConnection<DataA8::Ck, p> {
	static_assert((p == Peripheral::Usart1),"GpioA8::Ck only connects to Usart1!"); };
template<Peripheral p> struct SignalConnection<DataA8::Mco1, p> {
	static_assert((p == Peripheral::Rcc),"GpioA8::Mco1 only connects to Rcc!"); };
template<Peripheral p> struct SignalConnection<DataA8::R6, p> {
	static_assert((p == Peripheral::Ltdc),"GpioA8::R6 only connects to Ltdc!"); };
template<Peripheral p> struct SignalConnection<DataA8::Rx, p> {
	static_assert((p == Peripheral::Uart7),"GpioA8::Rx only connects to Uart7!"); };
template<Peripheral p> struct SignalConnection<DataA8::Scl, p> {
	static_assert((p == Peripheral::I2c3) or (p == Peripheral::I2c5),
		"GpioA8::Scl only connects to I2c3 or I2c5!"); };
template<Peripheral p> struct SignalConnection<DataA8::Sof, p> {
	static_assert((p == Peripheral::Usbotghs),"GpioA8::Sof only connects to Usbotghs!"); };
template<> struct SignalConnection<DataA8::BitBang, Peripheral::BitBang> { static constexpr int8_t af = -1; };
template<> struct SignalConnection<DataA8::B3, Peripheral::Ltdc> { static constexpr int8_t af = 13; };
template<> struct SignalConnection<DataA8::Bkin2, Peripheral::Tim8> { static constexpr int8_t af = 3; };
template<> struct SignalConnection<DataA8::Bkin2comp1, Peripheral::Tim8> { static constexpr int8_t af = 12; };
template<> struct SignalConnection<DataA8::Bkin2comp2, Peripheral::Tim8> { static constexpr int8_t af = 12; };
template<> struct SignalConnection<DataA8::Ch1, Peripheral::Tim1> { static constexpr int8_t af = 1; };
template<> struct SignalConnection<DataA8::Ck, Peripheral::Usart1> { static constexpr int8_t af = 7; };
template<> struct SignalConnection<DataA8::Mco1, Peripheral::Rcc> { static constexpr int8_t af = 0; };
template<> struct SignalConnection<DataA8::R6, Peripheral::Ltdc> { static constexpr int8_t af = 14; };
template<> struct SignalConnection<DataA8::Rx, Peripheral::Uart7> { static constexpr int8_t af = 11; };
template<> struct SignalConnection<DataA8::Scl, Peripheral::I2c3> { static constexpr int8_t af = 4; };
template<> struct SignalConnection<DataA8::Scl, Peripheral::I2c5> { static constexpr int8_t af = 6; };
template<> struct SignalConnection<DataA8::Sof, Peripheral::Usbotghs> { static constexpr int8_t af = 10; };

struct DataA9 {
	static constexpr Gpio::Port port = Gpio::Port::A;
	static constexpr uint8_t pin = 9;
	struct BitBang { using Data = DataA9; static constexpr Gpio::Signal Signal = Gpio::Signal::BitBang; };
	struct Ch2 { using Data = DataA9; static constexpr Gpio::Signal Signal = Gpio::Signal::Ch2; };
	struct Ck { using Data = DataA9; static constexpr Gpio::Signal Signal = Gpio::Signal::Ck; };
	struct D0 { using Data = DataA9; static constexpr Gpio::Signal Signal = Gpio::Signal::D0; };
	struct R5 { using Data = DataA9; static constexpr Gpio::Signal Signal = Gpio::Signal::R5; };
	struct Sck { using Data = DataA9; static constexpr Gpio::Signal Signal = Gpio::Signal::Sck; };
	struct Smba { using Data = DataA9; static constexpr Gpio::Signal Signal = Gpio::Signal::Smba; };
	struct Tx { using Data = DataA9; static constexpr Gpio::Signal Signal = Gpio::Signal::Tx; };
	struct Txer { using Data = DataA9; static constexpr Gpio::Signal Signal = Gpio::Signal::Txer; };
	struct Vbus { using Data = DataA9; static constexpr Gpio::Signal Signal = Gpio::Signal::Vbus; };
};
template<Peripheral p> struct SignalConnection<DataA9::BitBang, p> {
	static_assert(p == Peripheral::BitBang, "GpioA9::BitBang only connects to software drivers!"); };
template<Peripheral p> struct SignalConnection<DataA9::Ch2, p> {
	static_assert((p == Peripheral::Tim1),"GpioA9::Ch2 only connects to Tim1!"); };
template<Peripheral p> struct SignalConnection<DataA9::Ck, p> {
	static_assert((p == Peripheral::I2s2),"GpioA9::Ck only connects to I2s2!"); };
template<Peripheral p> struct SignalConnection<DataA9::D0, p> {
	static_assert((p == Peripheral::Dcmi) or (p == Peripheral::Pssi),
		"GpioA9::D0 only connects to Dcmi or Pssi!"); };
template<Peripheral p> struct SignalConnection<DataA9::R5, p> {
	static_assert((p == Peripheral::Ltdc),"GpioA9::R5 only connects to Ltdc!"); };
template<Peripheral p> struct SignalConnection<DataA9::Sck, p> {
	static_assert((p == Peripheral::Spi2),"GpioA9::Sck only connects to Spi2!"); };
template<Peripheral p> struct SignalConnection<DataA9::Smba, p> {
	static_assert((p == Peripheral::I2c3) or (p == Peripheral::I2c5),
		"GpioA9::Smba only connects to I2c3 or I2c5!"); };
template<Peripheral p> struct SignalConnection<DataA9::Tx, p> {
	static_assert((p == Peripheral::Lpuart1) or (p == Peripheral::Usart1),
		"GpioA9::Tx only connects to Lpuart1 or Usart1!"); };
template<Peripheral p> struct SignalConnection<DataA9::Txer, p> {
	static_assert((p == Peripheral::Eth),"GpioA9::Txer only connects to Eth!"); };
template<Peripheral p> struct SignalConnection<DataA9::Vbus, p> {
	static_assert((p == Peripheral::Usbotghs),"GpioA9::Vbus only connects to Usbotghs!"); };
template<> struct SignalConnection<DataA9::BitBang, Peripheral::BitBang> { static constexpr int8_t af = -1; };
template<> struct SignalConnection<DataA9::Ch2, Peripheral::Tim1> { static constexpr int8_t af = 1; };
template<> struct SignalConnection<DataA9::Ck, Peripheral::I2s2> { static constexpr int8_t af = 5; };
template<> struct SignalConnection<DataA9::D0, Peripheral::Dcmi> { static constexpr int8_t af = 13; };
template<> struct SignalConnection<DataA9::D0, Peripheral::Pssi> { static constexpr int8_t af = 13; };
template<> struct SignalConnection<DataA9::R5, Peripheral::Ltdc> { static constexpr int8_t af = 14; };
template<> struct SignalConnection<DataA9::Sck, Peripheral::Spi2> { static constexpr int8_t af = 5; };
template<> struct SignalConnection<DataA9::Smba, Peripheral::I2c3> { static constexpr int8_t af = 4; };
template<> struct SignalConnection<DataA9::Smba, Peripheral::I2c5> { static constexpr int8_t af = 6; };
template<> struct SignalConnection<DataA9::Tx, Peripheral::Lpuart1> { static constexpr int8_t af = 3; };
template<> struct SignalConnection<DataA9::Tx, Peripheral::Usart1> { static constexpr int8_t af = 7; };
template<> struct SignalConnection<DataA9::Txer, Peripheral::Eth> { static constexpr int8_t af = 11; };
template<> struct SignalConnection<DataA9::Vbus, Peripheral::Usbotghs> { static constexpr int8_t af = -1; };

struct DataA10 {
	static constexpr Gpio::Port port = Gpio::Port::A;
	static constexpr uint8_t pin = 10;
	struct BitBang { using Data = DataA10; static constexpr Gpio::Signal Signal = Gpio::Signal::BitBang; };
	struct B1 { using Data = DataA10; static constexpr Gpio::Signal Signal = Gpio::Signal::B1; };
	struct B4 { using Data = DataA10; static constexpr Gpio::Signal Signal = Gpio::Signal::B4; };
	struct Ch3 { using Data = DataA10; static constexpr Gpio::Signal Signal = Gpio::Signal::Ch3; };
	struct D1 { using Data = DataA10; static constexpr Gpio::Signal Signal = Gpio::Signal::D1; };
	struct Id { using Data = DataA10; static constexpr Gpio::Signal Signal = Gpio::Signal::Id; };
	struct Mdio { using Data = DataA10; static constexpr Gpio::Signal Signal = Gpio::Signal::Mdio; };
	struct Rx { using Data = DataA10; static constexpr Gpio::Signal Signal = Gpio::Signal::Rx; };
};
template<Peripheral p> struct SignalConnection<DataA10::BitBang, p> {
	static_assert(p == Peripheral::BitBang, "GpioA10::BitBang only connects to software drivers!"); };
template<Peripheral p> struct SignalConnection<DataA10::B1, p> {
	static_assert((p == Peripheral::Ltdc),"GpioA10::B1 only connects to Ltdc!"); };
template<Peripheral p> struct SignalConnection<DataA10::B4, p> {
	static_assert((p == Peripheral::Ltdc),"GpioA10::B4 only connects to Ltdc!"); };
template<Peripheral p> struct SignalConnection<DataA10::Ch3, p> {
	static_assert((p == Peripheral::Tim1),"GpioA10::Ch3 only connects to Tim1!"); };
template<Peripheral p> struct SignalConnection<DataA10::D1, p> {
	static_assert((p == Peripheral::Dcmi) or (p == Peripheral::Pssi),
		"GpioA10::D1 only connects to Dcmi or Pssi!"); };
template<Peripheral p> struct SignalConnection<DataA10::Id, p> {
	static_assert((p == Peripheral::Usbotghs),"GpioA10::Id only connects to Usbotghs!"); };
template<Peripheral p> struct SignalConnection<DataA10::Mdio, p> {
	static_assert((p == Peripheral::Mdios),"GpioA10::Mdio only connects to Mdios!"); };
template<Peripheral p> struct SignalConnection<DataA10::Rx, p> {
	static_assert((p == Peripheral::Lpuart1) or (p == Peripheral::Usart1),
		"GpioA10::Rx only connects to Lpuart1 or Usart1!"); };
template<> struct SignalConnection<DataA10::BitBang, Peripheral::BitBang> { static constexpr int8_t af = -1; };
template<> struct SignalConnection<DataA10::B1, Peripheral::Ltdc> { static constexpr int8_t af = 14; };
template<> struct SignalConnection<DataA10::B4, Peripheral::Ltdc> { static constexpr int8_t af = 12; };
template<> struct SignalConnection<DataA10::Ch3, Peripheral::Tim1> { static constexpr int8_t af = 1; };
template<> struct SignalConnection<DataA10::D1, Peripheral::Dcmi> { static constexpr int8_t af = 13; };
template<> struct SignalConnection<DataA10::D1, Peripheral::Pssi> { static constexpr int8_t af = 13; };
template<> struct SignalConnection<DataA10::Id, Peripheral::Usbotghs> { static constexpr int8_t af = 10; };
template<> struct SignalConnection<DataA10::Mdio, Peripheral::Mdios> { static constexpr int8_t af = 11; };
template<> struct SignalConnection<DataA10::Rx, Peripheral::Lpuart1> { static constexpr int8_t af = 3; };
template<> struct SignalConnection<DataA10::Rx, Peripheral::Usart1> { static constexpr int8_t af = 7; };

struct DataA11 {
	static constexpr Gpio::Port port = Gpio::Port::A;
	static constexpr uint8_t pin = 11;
	struct BitBang { using Data = DataA11; static constexpr Gpio::Signal Signal = Gpio::Signal::BitBang; };
	struct Ch4 { using Data = DataA11; static constexpr Gpio::Signal Signal = Gpio::Signal::Ch4; };
	struct Cts { using Data = DataA11; static constexpr Gpio::Signal Signal = Gpio::Signal::Cts; };
	struct Dm { using Data = DataA11; static constexpr Gpio::Signal Signal = Gpio::Signal::Dm; };
	struct Nss { using Data = DataA11; static constexpr Gpio::Signal Signal = Gpio::Signal::Nss; };
	struct R4 { using Data = DataA11; static constexpr Gpio::Signal Signal = Gpio::Signal::R4; };
	struct Rx { using Data = DataA11; static constexpr Gpio::Signal Signal = Gpio::Signal::Rx; };
	struct Ws { using Data = DataA11; static constexpr Gpio::Signal Signal = Gpio::Signal::Ws; };
};
template<Peripheral p> struct SignalConnection<DataA11::BitBang, p> {
	static_assert(p == Peripheral::BitBang, "GpioA11::BitBang only connects to software drivers!"); };
template<Peripheral p> struct SignalConnection<DataA11::Ch4, p> {
	static_assert((p == Peripheral::Tim1),"GpioA11::Ch4 only connects to Tim1!"); };
template<Peripheral p> struct SignalConnection<DataA11::Cts, p> {
	static_assert((p == Peripheral::Lpuart1) or (p == Peripheral::Usart1),
		"GpioA11::Cts only connects to Lpuart1 or Usart1!"); };
template<Peripheral p> struct SignalConnection<DataA11::Dm, p> {
	static_assert((p == Peripheral::Usbotghs),"GpioA11::Dm only connects to Usbotghs!"); };
template<Peripheral p> struct SignalConnection<DataA11::Nss, p> {
	static_assert((p == Peripheral::Spi2) or (p == Peripheral::Usart1),
		"GpioA11::Nss only connects to Spi2 or Usart1!"); };
template<Peripheral p> struct SignalConnection<DataA11::R4, p> {
	static_assert((p == Peripheral::Ltdc),"GpioA11::R4 only connects to Ltdc!"); };
template<Peripheral p> struct SignalConnection<DataA11::Rx, p> {
	static_assert((p == Peripheral::Uart4) or (p == Peripheral::Fdcan1),
		"GpioA11::Rx only connects to Uart4 or Fdcan1!"); };
template<Peripheral p> struct SignalConnection<DataA11::Ws, p> {
	static_assert((p == Peripheral::I2s2),"GpioA11::Ws only connects to I2s2!"); };
template<> struct SignalConnection<DataA11::BitBang, Peripheral::BitBang> { static constexpr int8_t af = -1; };
template<> struct SignalConnection<DataA11::Ch4, Peripheral::Tim1> { static constexpr int8_t af = 1; };
template<> struct SignalConnection<DataA11::Cts, Peripheral::Lpuart1> { static constexpr int8_t af = 3; };
template<> struct SignalConnection<DataA11::Cts, Peripheral::Usart1> { static constexpr int8_t af = 7; };
template<> struct SignalConnection<DataA11::Dm, Peripheral::Usbotghs> { static constexpr int8_t af = -1; };
template<> struct SignalConnection<DataA11::Nss, Peripheral::Spi2> { static constexpr int8_t af = 5; };
template<> struct SignalConnection<DataA11::Nss, Peripheral::Usart1> { static constexpr int8_t af = 7; };
template<> struct SignalConnection<DataA11::R4, Peripheral::Ltdc> { static constexpr int8_t af = 14; };
template<> struct SignalConnection<DataA11::Rx, Peripheral::Uart4> { static constexpr int8_t af = 6; };
template<> struct SignalConnection<DataA11::Rx, Peripheral::Fdcan1> { static constexpr int8_t af = 9; };
template<> struct SignalConnection<DataA11::Ws, Peripheral::I2s2> { static constexpr int8_t af = 5; };

struct DataA12 {
	static constexpr Gpio::Port port = Gpio::Port::A;
	static constexpr uint8_t pin = 12;
	struct BitBang { using Data = DataA12; static constexpr Gpio::Signal Signal = Gpio::Signal::BitBang; };
	struct Bkin2 { using Data = DataA12; static constexpr Gpio::Signal Signal = Gpio::Signal::Bkin2; };
	struct Ck { using Data = DataA12; static constexpr Gpio::Signal Signal = Gpio::Signal::Ck; };
	struct De { using Data = DataA12; static constexpr Gpio::Signal Signal = Gpio::Signal::De; };
	struct Dp { using Data = DataA12; static constexpr Gpio::Signal Signal = Gpio::Signal::Dp; };
	struct Etr { using Data = DataA12; static constexpr Gpio::Signal Signal = Gpio::Signal::Etr; };
	struct Fsb { using Data = DataA12; static constexpr Gpio::Signal Signal = Gpio::Signal::Fsb; };
	struct R5 { using Data = DataA12; static constexpr Gpio::Signal Signal = Gpio::Signal::R5; };
	struct Rts { using Data = DataA12; static constexpr Gpio::Signal Signal = Gpio::Signal::Rts; };
	struct Sck { using Data = DataA12; static constexpr Gpio::Signal Signal = Gpio::Signal::Sck; };
	struct Tx { using Data = DataA12; static constexpr Gpio::Signal Signal = Gpio::Signal::Tx; };
};
template<Peripheral p> struct SignalConnection<DataA12::BitBang, p> {
	static_assert(p == Peripheral::BitBang, "GpioA12::BitBang only connects to software drivers!"); };
template<Peripheral p> struct SignalConnection<DataA12::Bkin2, p> {
	static_assert((p == Peripheral::Tim1),"GpioA12::Bkin2 only connects to Tim1!"); };
template<Peripheral p> struct SignalConnection<DataA12::Ck, p> {
	static_assert((p == Peripheral::I2s2),"GpioA12::Ck only connects to I2s2!"); };
template<Peripheral p> struct SignalConnection<DataA12::De, p> {
	static_assert((p == Peripheral::Lpuart1) or (p == Peripheral::Usart1),
		"GpioA12::De only connects to Lpuart1 or Usart1!"); };
template<Peripheral p> struct SignalConnection<DataA12::Dp, p> {
	static_assert((p == Peripheral::Usbotghs),"GpioA12::Dp only connects to Usbotghs!"); };
template<Peripheral p> struct SignalConnection<DataA12::Etr, p> {
	static_assert((p == Peripheral::Tim1),"GpioA12::Etr only connects to Tim1!"); };
template<Peripheral p> struct SignalConnection<DataA12::Fsb, p> {
	static_assert((p == Peripheral::Sai4),"GpioA12::Fsb only connects to Sai4!"); };
template<Peripheral p> struct SignalConnection<DataA12::R5, p> {
	static_assert((p == Peripheral::Ltdc),"GpioA12::R5 only connects to Ltdc!"); };
template<Peripheral p> struct SignalConnection<DataA12::Rts, p> {
	static_assert((p == Peripheral::Lpuart1) or (p == Peripheral::Usart1),
		"GpioA12::Rts only connects to Lpuart1 or Usart1!"); };
template<Peripheral p> struct SignalConnection<DataA12::Sck, p> {
	static_assert((p == Peripheral::Spi2),"GpioA12::Sck only connects to Spi2!"); };
template<Peripheral p> struct SignalConnection<DataA12::Tx, p> {
	static_assert((p == Peripheral::Uart4) or (p == Peripheral::Fdcan1),
		"GpioA12::Tx only connects to Uart4 or Fdcan1!"); };
template<> struct SignalConnection<DataA12::BitBang, Peripheral::BitBang> { static constexpr int8_t af = -1; };
template<> struct SignalConnection<DataA12::Bkin2, Peripheral::Tim1> { static constexpr int8_t af = 12; };
template<> struct SignalConnection<DataA12::Ck, Peripheral::I2s2> { static constexpr int8_t af = 5; };
template<> struct SignalConnection<DataA12::De, Peripheral::Lpuart1> { static constexpr int8_t af = 3; };
template<> struct SignalConnection<DataA12::De, Peripheral::Usart1> { static constexpr int8_t af = 7; };
template<> struct SignalConnection<DataA12::Dp, Peripheral::Usbotghs> { static constexpr int8_t af = -1; };
template<> struct SignalConnection<DataA12::Etr, Peripheral::Tim1> { static constexpr int8_t af = 1; };
template<> struct SignalConnection<DataA12::Fsb, Peripheral::Sai4> { static constexpr int8_t af = 8; };
template<> struct SignalConnection<DataA12::R5, Peripheral::Ltdc> { static constexpr int8_t af = 14; };
template<> struct SignalConnection<DataA12::Rts, Peripheral::Lpuart1> { static constexpr int8_t af = 3; };
template<> struct SignalConnection<DataA12::Rts, Peripheral::Usart1> { static constexpr int8_t af = 7; };
template<> struct SignalConnection<DataA12::Sck, Peripheral::Spi2> { static constexpr int8_t af = 5; };
template<> struct SignalConnection<DataA12::Tx, Peripheral::Uart4> { static constexpr int8_t af = 6; };
template<> struct SignalConnection<DataA12::Tx, Peripheral::Fdcan1> { static constexpr int8_t af = 9; };

struct DataA13 {
	static constexpr Gpio::Port port = Gpio::Port::A;
	static constexpr uint8_t pin = 13;
	struct BitBang { using Data = DataA13; static constexpr Gpio::Signal Signal = Gpio::Signal::BitBang; };
	struct Jtms { using Data = DataA13; static constexpr Gpio::Signal Signal = Gpio::Signal::Jtms; };
	struct Swdio { using Data = DataA13; static constexpr Gpio::Signal Signal = Gpio::Signal::Swdio; };
};
template<Peripheral p> struct SignalConnection<DataA13::BitBang, p> {
	static_assert(p == Peripheral::BitBang, "GpioA13::BitBang only connects to software drivers!"); };
template<Peripheral p> struct SignalConnection<DataA13::Jtms, p> {
	static_assert((p == Peripheral::Debug),"GpioA13::Jtms only connects to Debug!"); };
template<Peripheral p> struct SignalConnection<DataA13::Swdio, p> {
	static_assert((p == Peripheral::Debug),"GpioA13::Swdio only connects to Debug!"); };
template<> struct SignalConnection<DataA13::BitBang, Peripheral::BitBang> { static constexpr int8_t af = -1; };
template<> struct SignalConnection<DataA13::Jtms, Peripheral::Debug> { static constexpr int8_t af = 0; };
template<> struct SignalConnection<DataA13::Swdio, Peripheral::Debug> { static constexpr int8_t af = 0; };

struct DataA14 {
	static constexpr Gpio::Port port = Gpio::Port::A;
	static constexpr uint8_t pin = 14;
	struct BitBang { using Data = DataA14; static constexpr Gpio::Signal Signal = Gpio::Signal::BitBang; };
	struct Jtck { using Data = DataA14; static constexpr Gpio::Signal Signal = Gpio::Signal::Jtck; };
	struct Swclk { using Data = DataA14; static constexpr Gpio::Signal Signal = Gpio::Signal::Swclk; };
};
template<Peripheral p> struct SignalConnection<DataA14::BitBang, p> {
	static_assert(p == Peripheral::BitBang, "GpioA14::BitBang only connects to software drivers!"); };
template<Peripheral p> struct SignalConnection<DataA14::Jtck, p> {
	static_assert((p == Peripheral::Debug),"GpioA14::Jtck only connects to Debug!"); };
template<Peripheral p> struct SignalConnection<DataA14::Swclk, p> {
	static_assert((p == Peripheral::Debug),"GpioA14::Swclk only connects to Debug!"); };
template<> struct SignalConnection<DataA14::BitBang, Peripheral::BitBang> { static constexpr int8_t af = -1; };
template<> struct SignalConnection<DataA14::Jtck, Peripheral::Debug> { static constexpr int8_t af = 0; };
template<> struct SignalConnection<DataA14::Swclk, Peripheral::Debug> { static constexpr int8_t af = 0; };

struct DataA15 {
	static constexpr Gpio::Port port = Gpio::Port::A;
	static constexpr uint8_t pin = 15;
	struct BitBang { using Data = DataA15; static constexpr Gpio::Signal Signal = Gpio::Signal::BitBang; };
	struct B6 { using Data = DataA15; static constexpr Gpio::Signal Signal = Gpio::Signal::B6; };
	struct Cec { using Data = DataA15; static constexpr Gpio::Signal Signal = Gpio::Signal::Cec; };
	struct Ch1 { using Data = DataA15; static constexpr Gpio::Signal Signal = Gpio::Signal::Ch1; };
	struct De { using Data = DataA15; static constexpr Gpio::Signal Signal = Gpio::Signal::De; };
	struct Etr { using Data = DataA15; static constexpr Gpio::Signal Signal = Gpio::Signal::Etr; };
	struct Jtdi { using Data = DataA15; static constexpr Gpio::Signal Signal = Gpio::Signal::Jtdi; };
	struct Nss { using Data = DataA15; static constexpr Gpio::Signal Signal = Gpio::Signal::Nss; };
	struct R3 { using Data = DataA15; static constexpr Gpio::Signal Signal = Gpio::Signal::R3; };
	struct Rts { using Data = DataA15; static constexpr Gpio::Signal Signal = Gpio::Signal::Rts; };
	struct Tx { using Data = DataA15; static constexpr Gpio::Signal Signal = Gpio::Signal::Tx; };
	struct Ws { using Data = DataA15; static constexpr Gpio::Signal Signal = Gpio::Signal::Ws; };
};
template<Peripheral p> struct SignalConnection<DataA15::BitBang, p> {
	static_assert(p == Peripheral::BitBang, "GpioA15::BitBang only connects to software drivers!"); };
template<Peripheral p> struct SignalConnection<DataA15::B6, p> {
	static_assert((p == Peripheral::Ltdc),"GpioA15::B6 only connects to Ltdc!"); };
template<Peripheral p> struct SignalConnection<DataA15::Cec, p> {
	static_assert((p == Peripheral::Hdmicec),"GpioA15::Cec only connects to Hdmicec!"); };
template<Peripheral p> struct SignalConnection<DataA15::Ch1, p> {
	static_assert((p == Peripheral::Tim2),"GpioA15::Ch1 only connects to Tim2!"); };
template<Peripheral p> struct SignalConnection<DataA15::De, p> {
	static_assert((p == Peripheral::Uart4),"GpioA15::De only connects to Uart4!"); };
template<Peripheral p> struct SignalConnection<DataA15::Etr, p> {
	static_assert((p == Peripheral::Tim2),"GpioA15::Etr only connects to Tim2!"); };
template<Peripheral p> struct SignalConnection<DataA15::Jtdi, p> {
	static_assert((p == Peripheral::Debug),"GpioA15::Jtdi only connects to Debug!"); };
template<Peripheral p> struct SignalConnection<DataA15::Nss, p> {
	static_assert((p == Peripheral::Spi1) or (p == Peripheral::Spi3) or (p == Peripheral::Spi6),
		"GpioA15::Nss only connects to Spi1 or Spi3 or Spi6!"); };
template<Peripheral p> struct SignalConnection<DataA15::R3, p> {
	static_assert((p == Peripheral::Ltdc),"GpioA15::R3 only connects to Ltdc!"); };
template<Peripheral p> struct SignalConnection<DataA15::Rts, p> {
	static_assert((p == Peripheral::Uart4),"GpioA15::Rts only connects to Uart4!"); };
template<Peripheral p> struct SignalConnection<DataA15::Tx, p> {
	static_assert((p == Peripheral::Uart7),"GpioA15::Tx only connects to Uart7!"); };
template<Peripheral p> struct SignalConnection<DataA15::Ws, p> {
	static_assert((p == Peripheral::I2s1) or (p == Peripheral::I2s3) or (p == Peripheral::I2s6),
		"GpioA15::Ws only connects to I2s1 or I2s3 or I2s6!"); };
template<> struct SignalConnection<DataA15::BitBang, Peripheral::BitBang> { static constexpr int8_t af = -1; };
template<> struct SignalConnection<DataA15::B6, Peripheral::Ltdc> { static constexpr int8_t af = 14; };
template<> struct SignalConnection<DataA15::Cec, Peripheral::Hdmicec> { static constexpr int8_t af = 4; };
template<> struct SignalConnection<DataA15::Ch1, Peripheral::Tim2> { static constexpr int8_t af = 1; };
template<> struct SignalConnection<DataA15::De, Peripheral::Uart4> { static constexpr int8_t af = 8; };
template<> struct SignalConnection<DataA15::Etr, Peripheral::Tim2> { static constexpr int8_t af = 1; };
template<> struct SignalConnection<DataA15::Jtdi, Peripheral::Debug> { static constexpr int8_t af = 0; };
template<> struct SignalConnection<DataA15::Nss, Peripheral::Spi1> { static constexpr int8_t af = 5; };
template<> struct SignalConnection<DataA15::Nss, Peripheral::Spi3> { static constexpr int8_t af = 6; };
template<> struct SignalConnection<DataA15::Nss, Peripheral::Spi6> { static constexpr int8_t af = 7; };
template<> struct SignalConnection<DataA15::R3, Peripheral::Ltdc> { static constexpr int8_t af = 9; };
template<> struct SignalConnection<DataA15::Rts, Peripheral::Uart4> { static constexpr int8_t af = 8; };
template<> struct SignalConnection<DataA15::Tx, Peripheral::Uart7> { static constexpr int8_t af = 11; };
template<> struct SignalConnection<DataA15::Ws, Peripheral::I2s1> { static constexpr int8_t af = 5; };
template<> struct SignalConnection<DataA15::Ws, Peripheral::I2s3> { static constexpr int8_t af = 6; };
template<> struct SignalConnection<DataA15::Ws, Peripheral::I2s6> { static constexpr int8_t af = 7; };

struct DataB0 {
	static constexpr Gpio::Port port = Gpio::Port::B;
	static constexpr uint8_t pin = 0;
	struct BitBang { using Data = DataB0; static constexpr Gpio::Signal Signal = Gpio::Signal::BitBang; };
	struct Ch2n { using Data = DataB0; static constexpr Gpio::Signal Signal = Gpio::Signal::Ch2n; };
	struct Ch3 { using Data = DataB0; static constexpr Gpio::Signal Signal = Gpio::Signal::Ch3; };
	struct Ckout { using Data = DataB0; static constexpr Gpio::Signal Signal = Gpio::Signal::Ckout; };
	struct Cts { using Data = DataB0; static constexpr Gpio::Signal Signal = Gpio::Signal::Cts; };
	struct G1 { using Data = DataB0; static constexpr Gpio::Signal Signal = Gpio::Signal::G1; };
	struct Inn5 { using Data = DataB0; static constexpr Gpio::Signal Signal = Gpio::Signal::Inn5; };
	struct Inp { using Data = DataB0; static constexpr Gpio::Signal Signal = Gpio::Signal::Inp; };
	struct Inp9 { using Data = DataB0; static constexpr Gpio::Signal Signal = Gpio::Signal::Inp9; };
	struct P1io1 { using Data = DataB0; static constexpr Gpio::Signal Signal = Gpio::Signal::P1io1; };
	struct R3 { using Data = DataB0; static constexpr Gpio::Signal Signal = Gpio::Signal::R3; };
	struct Rxd2 { using Data = DataB0; static constexpr Gpio::Signal Signal = Gpio::Signal::Rxd2; };
	struct Ulpid1 { using Data = DataB0; static constexpr Gpio::Signal Signal = Gpio::Signal::Ulpid1; };
	struct Vinp { using Data = DataB0; static constexpr Gpio::Signal Signal = Gpio::Signal::Vinp; };
};
template<Peripheral p> struct SignalConnection<DataB0::BitBang, p> {
	static_assert(p == Peripheral::BitBang, "GpioB0::BitBang only connects to software drivers!"); };
template<Peripheral p> struct SignalConnection<DataB0::Ch2n, p> {
	static_assert((p == Peripheral::Tim1) or (p == Peripheral::Tim8),
		"GpioB0::Ch2n only connects to Tim1 or Tim8!"); };
template<Peripheral p> struct SignalConnection<DataB0::Ch3, p> {
	static_assert((p == Peripheral::Tim3),"GpioB0::Ch3 only connects to Tim3!"); };
template<Peripheral p> struct SignalConnection<DataB0::Ckout, p> {
	static_assert((p == Peripheral::Dfsdm1),"GpioB0::Ckout only connects to Dfsdm1!"); };
template<Peripheral p> struct SignalConnection<DataB0::Cts, p> {
	static_assert((p == Peripheral::Uart4),"GpioB0::Cts only connects to Uart4!"); };
template<Peripheral p> struct SignalConnection<DataB0::G1, p> {
	static_assert((p == Peripheral::Ltdc),"GpioB0::G1 only connects to Ltdc!"); };
template<Peripheral p> struct SignalConnection<DataB0::Inn5, p> {
	static_assert((p == Peripheral::Adc1) or (p == Peripheral::Adc2),
		"GpioB0::Inn5 only connects to Adc1 or Adc2!"); };
template<Peripheral p> struct SignalConnection<DataB0::Inp, p> {
	static_assert((p == Peripheral::Comp1),"GpioB0::Inp only connects to Comp1!"); };
template<Peripheral p> struct SignalConnection<DataB0::Inp9, p> {
	static_assert((p == Peripheral::Adc1) or (p == Peripheral::Adc2),
		"GpioB0::Inp9 only connects to Adc1 or Adc2!"); };
template<Peripheral p> struct SignalConnection<DataB0::P1io1, p> {
	static_assert((p == Peripheral::Octospim),"GpioB0::P1io1 only connects to Octospim!"); };
template<Peripheral p> struct SignalConnection<DataB0::R3, p> {
	static_assert((p == Peripheral::Ltdc),"GpioB0::R3 only connects to Ltdc!"); };
template<Peripheral p> struct SignalConnection<DataB0::Rxd2, p> {
	static_assert((p == Peripheral::Eth),"GpioB0::Rxd2 only connects to Eth!"); };
template<Peripheral p> struct SignalConnection<DataB0::Ulpid1, p> {
	static_assert((p == Peripheral::Usbotghs),"GpioB0::Ulpid1 only connects to Usbotghs!"); };
template<Peripheral p> struct SignalConnection<DataB0::Vinp, p> {
	static_assert((p == Peripheral::Opamp1),"GpioB0::Vinp only connects to Opamp1!"); };
template<> struct SignalConnection<DataB0::BitBang, Peripheral::BitBang> { static constexpr int8_t af = -1; };
template<> struct SignalConnection<DataB0::Ch2n, Peripheral::Tim1> { static constexpr int8_t af = 1; };
template<> struct SignalConnection<DataB0::Ch2n, Peripheral::Tim8> { static constexpr int8_t af = 3; };
template<> struct SignalConnection<DataB0::Ch3, Peripheral::Tim3> { static constexpr int8_t af = 2; };
template<> struct SignalConnection<DataB0::Ckout, Peripheral::Dfsdm1> { static constexpr int8_t af = 6; };
template<> struct SignalConnection<DataB0::Cts, Peripheral::Uart4> { static constexpr int8_t af = 8; };
template<> struct SignalConnection<DataB0::G1, Peripheral::Ltdc> { static constexpr int8_t af = 14; };
template<> struct SignalConnection<DataB0::Inn5, Peripheral::Adc1> { static constexpr int8_t af = -2; };
template<> struct SignalConnection<DataB0::Inn5, Peripheral::Adc2> { static constexpr int8_t af = -2; };
template<> struct SignalConnection<DataB0::Inp, Peripheral::Comp1> { static constexpr int8_t af = -2; };
template<> struct SignalConnection<DataB0::Inp9, Peripheral::Adc1> { static constexpr int8_t af = -2; };
template<> struct SignalConnection<DataB0::Inp9, Peripheral::Adc2> { static constexpr int8_t af = -2; };
template<> struct SignalConnection<DataB0::P1io1, Peripheral::Octospim> { static constexpr int8_t af = 4; };
template<> struct SignalConnection<DataB0::R3, Peripheral::Ltdc> { static constexpr int8_t af = 9; };
template<> struct SignalConnection<DataB0::Rxd2, Peripheral::Eth> { static constexpr int8_t af = 11; };
template<> struct SignalConnection<DataB0::Ulpid1, Peripheral::Usbotghs> { static constexpr int8_t af = 10; };
template<> struct SignalConnection<DataB0::Vinp, Peripheral::Opamp1> { static constexpr int8_t af = -1; };
template<> constexpr inline int8_t AdcChannel<DataB0, Peripheral::Adc1, AdcPolarity::Negative> = 5;
template<> constexpr inline int8_t AdcChannel<DataB0, Peripheral::Adc2, AdcPolarity::Negative> = 5;
template<> constexpr inline int8_t AdcChannel<DataB0, Peripheral::Adc1, AdcPolarity::Positive> = 9;
template<> constexpr inline int8_t AdcChannel<DataB0, Peripheral::Adc2, AdcPolarity::Positive> = 9;

struct DataB1 {
	static constexpr Gpio::Port port = Gpio::Port::B;
	static constexpr uint8_t pin = 1;
	struct BitBang { using Data = DataB1; static constexpr Gpio::Signal Signal = Gpio::Signal::BitBang; };
	struct Ch3n { using Data = DataB1; static constexpr Gpio::Signal Signal = Gpio::Signal::Ch3n; };
	struct Ch4 { using Data = DataB1; static constexpr Gpio::Signal Signal = Gpio::Signal::Ch4; };
	struct Datin1 { using Data = DataB1; static constexpr Gpio::Signal Signal = Gpio::Signal::Datin1; };
	struct G0 { using Data = DataB1; static constexpr Gpio::Signal Signal = Gpio::Signal::G0; };
	struct Inm { using Data = DataB1; static constexpr Gpio::Signal Signal = Gpio::Signal::Inm; };
	struct Inp5 { using Data = DataB1; static constexpr Gpio::Signal Signal = Gpio::Signal::Inp5; };
	struct P1io0 { using Data = DataB1; static constexpr Gpio::Signal Signal = Gpio::Signal::P1io0; };
	struct R6 { using Data = DataB1; static constexpr Gpio::Signal Signal = Gpio::Signal::R6; };
	struct Rxd3 { using Data = DataB1; static constexpr Gpio::Signal Signal = Gpio::Signal::Rxd3; };
	struct Ulpid2 { using Data = DataB1; static constexpr Gpio::Signal Signal = Gpio::Signal::Ulpid2; };
};
template<Peripheral p> struct SignalConnection<DataB1::BitBang, p> {
	static_assert(p == Peripheral::BitBang, "GpioB1::BitBang only connects to software drivers!"); };
template<Peripheral p> struct SignalConnection<DataB1::Ch3n, p> {
	static_assert((p == Peripheral::Tim1) or (p == Peripheral::Tim8),
		"GpioB1::Ch3n only connects to Tim1 or Tim8!"); };
template<Peripheral p> struct SignalConnection<DataB1::Ch4, p> {
	static_assert((p == Peripheral::Tim3),"GpioB1::Ch4 only connects to Tim3!"); };
template<Peripheral p> struct SignalConnection<DataB1::Datin1, p> {
	static_assert((p == Peripheral::Dfsdm1),"GpioB1::Datin1 only connects to Dfsdm1!"); };
template<Peripheral p> struct SignalConnection<DataB1::G0, p> {
	static_assert((p == Peripheral::Ltdc),"GpioB1::G0 only connects to Ltdc!"); };
template<Peripheral p> struct SignalConnection<DataB1::Inm, p> {
	static_assert((p == Peripheral::Comp1),"GpioB1::Inm only connects to Comp1!"); };
template<Peripheral p> struct SignalConnection<DataB1::Inp5, p> {
	static_assert((p == Peripheral::Adc1) or (p == Peripheral::Adc2),
		"GpioB1::Inp5 only connects to Adc1 or Adc2!"); };
template<Peripheral p> struct SignalConnection<DataB1::P1io0, p> {
	static_assert((p == Peripheral::Octospim),"GpioB1::P1io0 only connects to Octospim!"); };
template<Peripheral p> struct SignalConnection<DataB1::R6, p> {
	static_assert((p == Peripheral::Ltdc),"GpioB1::R6 only connects to Ltdc!"); };
template<Peripheral p> struct SignalConnection<DataB1::Rxd3, p> {
	static_assert((p == Peripheral::Eth),"GpioB1::Rxd3 only connects to Eth!"); };
template<Peripheral p> struct SignalConnection<DataB1::Ulpid2, p> {
	static_assert((p == Peripheral::Usbotghs),"GpioB1::Ulpid2 only connects to Usbotghs!"); };
template<> struct SignalConnection<DataB1::BitBang, Peripheral::BitBang> { static constexpr int8_t af = -1; };
template<> struct SignalConnection<DataB1::Ch3n, Peripheral::Tim1> { static constexpr int8_t af = 1; };
template<> struct SignalConnection<DataB1::Ch3n, Peripheral::Tim8> { static constexpr int8_t af = 3; };
template<> struct SignalConnection<DataB1::Ch4, Peripheral::Tim3> { static constexpr int8_t af = 2; };
template<> struct SignalConnection<DataB1::Datin1, Peripheral::Dfsdm1> { static constexpr int8_t af = 6; };
template<> struct SignalConnection<DataB1::G0, Peripheral::Ltdc> { static constexpr int8_t af = 14; };
template<> struct SignalConnection<DataB1::Inm, Peripheral::Comp1> { static constexpr int8_t af = -2; };
template<> struct SignalConnection<DataB1::Inp5, Peripheral::Adc1> { static constexpr int8_t af = -2; };
template<> struct SignalConnection<DataB1::Inp5, Peripheral::Adc2> { static constexpr int8_t af = -2; };
template<> struct SignalConnection<DataB1::P1io0, Peripheral::Octospim> { static constexpr int8_t af = 4; };
template<> struct SignalConnection<DataB1::R6, Peripheral::Ltdc> { static constexpr int8_t af = 9; };
template<> struct SignalConnection<DataB1::Rxd3, Peripheral::Eth> { static constexpr int8_t af = 11; };
template<> struct SignalConnection<DataB1::Ulpid2, Peripheral::Usbotghs> { static constexpr int8_t af = 10; };
template<> constexpr inline int8_t AdcChannel<DataB1, Peripheral::Adc1, AdcPolarity::Positive> = 5;
template<> constexpr inline int8_t AdcChannel<DataB1, Peripheral::Adc2, AdcPolarity::Positive> = 5;

struct DataB2 {
	static constexpr Gpio::Port port = Gpio::Port::B;
	static constexpr uint8_t pin = 2;
	struct BitBang { using Data = DataB2; static constexpr Gpio::Signal Signal = Gpio::Signal::BitBang; };
	struct Ckin1 { using Data = DataB2; static constexpr Gpio::Signal Signal = Gpio::Signal::Ckin1; };
	struct D1 { using Data = DataB2; static constexpr Gpio::Signal Signal = Gpio::Signal::D1; };
	struct Etr { using Data = DataB2; static constexpr Gpio::Signal Signal = Gpio::Signal::Etr; };
	struct Inp { using Data = DataB2; static constexpr Gpio::Signal Signal = Gpio::Signal::Inp; };
	struct Mosi { using Data = DataB2; static constexpr Gpio::Signal Signal = Gpio::Signal::Mosi; };
	struct Outalarm { using Data = DataB2; static constexpr Gpio::Signal Signal = Gpio::Signal::Outalarm; };
	struct Outcalib { using Data = DataB2; static constexpr Gpio::Signal Signal = Gpio::Signal::Outcalib; };
	struct P1clk { using Data = DataB2; static constexpr Gpio::Signal Signal = Gpio::Signal::P1clk; };
	struct P1dqs { using Data = DataB2; static constexpr Gpio::Signal Signal = Gpio::Signal::P1dqs; };
	struct Sda { using Data = DataB2; static constexpr Gpio::Signal Signal = Gpio::Signal::Sda; };
	struct Sdo { using Data = DataB2; static constexpr Gpio::Signal Signal = Gpio::Signal::Sdo; };
	struct Txer { using Data = DataB2; static constexpr Gpio::Signal Signal = Gpio::Signal::Txer; };
};
template<Peripheral p> struct SignalConnection<DataB2::BitBang, p> {
	static_assert(p == Peripheral::BitBang, "GpioB2::BitBang only connects to software drivers!"); };
template<Peripheral p> struct SignalConnection<DataB2::Ckin1, p> {
	static_assert((p == Peripheral::Dfsdm1),"GpioB2::Ckin1 only connects to Dfsdm1!"); };
template<Peripheral p> struct SignalConnection<DataB2::D1, p> {
	static_assert((p == Peripheral::Sai4) or (p == Peripheral::Sai1),
		"GpioB2::D1 only connects to Sai4 or Sai1!"); };
template<Peripheral p> struct SignalConnection<DataB2::Etr, p> {
	static_assert((p == Peripheral::Tim23),"GpioB2::Etr only connects to Tim23!"); };
template<Peripheral p> struct SignalConnection<DataB2::Inp, p> {
	static_assert((p == Peripheral::Comp1),"GpioB2::Inp only connects to Comp1!"); };
template<Peripheral p> struct SignalConnection<DataB2::Mosi, p> {
	static_assert((p == Peripheral::Spi3),"GpioB2::Mosi only connects to Spi3!"); };
template<Peripheral p> struct SignalConnection<DataB2::Outalarm, p> {
	static_assert((p == Peripheral::Rtc),"GpioB2::Outalarm only connects to Rtc!"); };
template<Peripheral p> struct SignalConnection<DataB2::Outcalib, p> {
	static_assert((p == Peripheral::Rtc),"GpioB2::Outcalib only connects to Rtc!"); };
template<Peripheral p> struct SignalConnection<DataB2::P1clk, p> {
	static_assert((p == Peripheral::Octospim),"GpioB2::P1clk only connects to Octospim!"); };
template<Peripheral p> struct SignalConnection<DataB2::P1dqs, p> {
	static_assert((p == Peripheral::Octospim),"GpioB2::P1dqs only connects to Octospim!"); };
template<Peripheral p> struct SignalConnection<DataB2::Sda, p> {
	static_assert((p == Peripheral::Sai1) or (p == Peripheral::Sai4),
		"GpioB2::Sda only connects to Sai1 or Sai4!"); };
template<Peripheral p> struct SignalConnection<DataB2::Sdo, p> {
	static_assert((p == Peripheral::I2s3),"GpioB2::Sdo only connects to I2s3!"); };
template<Peripheral p> struct SignalConnection<DataB2::Txer, p> {
	static_assert((p == Peripheral::Eth),"GpioB2::Txer only connects to Eth!"); };
template<> struct SignalConnection<DataB2::BitBang, Peripheral::BitBang> { static constexpr int8_t af = -1; };
template<> struct SignalConnection<DataB2::Ckin1, Peripheral::Dfsdm1> { static constexpr int8_t af = 4; };
template<> struct SignalConnection<DataB2::D1, Peripheral::Sai4> { static constexpr int8_t af = 1; };
template<> struct SignalConnection<DataB2::D1, Peripheral::Sai1> { static constexpr int8_t af = 2; };
template<> struct SignalConnection<DataB2::Etr, Peripheral::Tim23> { static constexpr int8_t af = 13; };
template<> struct SignalConnection<DataB2::Inp, Peripheral::Comp1> { static constexpr int8_t af = -2; };
template<> struct SignalConnection<DataB2::Mosi, Peripheral::Spi3> { static constexpr int8_t af = 7; };
template<> struct SignalConnection<DataB2::Outalarm, Peripheral::Rtc> { static constexpr int8_t af = 0; };
template<> struct SignalConnection<DataB2::Outcalib, Peripheral::Rtc> { static constexpr int8_t af = 0; };
template<> struct SignalConnection<DataB2::P1clk, Peripheral::Octospim> { static constexpr int8_t af = 9; };
template<> struct SignalConnection<DataB2::P1dqs, Peripheral::Octospim> { static constexpr int8_t af = 10; };
template<> struct SignalConnection<DataB2::Sda, Peripheral::Sai1> { static constexpr int8_t af = 6; };
template<> struct SignalConnection<DataB2::Sda, Peripheral::Sai4> { static constexpr int8_t af = 8; };
template<> struct SignalConnection<DataB2::Sdo, Peripheral::I2s3> { static constexpr int8_t af = 7; };
template<> struct SignalConnection<DataB2::Txer, Peripheral::Eth> { static constexpr int8_t af = 11; };

struct DataB3 {
	static constexpr Gpio::Port port = Gpio::Port::B;
	static constexpr uint8_t pin = 3;
	struct BitBang { using Data = DataB3; static constexpr Gpio::Signal Signal = Gpio::Signal::BitBang; };
	struct Ch2 { using Data = DataB3; static constexpr Gpio::Signal Signal = Gpio::Signal::Ch2; };
	struct Ck { using Data = DataB3; static constexpr Gpio::Signal Signal = Gpio::Signal::Ck; };
	struct Crssync { using Data = DataB3; static constexpr Gpio::Signal Signal = Gpio::Signal::Crssync; };
	struct D2 { using Data = DataB3; static constexpr Gpio::Signal Signal = Gpio::Signal::D2; };
	struct Etr { using Data = DataB3; static constexpr Gpio::Signal Signal = Gpio::Signal::Etr; };
	struct Jtdo { using Data = DataB3; static constexpr Gpio::Signal Signal = Gpio::Signal::Jtdo; };
	struct Rx { using Data = DataB3; static constexpr Gpio::Signal Signal = Gpio::Signal::Rx; };
	struct Sck { using Data = DataB3; static constexpr Gpio::Signal Signal = Gpio::Signal::Sck; };
	struct Swo { using Data = DataB3; static constexpr Gpio::Signal Signal = Gpio::Signal::Swo; };
};
template<Peripheral p> struct SignalConnection<DataB3::BitBang, p> {
	static_assert(p == Peripheral::BitBang, "GpioB3::BitBang only connects to software drivers!"); };
template<Peripheral p> struct SignalConnection<DataB3::Ch2, p> {
	static_assert((p == Peripheral::Tim2),"GpioB3::Ch2 only connects to Tim2!"); };
template<Peripheral p> struct SignalConnection<DataB3::Ck, p> {
	static_assert((p == Peripheral::I2s1) or (p == Peripheral::I2s3) or (p == Peripheral::I2s6),
		"GpioB3::Ck only connects to I2s1 or I2s3 or I2s6!"); };
template<Peripheral p> struct SignalConnection<DataB3::Crssync, p> {
	static_assert((p == Peripheral::Rcc),"GpioB3::Crssync only connects to Rcc!"); };
template<Peripheral p> struct SignalConnection<DataB3::D2, p> {
	static_assert((p == Peripheral::Sdmmc2),"GpioB3::D2 only connects to Sdmmc2!"); };
template<Peripheral p> struct SignalConnection<DataB3::Etr, p> {
	static_assert((p == Peripheral::Tim24),"GpioB3::Etr only connects to Tim24!"); };
template<Peripheral p> struct SignalConnection<DataB3::Jtdo, p> {
	static_assert((p == Peripheral::Debug),"GpioB3::Jtdo only connects to Debug!"); };
template<Peripheral p> struct SignalConnection<DataB3::Rx, p> {
	static_assert((p == Peripheral::Uart7),"GpioB3::Rx only connects to Uart7!"); };
template<Peripheral p> struct SignalConnection<DataB3::Sck, p> {
	static_assert((p == Peripheral::Spi1) or (p == Peripheral::Spi3) or (p == Peripheral::Spi6),
		"GpioB3::Sck only connects to Spi1 or Spi3 or Spi6!"); };
template<Peripheral p> struct SignalConnection<DataB3::Swo, p> {
	static_assert((p == Peripheral::Debug),"GpioB3::Swo only connects to Debug!"); };
template<> struct SignalConnection<DataB3::BitBang, Peripheral::BitBang> { static constexpr int8_t af = -1; };
template<> struct SignalConnection<DataB3::Ch2, Peripheral::Tim2> { static constexpr int8_t af = 1; };
template<> struct SignalConnection<DataB3::Ck, Peripheral::I2s1> { static constexpr int8_t af = 5; };
template<> struct SignalConnection<DataB3::Ck, Peripheral::I2s3> { static constexpr int8_t af = 6; };
template<> struct SignalConnection<DataB3::Ck, Peripheral::I2s6> { static constexpr int8_t af = 8; };
template<> struct SignalConnection<DataB3::Crssync, Peripheral::Rcc> { static constexpr int8_t af = 10; };
template<> struct SignalConnection<DataB3::D2, Peripheral::Sdmmc2> { static constexpr int8_t af = 9; };
template<> struct SignalConnection<DataB3::Etr, Peripheral::Tim24> { static constexpr int8_t af = 14; };
template<> struct SignalConnection<DataB3::Jtdo, Peripheral::Debug> { static constexpr int8_t af = 0; };
template<> struct SignalConnection<DataB3::Rx, Peripheral::Uart7> { static constexpr int8_t af = 11; };
template<> struct SignalConnection<DataB3::Sck, Peripheral::Spi1> { static constexpr int8_t af = 5; };
template<> struct SignalConnection<DataB3::Sck, Peripheral::Spi3> { static constexpr int8_t af = 6; };
template<> struct SignalConnection<DataB3::Sck, Peripheral::Spi6> { static constexpr int8_t af = 8; };
template<> struct SignalConnection<DataB3::Swo, Peripheral::Debug> { static constexpr int8_t af = 0; };

struct DataB4 {
	static constexpr Gpio::Port port = Gpio::Port::B;
	static constexpr uint8_t pin = 4;
	struct BitBang { using Data = DataB4; static constexpr Gpio::Signal Signal = Gpio::Signal::BitBang; };
	struct Bkin { using Data = DataB4; static constexpr Gpio::Signal Signal = Gpio::Signal::Bkin; };
	struct Ch1 { using Data = DataB4; static constexpr Gpio::Signal Signal = Gpio::Signal::Ch1; };
	struct D3 { using Data = DataB4; static constexpr Gpio::Signal Signal = Gpio::Signal::D3; };
	struct Jtrst { using Data = DataB4; static constexpr Gpio::Signal Signal = Gpio::Signal::Jtrst; };
	struct Miso { using Data = DataB4; static constexpr Gpio::Signal Signal = Gpio::Signal::Miso; };
	struct Nss { using Data = DataB4; static constexpr Gpio::Signal Signal = Gpio::Signal::Nss; };
	struct Sdi { using Data = DataB4; static constexpr Gpio::Signal Signal = Gpio::Signal::Sdi; };
	struct Tx { using Data = DataB4; static constexpr Gpio::Signal Signal = Gpio::Signal::Tx; };
	struct Ws { using Data = DataB4; static constexpr Gpio::Signal Signal = Gpio::Signal::Ws; };
};
template<Peripheral p> struct SignalConnection<DataB4::BitBang, p> {
	static_assert(p == Peripheral::BitBang, "GpioB4::BitBang only connects to software drivers!"); };
template<Peripheral p> struct SignalConnection<DataB4::Bkin, p> {
	static_assert((p == Peripheral::Tim16),"GpioB4::Bkin only connects to Tim16!"); };
template<Peripheral p> struct SignalConnection<DataB4::Ch1, p> {
	static_assert((p == Peripheral::Tim3),"GpioB4::Ch1 only connects to Tim3!"); };
template<Peripheral p> struct SignalConnection<DataB4::D3, p> {
	static_assert((p == Peripheral::Sdmmc2),"GpioB4::D3 only connects to Sdmmc2!"); };
template<Peripheral p> struct SignalConnection<DataB4::Jtrst, p> {
	static_assert((p == Peripheral::Debug),"GpioB4::Jtrst only connects to Debug!"); };
template<Peripheral p> struct SignalConnection<DataB4::Miso, p> {
	static_assert((p == Peripheral::Spi1) or (p == Peripheral::Spi3) or (p == Peripheral::Spi6),
		"GpioB4::Miso only connects to Spi1 or Spi3 or Spi6!"); };
template<Peripheral p> struct SignalConnection<DataB4::Nss, p> {
	static_assert((p == Peripheral::Spi2),"GpioB4::Nss only connects to Spi2!"); };
template<Peripheral p> struct SignalConnection<DataB4::Sdi, p> {
	static_assert((p == Peripheral::I2s1) or (p == Peripheral::I2s3) or (p == Peripheral::I2s6),
		"GpioB4::Sdi only connects to I2s1 or I2s3 or I2s6!"); };
template<Peripheral p> struct SignalConnection<DataB4::Tx, p> {
	static_assert((p == Peripheral::Uart7),"GpioB4::Tx only connects to Uart7!"); };
template<Peripheral p> struct SignalConnection<DataB4::Ws, p> {
	static_assert((p == Peripheral::I2s2),"GpioB4::Ws only connects to I2s2!"); };
template<> struct SignalConnection<DataB4::BitBang, Peripheral::BitBang> { static constexpr int8_t af = -1; };
template<> struct SignalConnection<DataB4::Bkin, Peripheral::Tim16> { static constexpr int8_t af = 1; };
template<> struct SignalConnection<DataB4::Ch1, Peripheral::Tim3> { static constexpr int8_t af = 2; };
template<> struct SignalConnection<DataB4::D3, Peripheral::Sdmmc2> { static constexpr int8_t af = 9; };
template<> struct SignalConnection<DataB4::Jtrst, Peripheral::Debug> { static constexpr int8_t af = 0; };
template<> struct SignalConnection<DataB4::Miso, Peripheral::Spi1> { static constexpr int8_t af = 5; };
template<> struct SignalConnection<DataB4::Miso, Peripheral::Spi3> { static constexpr int8_t af = 6; };
template<> struct SignalConnection<DataB4::Miso, Peripheral::Spi6> { static constexpr int8_t af = 8; };
template<> struct SignalConnection<DataB4::Nss, Peripheral::Spi2> { static constexpr int8_t af = 7; };
template<> struct SignalConnection<DataB4::Sdi, Peripheral::I2s1> { static constexpr int8_t af = 5; };
template<> struct SignalConnection<DataB4::Sdi, Peripheral::I2s3> { static constexpr int8_t af = 6; };
template<> struct SignalConnection<DataB4::Sdi, Peripheral::I2s6> { static constexpr int8_t af = 8; };
template<> struct SignalConnection<DataB4::Tx, Peripheral::Uart7> { static constexpr int8_t af = 11; };
template<> struct SignalConnection<DataB4::Ws, Peripheral::I2s2> { static constexpr int8_t af = 7; };

struct DataB5 {
	static constexpr Gpio::Port port = Gpio::Port::B;
	static constexpr uint8_t pin = 5;
	struct BitBang { using Data = DataB5; static constexpr Gpio::Signal Signal = Gpio::Signal::BitBang; };
	struct B5 { using Data = DataB5; static constexpr Gpio::Signal Signal = Gpio::Signal::B5; };
	struct Bkin { using Data = DataB5; static constexpr Gpio::Signal Signal = Gpio::Signal::Bkin; };
	struct Ch2 { using Data = DataB5; static constexpr Gpio::Signal Signal = Gpio::Signal::Ch2; };
	struct D10 { using Data = DataB5; static constexpr Gpio::Signal Signal = Gpio::Signal::D10; };
	struct Mosi { using Data = DataB5; static constexpr Gpio::Signal Signal = Gpio::Signal::Mosi; };
	struct Ppsout { using Data = DataB5; static constexpr Gpio::Signal Signal = Gpio::Signal::Ppsout; };
	struct Rx { using Data = DataB5; static constexpr Gpio::Signal Signal = Gpio::Signal::Rx; };
	struct Sdcke1 { using Data = DataB5; static constexpr Gpio::Signal Signal = Gpio::Signal::Sdcke1; };
	struct Sdo { using Data = DataB5; static constexpr Gpio::Signal Signal = Gpio::Signal::Sdo; };
	struct Smba { using Data = DataB5; static constexpr Gpio::Signal Signal = Gpio::Signal::Smba; };
	struct Ulpid7 { using Data = DataB5; static constexpr Gpio::Signal Signal = Gpio::Signal::Ulpid7; };
};
template<Peripheral p> struct SignalConnection<DataB5::BitBang, p> {
	static_assert(p == Peripheral::BitBang, "GpioB5::BitBang only connects to software drivers!"); };
template<Peripheral p> struct SignalConnection<DataB5::B5, p> {
	static_assert((p == Peripheral::Ltdc),"GpioB5::B5 only connects to Ltdc!"); };
template<Peripheral p> struct SignalConnection<DataB5::Bkin, p> {
	static_assert((p == Peripheral::Tim17),"GpioB5::Bkin only connects to Tim17!"); };
template<Peripheral p> struct SignalConnection<DataB5::Ch2, p> {
	static_assert((p == Peripheral::Tim3),"GpioB5::Ch2 only connects to Tim3!"); };
template<Peripheral p> struct SignalConnection<DataB5::D10, p> {
	static_assert((p == Peripheral::Dcmi) or (p == Peripheral::Pssi),
		"GpioB5::D10 only connects to Dcmi or Pssi!"); };
template<Peripheral p> struct SignalConnection<DataB5::Mosi, p> {
	static_assert((p == Peripheral::Spi1) or (p == Peripheral::Spi3) or (p == Peripheral::Spi6),
		"GpioB5::Mosi only connects to Spi1 or Spi3 or Spi6!"); };
template<Peripheral p> struct SignalConnection<DataB5::Ppsout, p> {
	static_assert((p == Peripheral::Eth),"GpioB5::Ppsout only connects to Eth!"); };
template<Peripheral p> struct SignalConnection<DataB5::Rx, p> {
	static_assert((p == Peripheral::Fdcan2) or (p == Peripheral::Uart5),
		"GpioB5::Rx only connects to Fdcan2 or Uart5!"); };
template<Peripheral p> struct SignalConnection<DataB5::Sdcke1, p> {
	static_assert((p == Peripheral::Fmc),"GpioB5::Sdcke1 only connects to Fmc!"); };
template<Peripheral p> struct SignalConnection<DataB5::Sdo, p> {
	static_assert((p == Peripheral::I2s1) or (p == Peripheral::I2s3) or (p == Peripheral::I2s6),
		"GpioB5::Sdo only connects to I2s1 or I2s3 or I2s6!"); };
template<Peripheral p> struct SignalConnection<DataB5::Smba, p> {
	static_assert((p == Peripheral::I2c1) or (p == Peripheral::I2c4),
		"GpioB5::Smba only connects to I2c1 or I2c4!"); };
template<Peripheral p> struct SignalConnection<DataB5::Ulpid7, p> {
	static_assert((p == Peripheral::Usbotghs),"GpioB5::Ulpid7 only connects to Usbotghs!"); };
template<> struct SignalConnection<DataB5::BitBang, Peripheral::BitBang> { static constexpr int8_t af = -1; };
template<> struct SignalConnection<DataB5::B5, Peripheral::Ltdc> { static constexpr int8_t af = 3; };
template<> struct SignalConnection<DataB5::Bkin, Peripheral::Tim17> { static constexpr int8_t af = 1; };
template<> struct SignalConnection<DataB5::Ch2, Peripheral::Tim3> { static constexpr int8_t af = 2; };
template<> struct SignalConnection<DataB5::D10, Peripheral::Dcmi> { static constexpr int8_t af = 13; };
template<> struct SignalConnection<DataB5::D10, Peripheral::Pssi> { static constexpr int8_t af = 13; };
template<> struct SignalConnection<DataB5::Mosi, Peripheral::Spi1> { static constexpr int8_t af = 5; };
template<> struct SignalConnection<DataB5::Mosi, Peripheral::Spi3> { static constexpr int8_t af = 7; };
template<> struct SignalConnection<DataB5::Mosi, Peripheral::Spi6> { static constexpr int8_t af = 8; };
template<> struct SignalConnection<DataB5::Ppsout, Peripheral::Eth> { static constexpr int8_t af = 11; };
template<> struct SignalConnection<DataB5::Rx, Peripheral::Fdcan2> { static constexpr int8_t af = 9; };
template<> struct SignalConnection<DataB5::Rx, Peripheral::Uart5> { static constexpr int8_t af = 14; };
template<> struct SignalConnection<DataB5::Sdcke1, Peripheral::Fmc> { static constexpr int8_t af = 12; };
template<> struct SignalConnection<DataB5::Sdo, Peripheral::I2s1> { static constexpr int8_t af = 5; };
template<> struct SignalConnection<DataB5::Sdo, Peripheral::I2s3> { static constexpr int8_t af = 7; };
template<> struct SignalConnection<DataB5::Sdo, Peripheral::I2s6> { static constexpr int8_t af = 8; };
template<> struct SignalConnection<DataB5::Smba, Peripheral::I2c1> { static constexpr int8_t af = 4; };
template<> struct SignalConnection<DataB5::Smba, Peripheral::I2c4> { static constexpr int8_t af = 6; };
template<> struct SignalConnection<DataB5::Ulpid7, Peripheral::Usbotghs> { static constexpr int8_t af = 10; };

struct DataB6 {
	static constexpr Gpio::Port port = Gpio::Port::B;
	static constexpr uint8_t pin = 6;
	struct BitBang { using Data = DataB6; static constexpr Gpio::Signal Signal = Gpio::Signal::BitBang; };
	struct Cec { using Data = DataB6; static constexpr Gpio::Signal Signal = Gpio::Signal::Cec; };
	struct Ch1 { using Data = DataB6; static constexpr Gpio::Signal Signal = Gpio::Signal::Ch1; };
	struct Ch1n { using Data = DataB6; static constexpr Gpio::Signal Signal = Gpio::Signal::Ch1n; };
	struct D5 { using Data = DataB6; static constexpr Gpio::Signal Signal = Gpio::Signal::D5; };
	struct Datin5 { using Data = DataB6; static constexpr Gpio::Signal Signal = Gpio::Signal::Datin5; };
	struct P1ncs { using Data = DataB6; static constexpr Gpio::Signal Signal = Gpio::Signal::P1ncs; };
	struct Scl { using Data = DataB6; static constexpr Gpio::Signal Signal = Gpio::Signal::Scl; };
	struct Sdne1 { using Data = DataB6; static constexpr Gpio::Signal Signal = Gpio::Signal::Sdne1; };
	struct Tx { using Data = DataB6; static constexpr Gpio::Signal Signal = Gpio::Signal::Tx; };
};
template<Peripheral p> struct SignalConnection<DataB6::BitBang, p> {
	static_assert(p == Peripheral::BitBang, "GpioB6::BitBang only connects to software drivers!"); };
template<Peripheral p> struct SignalConnection<DataB6::Cec, p> {
	static_assert((p == Peripheral::Hdmicec),"GpioB6::Cec only connects to Hdmicec!"); };
template<Peripheral p> struct SignalConnection<DataB6::Ch1, p> {
	static_assert((p == Peripheral::Tim4),"GpioB6::Ch1 only connects to Tim4!"); };
template<Peripheral p> struct SignalConnection<DataB6::Ch1n, p> {
	static_assert((p == Peripheral::Tim16),"GpioB6::Ch1n only connects to Tim16!"); };
template<Peripheral p> struct SignalConnection<DataB6::D5, p> {
	static_assert((p == Peripheral::Dcmi) or (p == Peripheral::Pssi),
		"GpioB6::D5 only connects to Dcmi or Pssi!"); };
template<Peripheral p> struct SignalConnection<DataB6::Datin5, p> {
	static_assert((p == Peripheral::Dfsdm1),"GpioB6::Datin5 only connects to Dfsdm1!"); };
template<Peripheral p> struct SignalConnection<DataB6::P1ncs, p> {
	static_assert((p == Peripheral::Octospim),"GpioB6::P1ncs only connects to Octospim!"); };
template<Peripheral p> struct SignalConnection<DataB6::Scl, p> {
	static_assert((p == Peripheral::I2c1) or (p == Peripheral::I2c4),
		"GpioB6::Scl only connects to I2c1 or I2c4!"); };
template<Peripheral p> struct SignalConnection<DataB6::Sdne1, p> {
	static_assert((p == Peripheral::Fmc),"GpioB6::Sdne1 only connects to Fmc!"); };
template<Peripheral p> struct SignalConnection<DataB6::Tx, p> {
	static_assert((p == Peripheral::Usart1) or (p == Peripheral::Lpuart1) or (p == Peripheral::Fdcan2) or (p == Peripheral::Uart5),
		"GpioB6::Tx only connects to Usart1 or Lpuart1 or Fdcan2 or Uart5!"); };
template<> struct SignalConnection<DataB6::BitBang, Peripheral::BitBang> { static constexpr int8_t af = -1; };
template<> struct SignalConnection<DataB6::Cec, Peripheral::Hdmicec> { static constexpr int8_t af = 5; };
template<> struct SignalConnection<DataB6::Ch1, Peripheral::Tim4> { static constexpr int8_t af = 2; };
template<> struct SignalConnection<DataB6::Ch1n, Peripheral::Tim16> { static constexpr int8_t af = 1; };
template<> struct SignalConnection<DataB6::D5, Peripheral::Dcmi> { static constexpr int8_t af = 13; };
template<> struct SignalConnection<DataB6::D5, Peripheral::Pssi> { static constexpr int8_t af = 13; };
template<> struct SignalConnection<DataB6::Datin5, Peripheral::Dfsdm1> { static constexpr int8_t af = 11; };
template<> struct SignalConnection<DataB6::P1ncs, Peripheral::Octospim> { static constexpr int8_t af = 10; };
template<> struct SignalConnection<DataB6::Scl, Peripheral::I2c1> { static constexpr int8_t af = 4; };
template<> struct SignalConnection<DataB6::Scl, Peripheral::I2c4> { static constexpr int8_t af = 6; };
template<> struct SignalConnection<DataB6::Sdne1, Peripheral::Fmc> { static constexpr int8_t af = 12; };
template<> struct SignalConnection<DataB6::Tx, Peripheral::Usart1> { static constexpr int8_t af = 7; };
template<> struct SignalConnection<DataB6::Tx, Peripheral::Lpuart1> { static constexpr int8_t af = 8; };
template<> struct SignalConnection<DataB6::Tx, Peripheral::Fdcan2> { static constexpr int8_t af = 9; };
template<> struct SignalConnection<DataB6::Tx, Peripheral::Uart5> { static constexpr int8_t af = 14; };

struct DataB7 {
	static constexpr Gpio::Port port = Gpio::Port::B;
	static constexpr uint8_t pin = 7;
	struct BitBang { using Data = DataB7; static constexpr Gpio::Signal Signal = Gpio::Signal::BitBang; };
	struct Ch1n { using Data = DataB7; static constexpr Gpio::Signal Signal = Gpio::Signal::Ch1n; };
	struct Ch2 { using Data = DataB7; static constexpr Gpio::Signal Signal = Gpio::Signal::Ch2; };
	struct Ckin5 { using Data = DataB7; static constexpr Gpio::Signal Signal = Gpio::Signal::Ckin5; };
	struct Nl { using Data = DataB7; static constexpr Gpio::Signal Signal = Gpio::Signal::Nl; };
	struct Pvdin { using Data = DataB7; static constexpr Gpio::Signal Signal = Gpio::Signal::Pvdin; };
	struct Rdy { using Data = DataB7; static constexpr Gpio::Signal Signal = Gpio::Signal::Rdy; };
	struct Rx { using Data = DataB7; static constexpr Gpio::Signal Signal = Gpio::Signal::Rx; };
	struct Sda { using Data = DataB7; static constexpr Gpio::Signal Signal = Gpio::Signal::Sda; };
	struct Vsync { using Data = DataB7; static constexpr Gpio::Signal Signal = Gpio::Signal::Vsync; };
};
template<Peripheral p> struct SignalConnection<DataB7::BitBang, p> {
	static_assert(p == Peripheral::BitBang, "GpioB7::BitBang only connects to software drivers!"); };
template<Peripheral p> struct SignalConnection<DataB7::Ch1n, p> {
	static_assert((p == Peripheral::Tim17),"GpioB7::Ch1n only connects to Tim17!"); };
template<Peripheral p> struct SignalConnection<DataB7::Ch2, p> {
	static_assert((p == Peripheral::Tim4),"GpioB7::Ch2 only connects to Tim4!"); };
template<Peripheral p> struct SignalConnection<DataB7::Ckin5, p> {
	static_assert((p == Peripheral::Dfsdm1),"GpioB7::Ckin5 only connects to Dfsdm1!"); };
template<Peripheral p> struct SignalConnection<DataB7::Nl, p> {
	static_assert((p == Peripheral::Fmc),"GpioB7::Nl only connects to Fmc!"); };
template<Peripheral p> struct SignalConnection<DataB7::Pvdin, p> {
	static_assert((p == Peripheral::Pwr),"GpioB7::Pvdin only connects to Pwr!"); };
template<Peripheral p> struct SignalConnection<DataB7::Rdy, p> {
	static_assert((p == Peripheral::Pssi),"GpioB7::Rdy only connects to Pssi!"); };
template<Peripheral p> struct SignalConnection<DataB7::Rx, p> {
	static_assert((p == Peripheral::Usart1) or (p == Peripheral::Lpuart1),
		"GpioB7::Rx only connects to Usart1 or Lpuart1!"); };
template<Peripheral p> struct SignalConnection<DataB7::Sda, p> {
	static_assert((p == Peripheral::I2c1) or (p == Peripheral::I2c4),
		"GpioB7::Sda only connects to I2c1 or I2c4!"); };
template<Peripheral p> struct SignalConnection<DataB7::Vsync, p> {
	static_assert((p == Peripheral::Dcmi),"GpioB7::Vsync only connects to Dcmi!"); };
template<> struct SignalConnection<DataB7::BitBang, Peripheral::BitBang> { static constexpr int8_t af = -1; };
template<> struct SignalConnection<DataB7::Ch1n, Peripheral::Tim17> { static constexpr int8_t af = 1; };
template<> struct SignalConnection<DataB7::Ch2, Peripheral::Tim4> { static constexpr int8_t af = 2; };
template<> struct SignalConnection<DataB7::Ckin5, Peripheral::Dfsdm1> { static constexpr int8_t af = 11; };
template<> struct SignalConnection<DataB7::Nl, Peripheral::Fmc> { static constexpr int8_t af = 12; };
template<> struct SignalConnection<DataB7::Pvdin, Peripheral::Pwr> { static constexpr int8_t af = -1; };
template<> struct SignalConnection<DataB7::Rdy, Peripheral::Pssi> { static constexpr int8_t af = 13; };
template<> struct SignalConnection<DataB7::Rx, Peripheral::Usart1> { static constexpr int8_t af = 7; };
template<> struct SignalConnection<DataB7::Rx, Peripheral::Lpuart1> { static constexpr int8_t af = 8; };
template<> struct SignalConnection<DataB7::Sda, Peripheral::I2c1> { static constexpr int8_t af = 4; };
template<> struct SignalConnection<DataB7::Sda, Peripheral::I2c4> { static constexpr int8_t af = 6; };
template<> struct SignalConnection<DataB7::Vsync, Peripheral::Dcmi> { static constexpr int8_t af = 13; };

struct DataB8 {
	static constexpr Gpio::Port port = Gpio::Port::B;
	static constexpr uint8_t pin = 8;
	struct BitBang { using Data = DataB8; static constexpr Gpio::Signal Signal = Gpio::Signal::BitBang; };
	struct B6 { using Data = DataB8; static constexpr Gpio::Signal Signal = Gpio::Signal::B6; };
	struct Ch1 { using Data = DataB8; static constexpr Gpio::Signal Signal = Gpio::Signal::Ch1; };
	struct Ch3 { using Data = DataB8; static constexpr Gpio::Signal Signal = Gpio::Signal::Ch3; };
	struct Ckin { using Data = DataB8; static constexpr Gpio::Signal Signal = Gpio::Signal::Ckin; };
	struct Ckin7 { using Data = DataB8; static constexpr Gpio::Signal Signal = Gpio::Signal::Ckin7; };
	struct D4 { using Data = DataB8; static constexpr Gpio::Signal Signal = Gpio::Signal::D4; };
	struct D6 { using Data = DataB8; static constexpr Gpio::Signal Signal = Gpio::Signal::D6; };
	struct Rx { using Data = DataB8; static constexpr Gpio::Signal Signal = Gpio::Signal::Rx; };
	struct Scl { using Data = DataB8; static constexpr Gpio::Signal Signal = Gpio::Signal::Scl; };
	struct Txd3 { using Data = DataB8; static constexpr Gpio::Signal Signal = Gpio::Signal::Txd3; };
};
template<Peripheral p> struct SignalConnection<DataB8::BitBang, p> {
	static_assert(p == Peripheral::BitBang, "GpioB8::BitBang only connects to software drivers!"); };
template<Peripheral p> struct SignalConnection<DataB8::B6, p> {
	static_assert((p == Peripheral::Ltdc),"GpioB8::B6 only connects to Ltdc!"); };
template<Peripheral p> struct SignalConnection<DataB8::Ch1, p> {
	static_assert((p == Peripheral::Tim16),"GpioB8::Ch1 only connects to Tim16!"); };
template<Peripheral p> struct SignalConnection<DataB8::Ch3, p> {
	static_assert((p == Peripheral::Tim4),"GpioB8::Ch3 only connects to Tim4!"); };
template<Peripheral p> struct SignalConnection<DataB8::Ckin, p> {
	static_assert((p == Peripheral::Sdmmc1),"GpioB8::Ckin only connects to Sdmmc1!"); };
template<Peripheral p> struct SignalConnection<DataB8::Ckin7, p> {
	static_assert((p == Peripheral::Dfsdm1),"GpioB8::Ckin7 only connects to Dfsdm1!"); };
template<Peripheral p> struct SignalConnection<DataB8::D4, p> {
	static_assert((p == Peripheral::Sdmmc2) or (p == Peripheral::Sdmmc1),
		"GpioB8::D4 only connects to Sdmmc2 or Sdmmc1!"); };
template<Peripheral p> struct SignalConnection<DataB8::D6, p> {
	static_assert((p == Peripheral::Dcmi) or (p == Peripheral::Pssi),
		"GpioB8::D6 only connects to Dcmi or Pssi!"); };
template<Peripheral p> struct SignalConnection<DataB8::Rx, p> {
	static_assert((p == Peripheral::Uart4) or (p == Peripheral::Fdcan1),
		"GpioB8::Rx only connects to Uart4 or Fdcan1!"); };
template<Peripheral p> struct SignalConnection<DataB8::Scl, p> {
	static_assert((p == Peripheral::I2c1) or (p == Peripheral::I2c4),
		"GpioB8::Scl only connects to I2c1 or I2c4!"); };
template<Peripheral p> struct SignalConnection<DataB8::Txd3, p> {
	static_assert((p == Peripheral::Eth),"GpioB8::Txd3 only connects to Eth!"); };
template<> struct SignalConnection<DataB8::BitBang, Peripheral::BitBang> { static constexpr int8_t af = -1; };
template<> struct SignalConnection<DataB8::B6, Peripheral::Ltdc> { static constexpr int8_t af = 14; };
template<> struct SignalConnection<DataB8::Ch1, Peripheral::Tim16> { static constexpr int8_t af = 1; };
template<> struct SignalConnection<DataB8::Ch3, Peripheral::Tim4> { static constexpr int8_t af = 2; };
template<> struct SignalConnection<DataB8::Ckin, Peripheral::Sdmmc1> { static constexpr int8_t af = 7; };
template<> struct SignalConnection<DataB8::Ckin7, Peripheral::Dfsdm1> { static constexpr int8_t af = 3; };
template<> struct SignalConnection<DataB8::D4, Peripheral::Sdmmc2> { static constexpr int8_t af = 10; };
template<> struct SignalConnection<DataB8::D4, Peripheral::Sdmmc1> { static constexpr int8_t af = 12; };
template<> struct SignalConnection<DataB8::D6, Peripheral::Dcmi> { static constexpr int8_t af = 13; };
template<> struct SignalConnection<DataB8::D6, Peripheral::Pssi> { static constexpr int8_t af = 13; };
template<> struct SignalConnection<DataB8::Rx, Peripheral::Uart4> { static constexpr int8_t af = 8; };
template<> struct SignalConnection<DataB8::Rx, Peripheral::Fdcan1> { static constexpr int8_t af = 9; };
template<> struct SignalConnection<DataB8::Scl, Peripheral::I2c1> { static constexpr int8_t af = 4; };
template<> struct SignalConnection<DataB8::Scl, Peripheral::I2c4> { static constexpr int8_t af = 6; };
template<> struct SignalConnection<DataB8::Txd3, Peripheral::Eth> { static constexpr int8_t af = 11; };

struct DataB9 {
	static constexpr Gpio::Port port = Gpio::Port::B;
	static constexpr uint8_t pin = 9;
	struct BitBang { using Data = DataB9; static constexpr Gpio::Signal Signal = Gpio::Signal::BitBang; };
	struct B7 { using Data = DataB9; static constexpr Gpio::Signal Signal = Gpio::Signal::B7; };
	struct Cdir { using Data = DataB9; static constexpr Gpio::Signal Signal = Gpio::Signal::Cdir; };
	struct Ch1 { using Data = DataB9; static constexpr Gpio::Signal Signal = Gpio::Signal::Ch1; };
	struct Ch4 { using Data = DataB9; static constexpr Gpio::Signal Signal = Gpio::Signal::Ch4; };
	struct D5 { using Data = DataB9; static constexpr Gpio::Signal Signal = Gpio::Signal::D5; };
	struct D7 { using Data = DataB9; static constexpr Gpio::Signal Signal = Gpio::Signal::D7; };
	struct Datin7 { using Data = DataB9; static constexpr Gpio::Signal Signal = Gpio::Signal::Datin7; };
	struct Nss { using Data = DataB9; static constexpr Gpio::Signal Signal = Gpio::Signal::Nss; };
	struct Sda { using Data = DataB9; static constexpr Gpio::Signal Signal = Gpio::Signal::Sda; };
	struct Smba { using Data = DataB9; static constexpr Gpio::Signal Signal = Gpio::Signal::Smba; };
	struct Tx { using Data = DataB9; static constexpr Gpio::Signal Signal = Gpio::Signal::Tx; };
	struct Ws { using Data = DataB9; static constexpr Gpio::Signal Signal = Gpio::Signal::Ws; };
};
template<Peripheral p> struct SignalConnection<DataB9::BitBang, p> {
	static_assert(p == Peripheral::BitBang, "GpioB9::BitBang only connects to software drivers!"); };
template<Peripheral p> struct SignalConnection<DataB9::B7, p> {
	static_assert((p == Peripheral::Ltdc),"GpioB9::B7 only connects to Ltdc!"); };
template<Peripheral p> struct SignalConnection<DataB9::Cdir, p> {
	static_assert((p == Peripheral::Sdmmc1),"GpioB9::Cdir only connects to Sdmmc1!"); };
template<Peripheral p> struct SignalConnection<DataB9::Ch1, p> {
	static_assert((p == Peripheral::Tim17),"GpioB9::Ch1 only connects to Tim17!"); };
template<Peripheral p> struct SignalConnection<DataB9::Ch4, p> {
	static_assert((p == Peripheral::Tim4),"GpioB9::Ch4 only connects to Tim4!"); };
template<Peripheral p> struct SignalConnection<DataB9::D5, p> {
	static_assert((p == Peripheral::Sdmmc2) or (p == Peripheral::Sdmmc1),
		"GpioB9::D5 only connects to Sdmmc2 or Sdmmc1!"); };
template<Peripheral p> struct SignalConnection<DataB9::D7, p> {
	static_assert((p == Peripheral::Dcmi) or (p == Peripheral::Pssi),
		"GpioB9::D7 only connects to Dcmi or Pssi!"); };
template<Peripheral p> struct SignalConnection<DataB9::Datin7, p> {
	static_assert((p == Peripheral::Dfsdm1),"GpioB9::Datin7 only connects to Dfsdm1!"); };
template<Peripheral p> struct SignalConnection<DataB9::Nss, p> {
	static_assert((p == Peripheral::Spi2),"GpioB9::Nss only connects to Spi2!"); };
template<Peripheral p> struct SignalConnection<DataB9::Sda, p> {
	static_assert((p == Peripheral::I2c1) or (p == Peripheral::I2c4),
		"GpioB9::Sda only connects to I2c1 or I2c4!"); };
template<Peripheral p> struct SignalConnection<DataB9::Smba, p> {
	static_assert((p == Peripheral::I2c4),"GpioB9::Smba only connects to I2c4!"); };
template<Peripheral p> struct SignalConnection<DataB9::Tx, p> {
	static_assert((p == Peripheral::Uart4) or (p == Peripheral::Fdcan1),
		"GpioB9::Tx only connects to Uart4 or Fdcan1!"); };
template<Peripheral p> struct SignalConnection<DataB9::Ws, p> {
	static_assert((p == Peripheral::I2s2),"GpioB9::Ws only connects to I2s2!"); };
template<> struct SignalConnection<DataB9::BitBang, Peripheral::BitBang> { static constexpr int8_t af = -1; };
template<> struct SignalConnection<DataB9::B7, Peripheral::Ltdc> { static constexpr int8_t af = 14; };
template<> struct SignalConnection<DataB9::Cdir, Peripheral::Sdmmc1> { static constexpr int8_t af = 7; };
template<> struct SignalConnection<DataB9::Ch1, Peripheral::Tim17> { static constexpr int8_t af = 1; };
template<> struct SignalConnection<DataB9::Ch4, Peripheral::Tim4> { static constexpr int8_t af = 2; };
template<> struct SignalConnection<DataB9::D5, Peripheral::Sdmmc2> { static constexpr int8_t af = 10; };
template<> struct SignalConnection<DataB9::D5, Peripheral::Sdmmc1> { static constexpr int8_t af = 12; };
template<> struct SignalConnection<DataB9::D7, Peripheral::Dcmi> { static constexpr int8_t af = 13; };
template<> struct SignalConnection<DataB9::D7, Peripheral::Pssi> { static constexpr int8_t af = 13; };
template<> struct SignalConnection<DataB9::Datin7, Peripheral::Dfsdm1> { static constexpr int8_t af = 3; };
template<> struct SignalConnection<DataB9::Nss, Peripheral::Spi2> { static constexpr int8_t af = 5; };
template<> struct SignalConnection<DataB9::Sda, Peripheral::I2c1> { static constexpr int8_t af = 4; };
template<> struct SignalConnection<DataB9::Sda, Peripheral::I2c4> { static constexpr int8_t af = 6; };
template<> struct SignalConnection<DataB9::Smba, Peripheral::I2c4> { static constexpr int8_t af = 11; };
template<> struct SignalConnection<DataB9::Tx, Peripheral::Uart4> { static constexpr int8_t af = 8; };
template<> struct SignalConnection<DataB9::Tx, Peripheral::Fdcan1> { static constexpr int8_t af = 9; };
template<> struct SignalConnection<DataB9::Ws, Peripheral::I2s2> { static constexpr int8_t af = 5; };

struct DataB10 {
	static constexpr Gpio::Port port = Gpio::Port::B;
	static constexpr uint8_t pin = 10;
	struct BitBang { using Data = DataB10; static constexpr Gpio::Signal Signal = Gpio::Signal::BitBang; };
	struct Ch3 { using Data = DataB10; static constexpr Gpio::Signal Signal = Gpio::Signal::Ch3; };
	struct Ck { using Data = DataB10; static constexpr Gpio::Signal Signal = Gpio::Signal::Ck; };
	struct Datin7 { using Data = DataB10; static constexpr Gpio::Signal Signal = Gpio::Signal::Datin7; };
	struct G4 { using Data = DataB10; static constexpr Gpio::Signal Signal = Gpio::Signal::G4; };
	struct In1 { using Data = DataB10; static constexpr Gpio::Signal Signal = Gpio::Signal::In1; };
	struct P1ncs { using Data = DataB10; static constexpr Gpio::Signal Signal = Gpio::Signal::P1ncs; };
	struct Rxer { using Data = DataB10; static constexpr Gpio::Signal Signal = Gpio::Signal::Rxer; };
	struct Sck { using Data = DataB10; static constexpr Gpio::Signal Signal = Gpio::Signal::Sck; };
	struct Scl { using Data = DataB10; static constexpr Gpio::Signal Signal = Gpio::Signal::Scl; };
	struct Tx { using Data = DataB10; static constexpr Gpio::Signal Signal = Gpio::Signal::Tx; };
	struct Ulpid3 { using Data = DataB10; static constexpr Gpio::Signal Signal = Gpio::Signal::Ulpid3; };
};
template<Peripheral p> struct SignalConnection<DataB10::BitBang, p> {
	static_assert(p == Peripheral::BitBang, "GpioB10::BitBang only connects to software drivers!"); };
template<Peripheral p> struct SignalConnection<DataB10::Ch3, p> {
	static_assert((p == Peripheral::Tim2),"GpioB10::Ch3 only connects to Tim2!"); };
template<Peripheral p> struct SignalConnection<DataB10::Ck, p> {
	static_assert((p == Peripheral::I2s2),"GpioB10::Ck only connects to I2s2!"); };
template<Peripheral p> struct SignalConnection<DataB10::Datin7, p> {
	static_assert((p == Peripheral::Dfsdm1),"GpioB10::Datin7 only connects to Dfsdm1!"); };
template<Peripheral p> struct SignalConnection<DataB10::G4, p> {
	static_assert((p == Peripheral::Ltdc),"GpioB10::G4 only connects to Ltdc!"); };
template<Peripheral p> struct SignalConnection<DataB10::In1, p> {
	static_assert((p == Peripheral::Lptim2),"GpioB10::In1 only connects to Lptim2!"); };
template<Peripheral p> struct SignalConnection<DataB10::P1ncs, p> {
	static_assert((p == Peripheral::Octospim),"GpioB10::P1ncs only connects to Octospim!"); };
template<Peripheral p> struct SignalConnection<DataB10::Rxer, p> {
	static_assert((p == Peripheral::Eth),"GpioB10::Rxer only connects to Eth!"); };
template<Peripheral p> struct SignalConnection<DataB10::Sck, p> {
	static_assert((p == Peripheral::Spi2),"GpioB10::Sck only connects to Spi2!"); };
template<Peripheral p> struct SignalConnection<DataB10::Scl, p> {
	static_assert((p == Peripheral::I2c2),"GpioB10::Scl only connects to I2c2!"); };
template<Peripheral p> struct SignalConnection<DataB10::Tx, p> {
	static_assert((p == Peripheral::Usart3),"GpioB10::Tx only connects to Usart3!"); };
template<Peripheral p> struct SignalConnection<DataB10::Ulpid3, p> {
	static_assert((p == Peripheral::Usbotghs),"GpioB10::Ulpid3 only connects to Usbotghs!"); };
template<> struct SignalConnection<DataB10::BitBang, Peripheral::BitBang> { static constexpr int8_t af = -1; };
template<> struct SignalConnection<DataB10::Ch3, Peripheral::Tim2> { static constexpr int8_t af = 1; };
template<> struct SignalConnection<DataB10::Ck, Peripheral::I2s2> { static constexpr int8_t af = 5; };
template<> struct SignalConnection<DataB10::Datin7, Peripheral::Dfsdm1> { static constexpr int8_t af = 6; };
template<> struct SignalConnection<DataB10::G4, Peripheral::Ltdc> { static constexpr int8_t af = 14; };
template<> struct SignalConnection<DataB10::In1, Peripheral::Lptim2> { static constexpr int8_t af = 3; };
template<> struct SignalConnection<DataB10::P1ncs, Peripheral::Octospim> { static constexpr int8_t af = 9; };
template<> struct SignalConnection<DataB10::Rxer, Peripheral::Eth> { static constexpr int8_t af = 11; };
template<> struct SignalConnection<DataB10::Sck, Peripheral::Spi2> { static constexpr int8_t af = 5; };
template<> struct SignalConnection<DataB10::Scl, Peripheral::I2c2> { static constexpr int8_t af = 4; };
template<> struct SignalConnection<DataB10::Tx, Peripheral::Usart3> { static constexpr int8_t af = 7; };
template<> struct SignalConnection<DataB10::Ulpid3, Peripheral::Usbotghs> { static constexpr int8_t af = 10; };

struct DataB11 {
	static constexpr Gpio::Port port = Gpio::Port::B;
	static constexpr uint8_t pin = 11;
	struct BitBang { using Data = DataB11; static constexpr Gpio::Signal Signal = Gpio::Signal::BitBang; };
	struct Ch4 { using Data = DataB11; static constexpr Gpio::Signal Signal = Gpio::Signal::Ch4; };
	struct Ckin7 { using Data = DataB11; static constexpr Gpio::Signal Signal = Gpio::Signal::Ckin7; };
	struct Etr { using Data = DataB11; static constexpr Gpio::Signal Signal = Gpio::Signal::Etr; };
	struct G5 { using Data = DataB11; static constexpr Gpio::Signal Signal = Gpio::Signal::G5; };
	struct Rx { using Data = DataB11; static constexpr Gpio::Signal Signal = Gpio::Signal::Rx; };
	struct Sda { using Data = DataB11; static constexpr Gpio::Signal Signal = Gpio::Signal::Sda; };
	struct Txen { using Data = DataB11; static constexpr Gpio::Signal Signal = Gpio::Signal::Txen; };
	struct Ulpid4 { using Data = DataB11; static constexpr Gpio::Signal Signal = Gpio::Signal::Ulpid4; };
};
template<Peripheral p> struct SignalConnection<DataB11::BitBang, p> {
	static_assert(p == Peripheral::BitBang, "GpioB11::BitBang only connects to software drivers!"); };
template<Peripheral p> struct SignalConnection<DataB11::Ch4, p> {
	static_assert((p == Peripheral::Tim2),"GpioB11::Ch4 only connects to Tim2!"); };
template<Peripheral p> struct SignalConnection<DataB11::Ckin7, p> {
	static_assert((p == Peripheral::Dfsdm1),"GpioB11::Ckin7 only connects to Dfsdm1!"); };
template<Peripheral p> struct SignalConnection<DataB11::Etr, p> {
	static_assert((p == Peripheral::Lptim2),"GpioB11::Etr only connects to Lptim2!"); };
template<Peripheral p> struct SignalConnection<DataB11::G5, p> {
	static_assert((p == Peripheral::Ltdc),"GpioB11::G5 only connects to Ltdc!"); };
template<Peripheral p> struct SignalConnection<DataB11::Rx, p> {
	static_assert((p == Peripheral::Usart3),"GpioB11::Rx only connects to Usart3!"); };
template<Peripheral p> struct SignalConnection<DataB11::Sda, p> {
	static_assert((p == Peripheral::I2c2),"GpioB11::Sda only connects to I2c2!"); };
template<Peripheral p> struct SignalConnection<DataB11::Txen, p> {
	static_assert((p == Peripheral::Eth),"GpioB11::Txen only connects to Eth!"); };
template<Peripheral p> struct SignalConnection<DataB11::Ulpid4, p> {
	static_assert((p == Peripheral::Usbotghs),"GpioB11::Ulpid4 only connects to Usbotghs!"); };
template<> struct SignalConnection<DataB11::BitBang, Peripheral::BitBang> { static constexpr int8_t af = -1; };
template<> struct SignalConnection<DataB11::Ch4, Peripheral::Tim2> { static constexpr int8_t af = 1; };
template<> struct SignalConnection<DataB11::Ckin7, Peripheral::Dfsdm1> { static constexpr int8_t af = 6; };
template<> struct SignalConnection<DataB11::Etr, Peripheral::Lptim2> { static constexpr int8_t af = 3; };
template<> struct SignalConnection<DataB11::G5, Peripheral::Ltdc> { static constexpr int8_t af = 14; };
template<> struct SignalConnection<DataB11::Rx, Peripheral::Usart3> { static constexpr int8_t af = 7; };
template<> struct SignalConnection<DataB11::Sda, Peripheral::I2c2> { static constexpr int8_t af = 4; };
template<> struct SignalConnection<DataB11::Txen, Peripheral::Eth> { static constexpr int8_t af = 11; };
template<> struct SignalConnection<DataB11::Ulpid4, Peripheral::Usbotghs> { static constexpr int8_t af = 10; };

struct DataB12 {
	static constexpr Gpio::Port port = Gpio::Port::B;
	static constexpr uint8_t pin = 12;
	struct BitBang { using Data = DataB12; static constexpr Gpio::Signal Signal = Gpio::Signal::BitBang; };
	struct Bkin { using Data = DataB12; static constexpr Gpio::Signal Signal = Gpio::Signal::Bkin; };
	struct Bkincomp1 { using Data = DataB12; static constexpr Gpio::Signal Signal = Gpio::Signal::Bkincomp1; };
	struct Bkincomp2 { using Data = DataB12; static constexpr Gpio::Signal Signal = Gpio::Signal::Bkincomp2; };
	struct Ck { using Data = DataB12; static constexpr Gpio::Signal Signal = Gpio::Signal::Ck; };
	struct Datin1 { using Data = DataB12; static constexpr Gpio::Signal Signal = Gpio::Signal::Datin1; };
	struct Nss { using Data = DataB12; static constexpr Gpio::Signal Signal = Gpio::Signal::Nss; };
	struct P1io0 { using Data = DataB12; static constexpr Gpio::Signal Signal = Gpio::Signal::P1io0; };
	struct P1nclk { using Data = DataB12; static constexpr Gpio::Signal Signal = Gpio::Signal::P1nclk; };
	struct Rx { using Data = DataB12; static constexpr Gpio::Signal Signal = Gpio::Signal::Rx; };
	struct Smba { using Data = DataB12; static constexpr Gpio::Signal Signal = Gpio::Signal::Smba; };
	struct Txd0 { using Data = DataB12; static constexpr Gpio::Signal Signal = Gpio::Signal::Txd0; };
	struct Ulpid5 { using Data = DataB12; static constexpr Gpio::Signal Signal = Gpio::Signal::Ulpid5; };
	struct Ws { using Data = DataB12; static constexpr Gpio::Signal Signal = Gpio::Signal::Ws; };
};
template<Peripheral p> struct SignalConnection<DataB12::BitBang, p> {
	static_assert(p == Peripheral::BitBang, "GpioB12::BitBang only connects to software drivers!"); };
template<Peripheral p> struct SignalConnection<DataB12::Bkin, p> {
	static_assert((p == Peripheral::Tim1),"GpioB12::Bkin only connects to Tim1!"); };
template<Peripheral p> struct SignalConnection<DataB12::Bkincomp1, p> {
	static_assert((p == Peripheral::Tim1),"GpioB12::Bkincomp1 only connects to Tim1!"); };
template<Peripheral p> struct SignalConnection<DataB12::Bkincomp2, p> {
	static_assert((p == Peripheral::Tim1),"GpioB12::Bkincomp2 only connects to Tim1!"); };
template<Peripheral p> struct SignalConnection<DataB12::Ck, p> {
	static_assert((p == Peripheral::Usart3),"GpioB12::Ck only connects to Usart3!"); };
template<Peripheral p> struct SignalConnection<DataB12::Datin1, p> {
	static_assert((p == Peripheral::Dfsdm1),"GpioB12::Datin1 only connects to Dfsdm1!"); };
template<Peripheral p> struct SignalConnection<DataB12::Nss, p> {
	static_assert((p == Peripheral::Spi2),"GpioB12::Nss only connects to Spi2!"); };
template<Peripheral p> struct SignalConnection<DataB12::P1io0, p> {
	static_assert((p == Peripheral::Octospim),"GpioB12::P1io0 only connects to Octospim!"); };
template<Peripheral p> struct SignalConnection<DataB12::P1nclk, p> {
	static_assert((p == Peripheral::Octospim),"GpioB12::P1nclk only connects to Octospim!"); };
template<Peripheral p> struct SignalConnection<DataB12::Rx, p> {
	static_assert((p == Peripheral::Fdcan2) or (p == Peripheral::Uart5),
		"GpioB12::Rx only connects to Fdcan2 or Uart5!"); };
template<Peripheral p> struct SignalConnection<DataB12::Smba, p> {
	static_assert((p == Peripheral::I2c2),"GpioB12::Smba only connects to I2c2!"); };
template<Peripheral p> struct SignalConnection<DataB12::Txd0, p> {
	static_assert((p == Peripheral::Eth),"GpioB12::Txd0 only connects to Eth!"); };
template<Peripheral p> struct SignalConnection<DataB12::Ulpid5, p> {
	static_assert((p == Peripheral::Usbotghs),"GpioB12::Ulpid5 only connects to Usbotghs!"); };
template<Peripheral p> struct SignalConnection<DataB12::Ws, p> {
	static_assert((p == Peripheral::I2s2),"GpioB12::Ws only connects to I2s2!"); };
template<> struct SignalConnection<DataB12::BitBang, Peripheral::BitBang> { static constexpr int8_t af = -1; };
template<> struct SignalConnection<DataB12::Bkin, Peripheral::Tim1> { static constexpr int8_t af = 1; };
template<> struct SignalConnection<DataB12::Bkincomp1, Peripheral::Tim1> { static constexpr int8_t af = 13; };
template<> struct SignalConnection<DataB12::Bkincomp2, Peripheral::Tim1> { static constexpr int8_t af = 13; };
template<> struct SignalConnection<DataB12::Ck, Peripheral::Usart3> { static constexpr int8_t af = 7; };
template<> struct SignalConnection<DataB12::Datin1, Peripheral::Dfsdm1> { static constexpr int8_t af = 6; };
template<> struct SignalConnection<DataB12::Nss, Peripheral::Spi2> { static constexpr int8_t af = 5; };
template<> struct SignalConnection<DataB12::P1io0, Peripheral::Octospim> { static constexpr int8_t af = 12; };
template<> struct SignalConnection<DataB12::P1nclk, Peripheral::Octospim> { static constexpr int8_t af = 3; };
template<> struct SignalConnection<DataB12::Rx, Peripheral::Fdcan2> { static constexpr int8_t af = 9; };
template<> struct SignalConnection<DataB12::Rx, Peripheral::Uart5> { static constexpr int8_t af = 14; };
template<> struct SignalConnection<DataB12::Smba, Peripheral::I2c2> { static constexpr int8_t af = 4; };
template<> struct SignalConnection<DataB12::Txd0, Peripheral::Eth> { static constexpr int8_t af = 11; };
template<> struct SignalConnection<DataB12::Ulpid5, Peripheral::Usbotghs> { static constexpr int8_t af = 10; };
template<> struct SignalConnection<DataB12::Ws, Peripheral::I2s2> { static constexpr int8_t af = 5; };

struct DataB13 {
	static constexpr Gpio::Port port = Gpio::Port::B;
	static constexpr uint8_t pin = 13;
	struct BitBang { using Data = DataB13; static constexpr Gpio::Signal Signal = Gpio::Signal::BitBang; };
	struct Ch1n { using Data = DataB13; static constexpr Gpio::Signal Signal = Gpio::Signal::Ch1n; };
	struct Ck { using Data = DataB13; static constexpr Gpio::Signal Signal = Gpio::Signal::Ck; };
	struct Ckin1 { using Data = DataB13; static constexpr Gpio::Signal Signal = Gpio::Signal::Ckin1; };
	struct Cts { using Data = DataB13; static constexpr Gpio::Signal Signal = Gpio::Signal::Cts; };
	struct D0 { using Data = DataB13; static constexpr Gpio::Signal Signal = Gpio::Signal::D0; };
	struct D2 { using Data = DataB13; static constexpr Gpio::Signal Signal = Gpio::Signal::D2; };
	struct Nss { using Data = DataB13; static constexpr Gpio::Signal Signal = Gpio::Signal::Nss; };
	struct Out { using Data = DataB13; static constexpr Gpio::Signal Signal = Gpio::Signal::Out; };
	struct P1io2 { using Data = DataB13; static constexpr Gpio::Signal Signal = Gpio::Signal::P1io2; };
	struct Sck { using Data = DataB13; static constexpr Gpio::Signal Signal = Gpio::Signal::Sck; };
	struct Tx { using Data = DataB13; static constexpr Gpio::Signal Signal = Gpio::Signal::Tx; };
	struct Txd1 { using Data = DataB13; static constexpr Gpio::Signal Signal = Gpio::Signal::Txd1; };
	struct Ulpid6 { using Data = DataB13; static constexpr Gpio::Signal Signal = Gpio::Signal::Ulpid6; };
};
template<Peripheral p> struct SignalConnection<DataB13::BitBang, p> {
	static_assert(p == Peripheral::BitBang, "GpioB13::BitBang only connects to software drivers!"); };
template<Peripheral p> struct SignalConnection<DataB13::Ch1n, p> {
	static_assert((p == Peripheral::Tim1),"GpioB13::Ch1n only connects to Tim1!"); };
template<Peripheral p> struct SignalConnection<DataB13::Ck, p> {
	static_assert((p == Peripheral::I2s2),"GpioB13::Ck only connects to I2s2!"); };
template<Peripheral p> struct SignalConnection<DataB13::Ckin1, p> {
	static_assert((p == Peripheral::Dfsdm1),"GpioB13::Ckin1 only connects to Dfsdm1!"); };
template<Peripheral p> struct SignalConnection<DataB13::Cts, p> {
	static_assert((p == Peripheral::Usart3),"GpioB13::Cts only connects to Usart3!"); };
template<Peripheral p> struct SignalConnection<DataB13::D0, p> {
	static_assert((p == Peripheral::Sdmmc1),"GpioB13::D0 only connects to Sdmmc1!"); };
template<Peripheral p> struct SignalConnection<DataB13::D2, p> {
	static_assert((p == Peripheral::Dcmi) or (p == Peripheral::Pssi),
		"GpioB13::D2 only connects to Dcmi or Pssi!"); };
template<Peripheral p> struct SignalConnection<DataB13::Nss, p> {
	static_assert((p == Peripheral::Usart3),"GpioB13::Nss only connects to Usart3!"); };
template<Peripheral p> struct SignalConnection<DataB13::Out, p> {
	static_assert((p == Peripheral::Lptim2),"GpioB13::Out only connects to Lptim2!"); };
template<Peripheral p> struct SignalConnection<DataB13::P1io2, p> {
	static_assert((p == Peripheral::Octospim),"GpioB13::P1io2 only connects to Octospim!"); };
template<Peripheral p> struct SignalConnection<DataB13::Sck, p> {
	static_assert((p == Peripheral::Spi2),"GpioB13::Sck only connects to Spi2!"); };
template<Peripheral p> struct SignalConnection<DataB13::Tx, p> {
	static_assert((p == Peripheral::Fdcan2) or (p == Peripheral::Uart5),
		"GpioB13::Tx only connects to Fdcan2 or Uart5!"); };
template<Peripheral p> struct SignalConnection<DataB13::Txd1, p> {
	static_assert((p == Peripheral::Eth),"GpioB13::Txd1 only connects to Eth!"); };
template<Peripheral p> struct SignalConnection<DataB13::Ulpid6, p> {
	static_assert((p == Peripheral::Usbotghs),"GpioB13::Ulpid6 only connects to Usbotghs!"); };
template<> struct SignalConnection<DataB13::BitBang, Peripheral::BitBang> { static constexpr int8_t af = -1; };
template<> struct SignalConnection<DataB13::Ch1n, Peripheral::Tim1> { static constexpr int8_t af = 1; };
template<> struct SignalConnection<DataB13::Ck, Peripheral::I2s2> { static constexpr int8_t af = 5; };
template<> struct SignalConnection<DataB13::Ckin1, Peripheral::Dfsdm1> { static constexpr int8_t af = 6; };
template<> struct SignalConnection<DataB13::Cts, Peripheral::Usart3> { static constexpr int8_t af = 7; };
template<> struct SignalConnection<DataB13::D0, Peripheral::Sdmmc1> { static constexpr int8_t af = 12; };
template<> struct SignalConnection<DataB13::D2, Peripheral::Dcmi> { static constexpr int8_t af = 13; };
template<> struct SignalConnection<DataB13::D2, Peripheral::Pssi> { static constexpr int8_t af = 13; };
template<> struct SignalConnection<DataB13::Nss, Peripheral::Usart3> { static constexpr int8_t af = 7; };
template<> struct SignalConnection<DataB13::Out, Peripheral::Lptim2> { static constexpr int8_t af = 3; };
template<> struct SignalConnection<DataB13::P1io2, Peripheral::Octospim> { static constexpr int8_t af = 4; };
template<> struct SignalConnection<DataB13::Sck, Peripheral::Spi2> { static constexpr int8_t af = 5; };
template<> struct SignalConnection<DataB13::Tx, Peripheral::Fdcan2> { static constexpr int8_t af = 9; };
template<> struct SignalConnection<DataB13::Tx, Peripheral::Uart5> { static constexpr int8_t af = 14; };
template<> struct SignalConnection<DataB13::Txd1, Peripheral::Eth> { static constexpr int8_t af = 11; };
template<> struct SignalConnection<DataB13::Ulpid6, Peripheral::Usbotghs> { static constexpr int8_t af = 10; };

struct DataB14 {
	static constexpr Gpio::Port port = Gpio::Port::B;
	static constexpr uint8_t pin = 14;
	struct BitBang { using Data = DataB14; static constexpr Gpio::Signal Signal = Gpio::Signal::BitBang; };
	struct Ch1 { using Data = DataB14; static constexpr Gpio::Signal Signal = Gpio::Signal::Ch1; };
	struct Ch2n { using Data = DataB14; static constexpr Gpio::Signal Signal = Gpio::Signal::Ch2n; };
	struct Clk { using Data = DataB14; static constexpr Gpio::Signal Signal = Gpio::Signal::Clk; };
	struct D0 { using Data = DataB14; static constexpr Gpio::Signal Signal = Gpio::Signal::D0; };
	struct D10 { using Data = DataB14; static constexpr Gpio::Signal Signal = Gpio::Signal::D10; };
	struct Da10 { using Data = DataB14; static constexpr Gpio::Signal Signal = Gpio::Signal::Da10; };
	struct Datin2 { using Data = DataB14; static constexpr Gpio::Signal Signal = Gpio::Signal::Datin2; };
	struct De { using Data = DataB14; static constexpr Gpio::Signal Signal = Gpio::Signal::De; };
	struct Miso { using Data = DataB14; static constexpr Gpio::Signal Signal = Gpio::Signal::Miso; };
	struct Rts { using Data = DataB14; static constexpr Gpio::Signal Signal = Gpio::Signal::Rts; };
	struct Sdi { using Data = DataB14; static constexpr Gpio::Signal Signal = Gpio::Signal::Sdi; };
	struct Tx { using Data = DataB14; static constexpr Gpio::Signal Signal = Gpio::Signal::Tx; };
};
template<Peripheral p> struct SignalConnection<DataB14::BitBang, p> {
	static_assert(p == Peripheral::BitBang, "GpioB14::BitBang only connects to software drivers!"); };
template<Peripheral p> struct SignalConnection<DataB14::Ch1, p> {
	static_assert((p == Peripheral::Tim12),"GpioB14::Ch1 only connects to Tim12!"); };
template<Peripheral p> struct SignalConnection<DataB14::Ch2n, p> {
	static_assert((p == Peripheral::Tim1) or (p == Peripheral::Tim8),
		"GpioB14::Ch2n only connects to Tim1 or Tim8!"); };
template<Peripheral p> struct SignalConnection<DataB14::Clk, p> {
	static_assert((p == Peripheral::Ltdc),"GpioB14::Clk only connects to Ltdc!"); };
template<Peripheral p> struct SignalConnection<DataB14::D0, p> {
	static_assert((p == Peripheral::Sdmmc2),"GpioB14::D0 only connects to Sdmmc2!"); };
template<Peripheral p> struct SignalConnection<DataB14::D10, p> {
	static_assert((p == Peripheral::Fmc),"GpioB14::D10 only connects to Fmc!"); };
template<Peripheral p> struct SignalConnection<DataB14::Da10, p> {
	static_assert((p == Peripheral::Fmc),"GpioB14::Da10 only connects to Fmc!"); };
template<Peripheral p> struct SignalConnection<DataB14::Datin2, p> {
	static_assert((p == Peripheral::Dfsdm1),"GpioB14::Datin2 only connects to Dfsdm1!"); };
template<Peripheral p> struct SignalConnection<DataB14::De, p> {
	static_assert((p == Peripheral::Usart3) or (p == Peripheral::Uart4),
		"GpioB14::De only connects to Usart3 or Uart4!"); };
template<Peripheral p> struct SignalConnection<DataB14::Miso, p> {
	static_assert((p == Peripheral::Spi2),"GpioB14::Miso only connects to Spi2!"); };
template<Peripheral p> struct SignalConnection<DataB14::Rts, p> {
	static_assert((p == Peripheral::Usart3) or (p == Peripheral::Uart4),
		"GpioB14::Rts only connects to Usart3 or Uart4!"); };
template<Peripheral p> struct SignalConnection<DataB14::Sdi, p> {
	static_assert((p == Peripheral::I2s2),"GpioB14::Sdi only connects to I2s2!"); };
template<Peripheral p> struct SignalConnection<DataB14::Tx, p> {
	static_assert((p == Peripheral::Usart1),"GpioB14::Tx only connects to Usart1!"); };
template<> struct SignalConnection<DataB14::BitBang, Peripheral::BitBang> { static constexpr int8_t af = -1; };
template<> struct SignalConnection<DataB14::Ch1, Peripheral::Tim12> { static constexpr int8_t af = 2; };
template<> struct SignalConnection<DataB14::Ch2n, Peripheral::Tim1> { static constexpr int8_t af = 1; };
template<> struct SignalConnection<DataB14::Ch2n, Peripheral::Tim8> { static constexpr int8_t af = 3; };
template<> struct SignalConnection<DataB14::Clk, Peripheral::Ltdc> { static constexpr int8_t af = 14; };
template<> struct SignalConnection<DataB14::D0, Peripheral::Sdmmc2> { static constexpr int8_t af = 9; };
template<> struct SignalConnection<DataB14::D10, Peripheral::Fmc> { static constexpr int8_t af = 12; };
template<> struct SignalConnection<DataB14::Da10, Peripheral::Fmc> { static constexpr int8_t af = 12; };
template<> struct SignalConnection<DataB14::Datin2, Peripheral::Dfsdm1> { static constexpr int8_t af = 6; };
template<> struct SignalConnection<DataB14::De, Peripheral::Usart3> { static constexpr int8_t af = 7; };
template<> struct SignalConnection<DataB14::De, Peripheral::Uart4> { static constexpr int8_t af = 8; };
template<> struct SignalConnection<DataB14::Miso, Peripheral::Spi2> { static constexpr int8_t af = 5; };
template<> struct SignalConnection<DataB14::Rts, Peripheral::Usart3> { static constexpr int8_t af = 7; };
template<> struct SignalConnection<DataB14::Rts, Peripheral::Uart4> { static constexpr int8_t af = 8; };
template<> struct SignalConnection<DataB14::Sdi, Peripheral::I2s2> { static constexpr int8_t af = 5; };
template<> struct SignalConnection<DataB14::Tx, Peripheral::Usart1> { static constexpr int8_t af = 4; };

struct DataB15 {
	static constexpr Gpio::Port port = Gpio::Port::B;
	static constexpr uint8_t pin = 15;
	struct BitBang { using Data = DataB15; static constexpr Gpio::Signal Signal = Gpio::Signal::BitBang; };
	struct Ch2 { using Data = DataB15; static constexpr Gpio::Signal Signal = Gpio::Signal::Ch2; };
	struct Ch3n { using Data = DataB15; static constexpr Gpio::Signal Signal = Gpio::Signal::Ch3n; };
	struct Ckin2 { using Data = DataB15; static constexpr Gpio::Signal Signal = Gpio::Signal::Ckin2; };
	struct Cts { using Data = DataB15; static constexpr Gpio::Signal Signal = Gpio::Signal::Cts; };
	struct D1 { using Data = DataB15; static constexpr Gpio::Signal Signal = Gpio::Signal::D1; };
	struct D11 { using Data = DataB15; static constexpr Gpio::Signal Signal = Gpio::Signal::D11; };
	struct Da11 { using Data = DataB15; static constexpr Gpio::Signal Signal = Gpio::Signal::Da11; };
	struct G7 { using Data = DataB15; static constexpr Gpio::Signal Signal = Gpio::Signal::G7; };
	struct Mosi { using Data = DataB15; static constexpr Gpio::Signal Signal = Gpio::Signal::Mosi; };
	struct Refin { using Data = DataB15; static constexpr Gpio::Signal Signal = Gpio::Signal::Refin; };
	struct Rx { using Data = DataB15; static constexpr Gpio::Signal Signal = Gpio::Signal::Rx; };
	struct Sdo { using Data = DataB15; static constexpr Gpio::Signal Signal = Gpio::Signal::Sdo; };
};
template<Peripheral p> struct SignalConnection<DataB15::BitBang, p> {
	static_assert(p == Peripheral::BitBang, "GpioB15::BitBang only connects to software drivers!"); };
template<Peripheral p> struct SignalConnection<DataB15::Ch2, p> {
	static_assert((p == Peripheral::Tim12),"GpioB15::Ch2 only connects to Tim12!"); };
template<Peripheral p> struct SignalConnection<DataB15::Ch3n, p> {
	static_assert((p == Peripheral::Tim1) or (p == Peripheral::Tim8),
		"GpioB15::Ch3n only connects to Tim1 or Tim8!"); };
template<Peripheral p> struct SignalConnection<DataB15::Ckin2, p> {
	static_assert((p == Peripheral::Dfsdm1),"GpioB15::Ckin2 only connects to Dfsdm1!"); };
template<Peripheral p> struct SignalConnection<DataB15::Cts, p> {
	static_assert((p == Peripheral::Uart4),"GpioB15::Cts only connects to Uart4!"); };
template<Peripheral p> struct SignalConnection<DataB15::D1, p> {
	static_assert((p == Peripheral::Sdmmc2),"GpioB15::D1 only connects to Sdmmc2!"); };
template<Peripheral p> struct SignalConnection<DataB15::D11, p> {
	static_assert((p == Peripheral::Fmc),"GpioB15::D11 only connects to Fmc!"); };
template<Peripheral p> struct SignalConnection<DataB15::Da11, p> {
	static_assert((p == Peripheral::Fmc),"GpioB15::Da11 only connects to Fmc!"); };
template<Peripheral p> struct SignalConnection<DataB15::G7, p> {
	static_assert((p == Peripheral::Ltdc),"GpioB15::G7 only connects to Ltdc!"); };
template<Peripheral p> struct SignalConnection<DataB15::Mosi, p> {
	static_assert((p == Peripheral::Spi2),"GpioB15::Mosi only connects to Spi2!"); };
template<Peripheral p> struct SignalConnection<DataB15::Refin, p> {
	static_assert((p == Peripheral::Rtc),"GpioB15::Refin only connects to Rtc!"); };
template<Peripheral p> struct SignalConnection<DataB15::Rx, p> {
	static_assert((p == Peripheral::Usart1),"GpioB15::Rx only connects to Usart1!"); };
template<Peripheral p> struct SignalConnection<DataB15::Sdo, p> {
	static_assert((p == Peripheral::I2s2),"GpioB15::Sdo only connects to I2s2!"); };
template<> struct SignalConnection<DataB15::BitBang, Peripheral::BitBang> { static constexpr int8_t af = -1; };
template<> struct SignalConnection<DataB15::Ch2, Peripheral::Tim12> { static constexpr int8_t af = 2; };
template<> struct SignalConnection<DataB15::Ch3n, Peripheral::Tim1> { static constexpr int8_t af = 1; };
template<> struct SignalConnection<DataB15::Ch3n, Peripheral::Tim8> { static constexpr int8_t af = 3; };
template<> struct SignalConnection<DataB15::Ckin2, Peripheral::Dfsdm1> { static constexpr int8_t af = 6; };
template<> struct SignalConnection<DataB15::Cts, Peripheral::Uart4> { static constexpr int8_t af = 8; };
template<> struct SignalConnection<DataB15::D1, Peripheral::Sdmmc2> { static constexpr int8_t af = 9; };
template<> struct SignalConnection<DataB15::D11, Peripheral::Fmc> { static constexpr int8_t af = 12; };
template<> struct SignalConnection<DataB15::Da11, Peripheral::Fmc> { static constexpr int8_t af = 12; };
template<> struct SignalConnection<DataB15::G7, Peripheral::Ltdc> { static constexpr int8_t af = 14; };
template<> struct SignalConnection<DataB15::Mosi, Peripheral::Spi2> { static constexpr int8_t af = 5; };
template<> struct SignalConnection<DataB15::Refin, Peripheral::Rtc> { static constexpr int8_t af = 0; };
template<> struct SignalConnection<DataB15::Rx, Peripheral::Usart1> { static constexpr int8_t af = 4; };
template<> struct SignalConnection<DataB15::Sdo, Peripheral::I2s2> { static constexpr int8_t af = 5; };

struct DataC0 {
	static constexpr Gpio::Port port = Gpio::Port::C;
	static constexpr uint8_t pin = 0;
	struct BitBang { using Data = DataC0; static constexpr Gpio::Signal Signal = Gpio::Signal::BitBang; };
	struct A25 { using Data = DataC0; static constexpr Gpio::Signal Signal = Gpio::Signal::A25; };
	struct Ckin0 { using Data = DataC0; static constexpr Gpio::Signal Signal = Gpio::Signal::Ckin0; };
	struct D12 { using Data = DataC0; static constexpr Gpio::Signal Signal = Gpio::Signal::D12; };
	struct Da12 { using Data = DataC0; static constexpr Gpio::Signal Signal = Gpio::Signal::Da12; };
	struct Datin4 { using Data = DataC0; static constexpr Gpio::Signal Signal = Gpio::Signal::Datin4; };
	struct Fsb { using Data = DataC0; static constexpr Gpio::Signal Signal = Gpio::Signal::Fsb; };
	struct G2 { using Data = DataC0; static constexpr Gpio::Signal Signal = Gpio::Signal::G2; };
	struct Inp10 { using Data = DataC0; static constexpr Gpio::Signal Signal = Gpio::Signal::Inp10; };
	struct R5 { using Data = DataC0; static constexpr Gpio::Signal Signal = Gpio::Signal::R5; };
	struct Sdnwe { using Data = DataC0; static constexpr Gpio::Signal Signal = Gpio::Signal::Sdnwe; };
	struct Ulpistp { using Data = DataC0; static constexpr Gpio::Signal Signal = Gpio::Signal::Ulpistp; };
};
template<Peripheral p> struct SignalConnection<DataC0::BitBang, p> {
	static_assert(p == Peripheral::BitBang, "GpioC0::BitBang only connects to software drivers!"); };
template<Peripheral p> struct SignalConnection<DataC0::A25, p> {
	static_assert((p == Peripheral::Fmc),"GpioC0::A25 only connects to Fmc!"); };
template<Peripheral p> struct SignalConnection<DataC0::Ckin0, p> {
	static_assert((p == Peripheral::Dfsdm1),"GpioC0::Ckin0 only connects to Dfsdm1!"); };
template<Peripheral p> struct SignalConnection<DataC0::D12, p> {
	static_assert((p == Peripheral::Fmc),"GpioC0::D12 only connects to Fmc!"); };
template<Peripheral p> struct SignalConnection<DataC0::Da12, p> {
	static_assert((p == Peripheral::Fmc),"GpioC0::Da12 only connects to Fmc!"); };
template<Peripheral p> struct SignalConnection<DataC0::Datin4, p> {
	static_assert((p == Peripheral::Dfsdm1),"GpioC0::Datin4 only connects to Dfsdm1!"); };
template<Peripheral p> struct SignalConnection<DataC0::Fsb, p> {
	static_assert((p == Peripheral::Sai4),"GpioC0::Fsb only connects to Sai4!"); };
template<Peripheral p> struct SignalConnection<DataC0::G2, p> {
	static_assert((p == Peripheral::Ltdc),"GpioC0::G2 only connects to Ltdc!"); };
template<Peripheral p> struct SignalConnection<DataC0::Inp10, p> {
	static_assert((p == Peripheral::Adc1) or (p == Peripheral::Adc2) or (p == Peripheral::Adc3),
		"GpioC0::Inp10 only connects to Adc1 or Adc2 or Adc3!"); };
template<Peripheral p> struct SignalConnection<DataC0::R5, p> {
	static_assert((p == Peripheral::Ltdc),"GpioC0::R5 only connects to Ltdc!"); };
template<Peripheral p> struct SignalConnection<DataC0::Sdnwe, p> {
	static_assert((p == Peripheral::Fmc),"GpioC0::Sdnwe only connects to Fmc!"); };
template<Peripheral p> struct SignalConnection<DataC0::Ulpistp, p> {
	static_assert((p == Peripheral::Usbotghs),"GpioC0::Ulpistp only connects to Usbotghs!"); };
template<> struct SignalConnection<DataC0::BitBang, Peripheral::BitBang> { static constexpr int8_t af = -1; };
template<> struct SignalConnection<DataC0::A25, Peripheral::Fmc> { static constexpr int8_t af = 9; };
template<> struct SignalConnection<DataC0::Ckin0, Peripheral::Dfsdm1> { static constexpr int8_t af = 3; };
template<> struct SignalConnection<DataC0::D12, Peripheral::Fmc> { static constexpr int8_t af = 1; };
template<> struct SignalConnection<DataC0::Da12, Peripheral::Fmc> { static constexpr int8_t af = 1; };
template<> struct SignalConnection<DataC0::Datin4, Peripheral::Dfsdm1> { static constexpr int8_t af = 6; };
template<> struct SignalConnection<DataC0::Fsb, Peripheral::Sai4> { static constexpr int8_t af = 8; };
template<> struct SignalConnection<DataC0::G2, Peripheral::Ltdc> { static constexpr int8_t af = 11; };
template<> struct SignalConnection<DataC0::Inp10, Peripheral::Adc1> { static constexpr int8_t af = -2; };
template<> struct SignalConnection<DataC0::Inp10, Peripheral::Adc2> { static constexpr int8_t af = -2; };
template<> struct SignalConnection<DataC0::Inp10, Peripheral::Adc3> { static constexpr int8_t af = -2; };
template<> struct SignalConnection<DataC0::R5, Peripheral::Ltdc> { static constexpr int8_t af = 14; };
template<> struct SignalConnection<DataC0::Sdnwe, Peripheral::Fmc> { static constexpr int8_t af = 12; };
template<> struct SignalConnection<DataC0::Ulpistp, Peripheral::Usbotghs> { static constexpr int8_t af = 10; };
template<> constexpr inline int8_t AdcChannel<DataC0, Peripheral::Adc1, AdcPolarity::Positive> = 10;
template<> constexpr inline int8_t AdcChannel<DataC0, Peripheral::Adc2, AdcPolarity::Positive> = 10;
template<> constexpr inline int8_t AdcChannel<DataC0, Peripheral::Adc3, AdcPolarity::Positive> = 10;

struct DataC1 {
	static constexpr Gpio::Port port = Gpio::Port::C;
	static constexpr uint8_t pin = 1;
	struct BitBang { using Data = DataC1; static constexpr Gpio::Signal Signal = Gpio::Signal::BitBang; };
	struct Ck { using Data = DataC1; static constexpr Gpio::Signal Signal = Gpio::Signal::Ck; };
	struct Ckin4 { using Data = DataC1; static constexpr Gpio::Signal Signal = Gpio::Signal::Ckin4; };
	struct D1 { using Data = DataC1; static constexpr Gpio::Signal Signal = Gpio::Signal::D1; };
	struct Datin0 { using Data = DataC1; static constexpr Gpio::Signal Signal = Gpio::Signal::Datin0; };
	struct G5 { using Data = DataC1; static constexpr Gpio::Signal Signal = Gpio::Signal::G5; };
	struct Inn10 { using Data = DataC1; static constexpr Gpio::Signal Signal = Gpio::Signal::Inn10; };
	struct Inp11 { using Data = DataC1; static constexpr Gpio::Signal Signal = Gpio::Signal::Inp11; };
	struct Mdc { using Data = DataC1; static constexpr Gpio::Signal Signal = Gpio::Signal::Mdc; };
	struct Mosi { using Data = DataC1; static constexpr Gpio::Signal Signal = Gpio::Signal::Mosi; };
	struct P1io4 { using Data = DataC1; static constexpr Gpio::Signal Signal = Gpio::Signal::P1io4; };
	struct Sda { using Data = DataC1; static constexpr Gpio::Signal Signal = Gpio::Signal::Sda; };
	struct Sdo { using Data = DataC1; static constexpr Gpio::Signal Signal = Gpio::Signal::Sdo; };
	struct Tamp3 { using Data = DataC1; static constexpr Gpio::Signal Signal = Gpio::Signal::Tamp3; };
	struct Traced0 { using Data = DataC1; static constexpr Gpio::Signal Signal = Gpio::Signal::Traced0; };
	struct Wkup6 { using Data = DataC1; static constexpr Gpio::Signal Signal = Gpio::Signal::Wkup6; };
};
template<Peripheral p> struct SignalConnection<DataC1::BitBang, p> {
	static_assert(p == Peripheral::BitBang, "GpioC1::BitBang only connects to software drivers!"); };
template<Peripheral p> struct SignalConnection<DataC1::Ck, p> {
	static_assert((p == Peripheral::Sdmmc2),"GpioC1::Ck only connects to Sdmmc2!"); };
template<Peripheral p> struct SignalConnection<DataC1::Ckin4, p> {
	static_assert((p == Peripheral::Dfsdm1),"GpioC1::Ckin4 only connects to Dfsdm1!"); };
template<Peripheral p> struct SignalConnection<DataC1::D1, p> {
	static_assert((p == Peripheral::Sai4) or (p == Peripheral::Sai1),
		"GpioC1::D1 only connects to Sai4 or Sai1!"); };
template<Peripheral p> struct SignalConnection<DataC1::Datin0, p> {
	static_assert((p == Peripheral::Dfsdm1),"GpioC1::Datin0 only connects to Dfsdm1!"); };
template<Peripheral p> struct SignalConnection<DataC1::G5, p> {
	static_assert((p == Peripheral::Ltdc),"GpioC1::G5 only connects to Ltdc!"); };
template<Peripheral p> struct SignalConnection<DataC1::Inn10, p> {
	static_assert((p == Peripheral::Adc1) or (p == Peripheral::Adc2) or (p == Peripheral::Adc3),
		"GpioC1::Inn10 only connects to Adc1 or Adc2 or Adc3!"); };
template<Peripheral p> struct SignalConnection<DataC1::Inp11, p> {
	static_assert((p == Peripheral::Adc1) or (p == Peripheral::Adc2) or (p == Peripheral::Adc3),
		"GpioC1::Inp11 only connects to Adc1 or Adc2 or Adc3!"); };
template<Peripheral p> struct SignalConnection<DataC1::Mdc, p> {
	static_assert((p == Peripheral::Eth) or (p == Peripheral::Mdios),
		"GpioC1::Mdc only connects to Eth or Mdios!"); };
template<Peripheral p> struct SignalConnection<DataC1::Mosi, p> {
	static_assert((p == Peripheral::Spi2),"GpioC1::Mosi only connects to Spi2!"); };
template<Peripheral p> struct SignalConnection<DataC1::P1io4, p> {
	static_assert((p == Peripheral::Octospim),"GpioC1::P1io4 only connects to Octospim!"); };
template<Peripheral p> struct SignalConnection<DataC1::Sda, p> {
	static_assert((p == Peripheral::Sai1) or (p == Peripheral::Sai4),
		"GpioC1::Sda only connects to Sai1 or Sai4!"); };
template<Peripheral p> struct SignalConnection<DataC1::Sdo, p> {
	static_assert((p == Peripheral::I2s2),"GpioC1::Sdo only connects to I2s2!"); };
template<Peripheral p> struct SignalConnection<DataC1::Tamp3, p> {
	static_assert((p == Peripheral::Rtc),"GpioC1::Tamp3 only connects to Rtc!"); };
template<Peripheral p> struct SignalConnection<DataC1::Traced0, p> {
	static_assert((p == Peripheral::Debug),"GpioC1::Traced0 only connects to Debug!"); };
template<Peripheral p> struct SignalConnection<DataC1::Wkup6, p> {
	static_assert((p == Peripheral::Pwr),"GpioC1::Wkup6 only connects to Pwr!"); };
template<> struct SignalConnection<DataC1::BitBang, Peripheral::BitBang> { static constexpr int8_t af = -1; };
template<> struct SignalConnection<DataC1::Ck, Peripheral::Sdmmc2> { static constexpr int8_t af = 9; };
template<> struct SignalConnection<DataC1::Ckin4, Peripheral::Dfsdm1> { static constexpr int8_t af = 4; };
template<> struct SignalConnection<DataC1::D1, Peripheral::Sai4> { static constexpr int8_t af = 1; };
template<> struct SignalConnection<DataC1::D1, Peripheral::Sai1> { static constexpr int8_t af = 2; };
template<> struct SignalConnection<DataC1::Datin0, Peripheral::Dfsdm1> { static constexpr int8_t af = 3; };
template<> struct SignalConnection<DataC1::G5, Peripheral::Ltdc> { static constexpr int8_t af = 14; };
template<> struct SignalConnection<DataC1::Inn10, Peripheral::Adc1> { static constexpr int8_t af = -2; };
template<> struct SignalConnection<DataC1::Inn10, Peripheral::Adc2> { static constexpr int8_t af = -2; };
template<> struct SignalConnection<DataC1::Inn10, Peripheral::Adc3> { static constexpr int8_t af = -2; };
template<> struct SignalConnection<DataC1::Inp11, Peripheral::Adc1> { static constexpr int8_t af = -2; };
template<> struct SignalConnection<DataC1::Inp11, Peripheral::Adc2> { static constexpr int8_t af = -2; };
template<> struct SignalConnection<DataC1::Inp11, Peripheral::Adc3> { static constexpr int8_t af = -2; };
template<> struct SignalConnection<DataC1::Mdc, Peripheral::Eth> { static constexpr int8_t af = 11; };
template<> struct SignalConnection<DataC1::Mdc, Peripheral::Mdios> { static constexpr int8_t af = 12; };
template<> struct SignalConnection<DataC1::Mosi, Peripheral::Spi2> { static constexpr int8_t af = 5; };
template<> struct SignalConnection<DataC1::P1io4, Peripheral::Octospim> { static constexpr int8_t af = 10; };
template<> struct SignalConnection<DataC1::Sda, Peripheral::Sai1> { static constexpr int8_t af = 6; };
template<> struct SignalConnection<DataC1::Sda, Peripheral::Sai4> { static constexpr int8_t af = 8; };
template<> struct SignalConnection<DataC1::Sdo, Peripheral::I2s2> { static constexpr int8_t af = 5; };
template<> struct SignalConnection<DataC1::Tamp3, Peripheral::Rtc> { static constexpr int8_t af = -1; };
template<> struct SignalConnection<DataC1::Traced0, Peripheral::Debug> { static constexpr int8_t af = 0; };
template<> struct SignalConnection<DataC1::Wkup6, Peripheral::Pwr> { static constexpr int8_t af = -1; };
template<> constexpr inline int8_t AdcChannel<DataC1, Peripheral::Adc1, AdcPolarity::Negative> = 10;
template<> constexpr inline int8_t AdcChannel<DataC1, Peripheral::Adc2, AdcPolarity::Negative> = 10;
template<> constexpr inline int8_t AdcChannel<DataC1, Peripheral::Adc3, AdcPolarity::Negative> = 10;
template<> constexpr inline int8_t AdcChannel<DataC1, Peripheral::Adc1, AdcPolarity::Positive> = 11;
template<> constexpr inline int8_t AdcChannel<DataC1, Peripheral::Adc2, AdcPolarity::Positive> = 11;
template<> constexpr inline int8_t AdcChannel<DataC1, Peripheral::Adc3, AdcPolarity::Positive> = 11;

struct DataC2 {
	static constexpr Gpio::Port port = Gpio::Port::C;
	static constexpr uint8_t pin = 2;
	struct BitBang { using Data = DataC2; static constexpr Gpio::Signal Signal = Gpio::Signal::BitBang; };
	struct Ckin1 { using Data = DataC2; static constexpr Gpio::Signal Signal = Gpio::Signal::Ckin1; };
	struct Ckout { using Data = DataC2; static constexpr Gpio::Signal Signal = Gpio::Signal::Ckout; };
	struct Cstop { using Data = DataC2; static constexpr Gpio::Signal Signal = Gpio::Signal::Cstop; };
	struct Inn1 { using Data = DataC2; static constexpr Gpio::Signal Signal = Gpio::Signal::Inn1; };
	struct Inp0 { using Data = DataC2; static constexpr Gpio::Signal Signal = Gpio::Signal::Inp0; };
	struct Miso { using Data = DataC2; static constexpr Gpio::Signal Signal = Gpio::Signal::Miso; };
	struct P1io2 { using Data = DataC2; static constexpr Gpio::Signal Signal = Gpio::Signal::P1io2; };
	struct P1io5 { using Data = DataC2; static constexpr Gpio::Signal Signal = Gpio::Signal::P1io5; };
	struct Sdi { using Data = DataC2; static constexpr Gpio::Signal Signal = Gpio::Signal::Sdi; };
	struct Sdne0 { using Data = DataC2; static constexpr Gpio::Signal Signal = Gpio::Signal::Sdne0; };
	struct Txd2 { using Data = DataC2; static constexpr Gpio::Signal Signal = Gpio::Signal::Txd2; };
	struct Ulpidir { using Data = DataC2; static constexpr Gpio::Signal Signal = Gpio::Signal::Ulpidir; };
};
template<Peripheral p> struct SignalConnection<DataC2::BitBang, p> {
	static_assert(p == Peripheral::BitBang, "GpioC2::BitBang only connects to software drivers!"); };
template<Peripheral p> struct SignalConnection<DataC2::Ckin1, p> {
	static_assert((p == Peripheral::Dfsdm1),"GpioC2::Ckin1 only connects to Dfsdm1!"); };
template<Peripheral p> struct SignalConnection<DataC2::Ckout, p> {
	static_assert((p == Peripheral::Dfsdm1),"GpioC2::Ckout only connects to Dfsdm1!"); };
template<Peripheral p> struct SignalConnection<DataC2::Cstop, p> {
	static_assert((p == Peripheral::Pwr),"GpioC2::Cstop only connects to Pwr!"); };
template<Peripheral p> struct SignalConnection<DataC2::Inn1, p> {
	static_assert((p == Peripheral::Adc3),"GpioC2::Inn1 only connects to Adc3!"); };
template<Peripheral p> struct SignalConnection<DataC2::Inp0, p> {
	static_assert((p == Peripheral::Adc3),"GpioC2::Inp0 only connects to Adc3!"); };
template<Peripheral p> struct SignalConnection<DataC2::Miso, p> {
	static_assert((p == Peripheral::Spi2),"GpioC2::Miso only connects to Spi2!"); };
template<Peripheral p> struct SignalConnection<DataC2::P1io2, p> {
	static_assert((p == Peripheral::Octospim),"GpioC2::P1io2 only connects to Octospim!"); };
template<Peripheral p> struct SignalConnection<DataC2::P1io5, p> {
	static_assert((p == Peripheral::Octospim),"GpioC2::P1io5 only connects to Octospim!"); };
template<Peripheral p> struct SignalConnection<DataC2::Sdi, p> {
	static_assert((p == Peripheral::I2s2),"GpioC2::Sdi only connects to I2s2!"); };
template<Peripheral p> struct SignalConnection<DataC2::Sdne0, p> {
	static_assert((p == Peripheral::Fmc),"GpioC2::Sdne0 only connects to Fmc!"); };
template<Peripheral p> struct SignalConnection<DataC2::Txd2, p> {
	static_assert((p == Peripheral::Eth),"GpioC2::Txd2 only connects to Eth!"); };
template<Peripheral p> struct SignalConnection<DataC2::Ulpidir, p> {
	static_assert((p == Peripheral::Usbotghs),"GpioC2::Ulpidir only connects to Usbotghs!"); };
template<> struct SignalConnection<DataC2::BitBang, Peripheral::BitBang> { static constexpr int8_t af = -1; };
template<> struct SignalConnection<DataC2::Ckin1, Peripheral::Dfsdm1> { static constexpr int8_t af = 3; };
template<> struct SignalConnection<DataC2::Ckout, Peripheral::Dfsdm1> { static constexpr int8_t af = 6; };
template<> struct SignalConnection<DataC2::Cstop, Peripheral::Pwr> { static constexpr int8_t af = -1; };
template<> struct SignalConnection<DataC2::Inn1, Peripheral::Adc3> { static constexpr int8_t af = -2; };
template<> struct SignalConnection<DataC2::Inp0, Peripheral::Adc3> { static constexpr int8_t af = -2; };
template<> struct SignalConnection<DataC2::Miso, Peripheral::Spi2> { static constexpr int8_t af = 5; };
template<> struct SignalConnection<DataC2::P1io2, Peripheral::Octospim> { static constexpr int8_t af = 9; };
template<> struct SignalConnection<DataC2::P1io5, Peripheral::Octospim> { static constexpr int8_t af = 4; };
template<> struct SignalConnection<DataC2::Sdi, Peripheral::I2s2> { static constexpr int8_t af = 5; };
template<> struct SignalConnection<DataC2::Sdne0, Peripheral::Fmc> { static constexpr int8_t af = 12; };
template<> struct SignalConnection<DataC2::Txd2, Peripheral::Eth> { static constexpr int8_t af = 11; };
template<> struct SignalConnection<DataC2::Ulpidir, Peripheral::Usbotghs> { static constexpr int8_t af = 10; };
template<> constexpr inline int8_t AdcChannel<DataC2, Peripheral::Adc3, AdcPolarity::Negative> = 1;
template<> constexpr inline int8_t AdcChannel<DataC2, Peripheral::Adc3, AdcPolarity::Positive> = 0;

struct DataC3 {
	static constexpr Gpio::Port port = Gpio::Port::C;
	static constexpr uint8_t pin = 3;
	struct BitBang { using Data = DataC3; static constexpr Gpio::Signal Signal = Gpio::Signal::BitBang; };
	struct Csleep { using Data = DataC3; static constexpr Gpio::Signal Signal = Gpio::Signal::Csleep; };
	struct Datin1 { using Data = DataC3; static constexpr Gpio::Signal Signal = Gpio::Signal::Datin1; };
	struct Inp1 { using Data = DataC3; static constexpr Gpio::Signal Signal = Gpio::Signal::Inp1; };
	struct Mosi { using Data = DataC3; static constexpr Gpio::Signal Signal = Gpio::Signal::Mosi; };
	struct P1io0 { using Data = DataC3; static constexpr Gpio::Signal Signal = Gpio::Signal::P1io0; };
	struct P1io6 { using Data = DataC3; static constexpr Gpio::Signal Signal = Gpio::Signal::P1io6; };
	struct Sdcke0 { using Data = DataC3; static constexpr Gpio::Signal Signal = Gpio::Signal::Sdcke0; };
	struct Sdo { using Data = DataC3; static constexpr Gpio::Signal Signal = Gpio::Signal::Sdo; };
	struct Txclk { using Data = DataC3; static constexpr Gpio::Signal Signal = Gpio::Signal::Txclk; };
	struct Ulpinxt { using Data = DataC3; static constexpr Gpio::Signal Signal = Gpio::Signal::Ulpinxt; };
};
template<Peripheral p> struct SignalConnection<DataC3::BitBang, p> {
	static_assert(p == Peripheral::BitBang, "GpioC3::BitBang only connects to software drivers!"); };
template<Peripheral p> struct SignalConnection<DataC3::Csleep, p> {
	static_assert((p == Peripheral::Pwr),"GpioC3::Csleep only connects to Pwr!"); };
template<Peripheral p> struct SignalConnection<DataC3::Datin1, p> {
	static_assert((p == Peripheral::Dfsdm1),"GpioC3::Datin1 only connects to Dfsdm1!"); };
template<Peripheral p> struct SignalConnection<DataC3::Inp1, p> {
	static_assert((p == Peripheral::Adc3),"GpioC3::Inp1 only connects to Adc3!"); };
template<Peripheral p> struct SignalConnection<DataC3::Mosi, p> {
	static_assert((p == Peripheral::Spi2),"GpioC3::Mosi only connects to Spi2!"); };
template<Peripheral p> struct SignalConnection<DataC3::P1io0, p> {
	static_assert((p == Peripheral::Octospim),"GpioC3::P1io0 only connects to Octospim!"); };
template<Peripheral p> struct SignalConnection<DataC3::P1io6, p> {
	static_assert((p == Peripheral::Octospim),"GpioC3::P1io6 only connects to Octospim!"); };
template<Peripheral p> struct SignalConnection<DataC3::Sdcke0, p> {
	static_assert((p == Peripheral::Fmc),"GpioC3::Sdcke0 only connects to Fmc!"); };
template<Peripheral p> struct SignalConnection<DataC3::Sdo, p> {
	static_assert((p == Peripheral::I2s2),"GpioC3::Sdo only connects to I2s2!"); };
template<Peripheral p> struct SignalConnection<DataC3::Txclk, p> {
	static_assert((p == Peripheral::Eth),"GpioC3::Txclk only connects to Eth!"); };
template<Peripheral p> struct SignalConnection<DataC3::Ulpinxt, p> {
	static_assert((p == Peripheral::Usbotghs),"GpioC3::Ulpinxt only connects to Usbotghs!"); };
template<> struct SignalConnection<DataC3::BitBang, Peripheral::BitBang> { static constexpr int8_t af = -1; };
template<> struct SignalConnection<DataC3::Csleep, Peripheral::Pwr> { static constexpr int8_t af = -1; };
template<> struct SignalConnection<DataC3::Datin1, Peripheral::Dfsdm1> { static constexpr int8_t af = 3; };
template<> struct SignalConnection<DataC3::Inp1, Peripheral::Adc3> { static constexpr int8_t af = -2; };
template<> struct SignalConnection<DataC3::Mosi, Peripheral::Spi2> { static constexpr int8_t af = 5; };
template<> struct SignalConnection<DataC3::P1io0, Peripheral::Octospim> { static constexpr int8_t af = 9; };
template<> struct SignalConnection<DataC3::P1io6, Peripheral::Octospim> { static constexpr int8_t af = 4; };
template<> struct SignalConnection<DataC3::Sdcke0, Peripheral::Fmc> { static constexpr int8_t af = 12; };
template<> struct SignalConnection<DataC3::Sdo, Peripheral::I2s2> { static constexpr int8_t af = 5; };
template<> struct SignalConnection<DataC3::Txclk, Peripheral::Eth> { static constexpr int8_t af = 11; };
template<> struct SignalConnection<DataC3::Ulpinxt, Peripheral::Usbotghs> { static constexpr int8_t af = 10; };
template<> constexpr inline int8_t AdcChannel<DataC3, Peripheral::Adc3, AdcPolarity::Positive> = 1;

struct DataC4 {
	static constexpr Gpio::Port port = Gpio::Port::C;
	static constexpr uint8_t pin = 4;
	struct BitBang { using Data = DataC4; static constexpr Gpio::Signal Signal = Gpio::Signal::BitBang; };
	struct A22 { using Data = DataC4; static constexpr Gpio::Signal Signal = Gpio::Signal::A22; };
	struct Ckin { using Data = DataC4; static constexpr Gpio::Signal Signal = Gpio::Signal::Ckin; };
	struct Ckin2 { using Data = DataC4; static constexpr Gpio::Signal Signal = Gpio::Signal::Ckin2; };
	struct In2 { using Data = DataC4; static constexpr Gpio::Signal Signal = Gpio::Signal::In2; };
	struct Inm { using Data = DataC4; static constexpr Gpio::Signal Signal = Gpio::Signal::Inm; };
	struct Inp4 { using Data = DataC4; static constexpr Gpio::Signal Signal = Gpio::Signal::Inp4; };
	struct Mck { using Data = DataC4; static constexpr Gpio::Signal Signal = Gpio::Signal::Mck; };
	struct R7 { using Data = DataC4; static constexpr Gpio::Signal Signal = Gpio::Signal::R7; };
	struct Rxd0 { using Data = DataC4; static constexpr Gpio::Signal Signal = Gpio::Signal::Rxd0; };
	struct Sdne0 { using Data = DataC4; static constexpr Gpio::Signal Signal = Gpio::Signal::Sdne0; };
	struct Vout { using Data = DataC4; static constexpr Gpio::Signal Signal = Gpio::Signal::Vout; };
};
template<Peripheral p> struct SignalConnection<DataC4::BitBang, p> {
	static_assert(p == Peripheral::BitBang, "GpioC4::BitBang only connects to software drivers!"); };
template<Peripheral p> struct SignalConnection<DataC4::A22, p> {
	static_assert((p == Peripheral::Fmc),"GpioC4::A22 only connects to Fmc!"); };
template<Peripheral p> struct SignalConnection<DataC4::Ckin, p> {
	static_assert((p == Peripheral::Sdmmc2),"GpioC4::Ckin only connects to Sdmmc2!"); };
template<Peripheral p> struct SignalConnection<DataC4::Ckin2, p> {
	static_assert((p == Peripheral::Dfsdm1),"GpioC4::Ckin2 only connects to Dfsdm1!"); };
template<Peripheral p> struct SignalConnection<DataC4::In2, p> {
	static_assert((p == Peripheral::Spdifrx),"GpioC4::In2 only connects to Spdifrx!"); };
template<Peripheral p> struct SignalConnection<DataC4::Inm, p> {
	static_assert((p == Peripheral::Comp1),"GpioC4::Inm only connects to Comp1!"); };
template<Peripheral p> struct SignalConnection<DataC4::Inp4, p> {
	static_assert((p == Peripheral::Adc1) or (p == Peripheral::Adc2),
		"GpioC4::Inp4 only connects to Adc1 or Adc2!"); };
template<Peripheral p> struct SignalConnection<DataC4::Mck, p> {
	static_assert((p == Peripheral::I2s1),"GpioC4::Mck only connects to I2s1!"); };
template<Peripheral p> struct SignalConnection<DataC4::R7, p> {
	static_assert((p == Peripheral::Ltdc),"GpioC4::R7 only connects to Ltdc!"); };
template<Peripheral p> struct SignalConnection<DataC4::Rxd0, p> {
	static_assert((p == Peripheral::Eth),"GpioC4::Rxd0 only connects to Eth!"); };
template<Peripheral p> struct SignalConnection<DataC4::Sdne0, p> {
	static_assert((p == Peripheral::Fmc),"GpioC4::Sdne0 only connects to Fmc!"); };
template<Peripheral p> struct SignalConnection<DataC4::Vout, p> {
	static_assert((p == Peripheral::Opamp1),"GpioC4::Vout only connects to Opamp1!"); };
template<> struct SignalConnection<DataC4::BitBang, Peripheral::BitBang> { static constexpr int8_t af = -1; };
template<> struct SignalConnection<DataC4::A22, Peripheral::Fmc> { static constexpr int8_t af = 1; };
template<> struct SignalConnection<DataC4::Ckin, Peripheral::Sdmmc2> { static constexpr int8_t af = 10; };
template<> struct SignalConnection<DataC4::Ckin2, Peripheral::Dfsdm1> { static constexpr int8_t af = 3; };
template<> struct SignalConnection<DataC4::In2, Peripheral::Spdifrx> { static constexpr int8_t af = 9; };
template<> struct SignalConnection<DataC4::Inm, Peripheral::Comp1> { static constexpr int8_t af = -2; };
template<> struct SignalConnection<DataC4::Inp4, Peripheral::Adc1> { static constexpr int8_t af = -2; };
template<> struct SignalConnection<DataC4::Inp4, Peripheral::Adc2> { static constexpr int8_t af = -2; };
template<> struct SignalConnection<DataC4::Mck, Peripheral::I2s1> { static constexpr int8_t af = 5; };
template<> struct SignalConnection<DataC4::R7, Peripheral::Ltdc> { static constexpr int8_t af = 14; };
template<> struct SignalConnection<DataC4::Rxd0, Peripheral::Eth> { static constexpr int8_t af = 11; };
template<> struct SignalConnection<DataC4::Sdne0, Peripheral::Fmc> { static constexpr int8_t af = 12; };
template<> struct SignalConnection<DataC4::Vout, Peripheral::Opamp1> { static constexpr int8_t af = -1; };
template<> constexpr inline int8_t AdcChannel<DataC4, Peripheral::Adc1, AdcPolarity::Positive> = 4;
template<> constexpr inline int8_t AdcChannel<DataC4, Peripheral::Adc2, AdcPolarity::Positive> = 4;

struct DataC5 {
	static constexpr Gpio::Port port = Gpio::Port::C;
	static constexpr uint8_t pin = 5;
	struct BitBang { using Data = DataC5; static constexpr Gpio::Signal Signal = Gpio::Signal::BitBang; };
	struct D15 { using Data = DataC5; static constexpr Gpio::Signal Signal = Gpio::Signal::D15; };
	struct D3 { using Data = DataC5; static constexpr Gpio::Signal Signal = Gpio::Signal::D3; };
	struct Datin2 { using Data = DataC5; static constexpr Gpio::Signal Signal = Gpio::Signal::Datin2; };
	struct De { using Data = DataC5; static constexpr Gpio::Signal Signal = Gpio::Signal::De; };
	struct In3 { using Data = DataC5; static constexpr Gpio::Signal Signal = Gpio::Signal::In3; };
	struct Inn4 { using Data = DataC5; static constexpr Gpio::Signal Signal = Gpio::Signal::Inn4; };
	struct Inp8 { using Data = DataC5; static constexpr Gpio::Signal Signal = Gpio::Signal::Inp8; };
	struct Out { using Data = DataC5; static constexpr Gpio::Signal Signal = Gpio::Signal::Out; };
	struct P1dqs { using Data = DataC5; static constexpr Gpio::Signal Signal = Gpio::Signal::P1dqs; };
	struct Rxd1 { using Data = DataC5; static constexpr Gpio::Signal Signal = Gpio::Signal::Rxd1; };
	struct Sdcke0 { using Data = DataC5; static constexpr Gpio::Signal Signal = Gpio::Signal::Sdcke0; };
	struct Vinm { using Data = DataC5; static constexpr Gpio::Signal Signal = Gpio::Signal::Vinm; };
};
template<Peripheral p> struct SignalConnection<DataC5::BitBang, p> {
	static_assert(p == Peripheral::BitBang, "GpioC5::BitBang only connects to software drivers!"); };
template<Peripheral p> struct SignalConnection<DataC5::D15, p> {
	static_assert((p == Peripheral::Pssi),"GpioC5::D15 only connects to Pssi!"); };
template<Peripheral p> struct SignalConnection<DataC5::D3, p> {
	static_assert((p == Peripheral::Sai4) or (p == Peripheral::Sai1),
		"GpioC5::D3 only connects to Sai4 or Sai1!"); };
template<Peripheral p> struct SignalConnection<DataC5::Datin2, p> {
	static_assert((p == Peripheral::Dfsdm1),"GpioC5::Datin2 only connects to Dfsdm1!"); };
template<Peripheral p> struct SignalConnection<DataC5::De, p> {
	static_assert((p == Peripheral::Ltdc),"GpioC5::De only connects to Ltdc!"); };
template<Peripheral p> struct SignalConnection<DataC5::In3, p> {
	static_assert((p == Peripheral::Spdifrx),"GpioC5::In3 only connects to Spdifrx!"); };
template<Peripheral p> struct SignalConnection<DataC5::Inn4, p> {
	static_assert((p == Peripheral::Adc1) or (p == Peripheral::Adc2),
		"GpioC5::Inn4 only connects to Adc1 or Adc2!"); };
template<Peripheral p> struct SignalConnection<DataC5::Inp8, p> {
	static_assert((p == Peripheral::Adc1) or (p == Peripheral::Adc2),
		"GpioC5::Inp8 only connects to Adc1 or Adc2!"); };
template<Peripheral p> struct SignalConnection<DataC5::Out, p> {
	static_assert((p == Peripheral::Comp1),"GpioC5::Out only connects to Comp1!"); };
template<Peripheral p> struct SignalConnection<DataC5::P1dqs, p> {
	static_assert((p == Peripheral::Octospim),"GpioC5::P1dqs only connects to Octospim!"); };
template<Peripheral p> struct SignalConnection<DataC5::Rxd1, p> {
	static_assert((p == Peripheral::Eth),"GpioC5::Rxd1 only connects to Eth!"); };
template<Peripheral p> struct SignalConnection<DataC5::Sdcke0, p> {
	static_assert((p == Peripheral::Fmc),"GpioC5::Sdcke0 only connects to Fmc!"); };
template<Peripheral p> struct SignalConnection<DataC5::Vinm, p> {
	static_assert((p == Peripheral::Opamp1),"GpioC5::Vinm only connects to Opamp1!"); };
template<> struct SignalConnection<DataC5::BitBang, Peripheral::BitBang> { static constexpr int8_t af = -1; };
template<> struct SignalConnection<DataC5::D15, Peripheral::Pssi> { static constexpr int8_t af = 4; };
template<> struct SignalConnection<DataC5::D3, Peripheral::Sai4> { static constexpr int8_t af = 1; };
template<> struct SignalConnection<DataC5::D3, Peripheral::Sai1> { static constexpr int8_t af = 2; };
template<> struct SignalConnection<DataC5::Datin2, Peripheral::Dfsdm1> { static constexpr int8_t af = 3; };
template<> struct SignalConnection<DataC5::De, Peripheral::Ltdc> { static constexpr int8_t af = 14; };
template<> struct SignalConnection<DataC5::In3, Peripheral::Spdifrx> { static constexpr int8_t af = 9; };
template<> struct SignalConnection<DataC5::Inn4, Peripheral::Adc1> { static constexpr int8_t af = -2; };
template<> struct SignalConnection<DataC5::Inn4, Peripheral::Adc2> { static constexpr int8_t af = -2; };
template<> struct SignalConnection<DataC5::Inp8, Peripheral::Adc1> { static constexpr int8_t af = -2; };
template<> struct SignalConnection<DataC5::Inp8, Peripheral::Adc2> { static constexpr int8_t af = -2; };
template<> struct SignalConnection<DataC5::Out, Peripheral::Comp1> { static constexpr int8_t af = 13; };
template<> struct SignalConnection<DataC5::P1dqs, Peripheral::Octospim> { static constexpr int8_t af = 10; };
template<> struct SignalConnection<DataC5::Rxd1, Peripheral::Eth> { static constexpr int8_t af = 11; };
template<> struct SignalConnection<DataC5::Sdcke0, Peripheral::Fmc> { static constexpr int8_t af = 12; };
template<> struct SignalConnection<DataC5::Vinm, Peripheral::Opamp1> { static constexpr int8_t af = -1; };
template<> constexpr inline int8_t AdcChannel<DataC5, Peripheral::Adc1, AdcPolarity::Negative> = 4;
template<> constexpr inline int8_t AdcChannel<DataC5, Peripheral::Adc2, AdcPolarity::Negative> = 4;
template<> constexpr inline int8_t AdcChannel<DataC5, Peripheral::Adc1, AdcPolarity::Positive> = 8;
template<> constexpr inline int8_t AdcChannel<DataC5, Peripheral::Adc2, AdcPolarity::Positive> = 8;

struct DataC6 {
	static constexpr Gpio::Port port = Gpio::Port::C;
	static constexpr uint8_t pin = 6;
	struct BitBang { using Data = DataC6; static constexpr Gpio::Signal Signal = Gpio::Signal::BitBang; };
	struct Ch1 { using Data = DataC6; static constexpr Gpio::Signal Signal = Gpio::Signal::Ch1; };
	struct Ckin3 { using Data = DataC6; static constexpr Gpio::Signal Signal = Gpio::Signal::Ckin3; };
	struct D0 { using Data = DataC6; static constexpr Gpio::Signal Signal = Gpio::Signal::D0; };
	struct D0dir { using Data = DataC6; static constexpr Gpio::Signal Signal = Gpio::Signal::D0dir; };
	struct D6 { using Data = DataC6; static constexpr Gpio::Signal Signal = Gpio::Signal::D6; };
	struct Hsync { using Data = DataC6; static constexpr Gpio::Signal Signal = Gpio::Signal::Hsync; };
	struct Io { using Data = DataC6; static constexpr Gpio::Signal Signal = Gpio::Signal::Io; };
	struct Mck { using Data = DataC6; static constexpr Gpio::Signal Signal = Gpio::Signal::Mck; };
	struct Nwait { using Data = DataC6; static constexpr Gpio::Signal Signal = Gpio::Signal::Nwait; };
	struct Tx { using Data = DataC6; static constexpr Gpio::Signal Signal = Gpio::Signal::Tx; };
};
template<Peripheral p> struct SignalConnection<DataC6::BitBang, p> {
	static_assert(p == Peripheral::BitBang, "GpioC6::BitBang only connects to software drivers!"); };
template<Peripheral p> struct SignalConnection<DataC6::Ch1, p> {
	static_assert((p == Peripheral::Tim3) or (p == Peripheral::Tim8),
		"GpioC6::Ch1 only connects to Tim3 or Tim8!"); };
template<Peripheral p> struct SignalConnection<DataC6::Ckin3, p> {
	static_assert((p == Peripheral::Dfsdm1),"GpioC6::Ckin3 only connects to Dfsdm1!"); };
template<Peripheral p> struct SignalConnection<DataC6::D0, p> {
	static_assert((p == Peripheral::Dcmi) or (p == Peripheral::Pssi),
		"GpioC6::D0 only connects to Dcmi or Pssi!"); };
template<Peripheral p> struct SignalConnection<DataC6::D0dir, p> {
	static_assert((p == Peripheral::Sdmmc1),"GpioC6::D0dir only connects to Sdmmc1!"); };
template<Peripheral p> struct SignalConnection<DataC6::D6, p> {
	static_assert((p == Peripheral::Sdmmc2) or (p == Peripheral::Sdmmc1),
		"GpioC6::D6 only connects to Sdmmc2 or Sdmmc1!"); };
template<Peripheral p> struct SignalConnection<DataC6::Hsync, p> {
	static_assert((p == Peripheral::Ltdc),"GpioC6::Hsync only connects to Ltdc!"); };
template<Peripheral p> struct SignalConnection<DataC6::Io, p> {
	static_assert((p == Peripheral::Swpmi1),"GpioC6::Io only connects to Swpmi1!"); };
template<Peripheral p> struct SignalConnection<DataC6::Mck, p> {
	static_assert((p == Peripheral::I2s2),"GpioC6::Mck only connects to I2s2!"); };
template<Peripheral p> struct SignalConnection<DataC6::Nwait, p> {
	static_assert((p == Peripheral::Fmc),"GpioC6::Nwait only connects to Fmc!"); };
template<Peripheral p> struct SignalConnection<DataC6::Tx, p> {
	static_assert((p == Peripheral::Usart6),"GpioC6::Tx only connects to Usart6!"); };
template<> struct SignalConnection<DataC6::BitBang, Peripheral::BitBang> { static constexpr int8_t af = -1; };
template<> struct SignalConnection<DataC6::Ch1, Peripheral::Tim3> { static constexpr int8_t af = 2; };
template<> struct SignalConnection<DataC6::Ch1, Peripheral::Tim8> { static constexpr int8_t af = 3; };
template<> struct SignalConnection<DataC6::Ckin3, Peripheral::Dfsdm1> { static constexpr int8_t af = 4; };
template<> struct SignalConnection<DataC6::D0, Peripheral::Dcmi> { static constexpr int8_t af = 13; };
template<> struct SignalConnection<DataC6::D0, Peripheral::Pssi> { static constexpr int8_t af = 13; };
template<> struct SignalConnection<DataC6::D0dir, Peripheral::Sdmmc1> { static constexpr int8_t af = 8; };
template<> struct SignalConnection<DataC6::D6, Peripheral::Sdmmc2> { static constexpr int8_t af = 10; };
template<> struct SignalConnection<DataC6::D6, Peripheral::Sdmmc1> { static constexpr int8_t af = 12; };
template<> struct SignalConnection<DataC6::Hsync, Peripheral::Ltdc> { static constexpr int8_t af = 14; };
template<> struct SignalConnection<DataC6::Io, Peripheral::Swpmi1> { static constexpr int8_t af = -1; };
template<> struct SignalConnection<DataC6::Mck, Peripheral::I2s2> { static constexpr int8_t af = 5; };
template<> struct SignalConnection<DataC6::Nwait, Peripheral::Fmc> { static constexpr int8_t af = 9; };
template<> struct SignalConnection<DataC6::Tx, Peripheral::Usart6> { static constexpr int8_t af = 7; };

struct DataC7 {
	static constexpr Gpio::Port port = Gpio::Port::C;
	static constexpr uint8_t pin = 7;
	struct BitBang { using Data = DataC7; static constexpr Gpio::Signal Signal = Gpio::Signal::BitBang; };
	struct Ch2 { using Data = DataC7; static constexpr Gpio::Signal Signal = Gpio::Signal::Ch2; };
	struct D1 { using Data = DataC7; static constexpr Gpio::Signal Signal = Gpio::Signal::D1; };
	struct D123dir { using Data = DataC7; static constexpr Gpio::Signal Signal = Gpio::Signal::D123dir; };
	struct D7 { using Data = DataC7; static constexpr Gpio::Signal Signal = Gpio::Signal::D7; };
	struct Datin3 { using Data = DataC7; static constexpr Gpio::Signal Signal = Gpio::Signal::Datin3; };
	struct G6 { using Data = DataC7; static constexpr Gpio::Signal Signal = Gpio::Signal::G6; };
	struct Mck { using Data = DataC7; static constexpr Gpio::Signal Signal = Gpio::Signal::Mck; };
	struct Ne1 { using Data = DataC7; static constexpr Gpio::Signal Signal = Gpio::Signal::Ne1; };
	struct Rx { using Data = DataC7; static constexpr Gpio::Signal Signal = Gpio::Signal::Rx; };
	struct Trgio { using Data = DataC7; static constexpr Gpio::Signal Signal = Gpio::Signal::Trgio; };
	struct Tx { using Data = DataC7; static constexpr Gpio::Signal Signal = Gpio::Signal::Tx; };
};
template<Peripheral p> struct SignalConnection<DataC7::BitBang, p> {
	static_assert(p == Peripheral::BitBang, "GpioC7::BitBang only connects to software drivers!"); };
template<Peripheral p> struct SignalConnection<DataC7::Ch2, p> {
	static_assert((p == Peripheral::Tim3) or (p == Peripheral::Tim8),
		"GpioC7::Ch2 only connects to Tim3 or Tim8!"); };
template<Peripheral p> struct SignalConnection<DataC7::D1, p> {
	static_assert((p == Peripheral::Dcmi) or (p == Peripheral::Pssi),
		"GpioC7::D1 only connects to Dcmi or Pssi!"); };
template<Peripheral p> struct SignalConnection<DataC7::D123dir, p> {
	static_assert((p == Peripheral::Sdmmc1),"GpioC7::D123dir only connects to Sdmmc1!"); };
template<Peripheral p> struct SignalConnection<DataC7::D7, p> {
	static_assert((p == Peripheral::Sdmmc2) or (p == Peripheral::Sdmmc1),
		"GpioC7::D7 only connects to Sdmmc2 or Sdmmc1!"); };
template<Peripheral p> struct SignalConnection<DataC7::Datin3, p> {
	static_assert((p == Peripheral::Dfsdm1),"GpioC7::Datin3 only connects to Dfsdm1!"); };
template<Peripheral p> struct SignalConnection<DataC7::G6, p> {
	static_assert((p == Peripheral::Ltdc),"GpioC7::G6 only connects to Ltdc!"); };
template<Peripheral p> struct SignalConnection<DataC7::Mck, p> {
	static_assert((p == Peripheral::I2s3),"GpioC7::Mck only connects to I2s3!"); };
template<Peripheral p> struct SignalConnection<DataC7::Ne1, p> {
	static_assert((p == Peripheral::Fmc),"GpioC7::Ne1 only connects to Fmc!"); };
template<Peripheral p> struct SignalConnection<DataC7::Rx, p> {
	static_assert((p == Peripheral::Usart6),"GpioC7::Rx only connects to Usart6!"); };
template<Peripheral p> struct SignalConnection<DataC7::Trgio, p> {
	static_assert((p == Peripheral::Debug),"GpioC7::Trgio only connects to Debug!"); };
template<Peripheral p> struct SignalConnection<DataC7::Tx, p> {
	static_assert((p == Peripheral::Swpmi1),"GpioC7::Tx only connects to Swpmi1!"); };
template<> struct SignalConnection<DataC7::BitBang, Peripheral::BitBang> { static constexpr int8_t af = -1; };
template<> struct SignalConnection<DataC7::Ch2, Peripheral::Tim3> { static constexpr int8_t af = 2; };
template<> struct SignalConnection<DataC7::Ch2, Peripheral::Tim8> { static constexpr int8_t af = 3; };
template<> struct SignalConnection<DataC7::D1, Peripheral::Dcmi> { static constexpr int8_t af = 13; };
template<> struct SignalConnection<DataC7::D1, Peripheral::Pssi> { static constexpr int8_t af = 13; };
template<> struct SignalConnection<DataC7::D123dir, Peripheral::Sdmmc1> { static constexpr int8_t af = 8; };
template<> struct SignalConnection<DataC7::D7, Peripheral::Sdmmc2> { static constexpr int8_t af = 10; };
template<> struct SignalConnection<DataC7::D7, Peripheral::Sdmmc1> { static constexpr int8_t af = 12; };
template<> struct SignalConnection<DataC7::Datin3, Peripheral::Dfsdm1> { static constexpr int8_t af = 4; };
template<> struct SignalConnection<DataC7::G6, Peripheral::Ltdc> { static constexpr int8_t af = 14; };
template<> struct SignalConnection<DataC7::Mck, Peripheral::I2s3> { static constexpr int8_t af = 6; };
template<> struct SignalConnection<DataC7::Ne1, Peripheral::Fmc> { static constexpr int8_t af = 9; };
template<> struct SignalConnection<DataC7::Rx, Peripheral::Usart6> { static constexpr int8_t af = 7; };
template<> struct SignalConnection<DataC7::Trgio, Peripheral::Debug> { static constexpr int8_t af = 0; };
template<> struct SignalConnection<DataC7::Tx, Peripheral::Swpmi1> { static constexpr int8_t af = 11; };

struct DataC8 {
	static constexpr Gpio::Port port = Gpio::Port::C;
	static constexpr uint8_t pin = 8;
	struct BitBang { using Data = DataC8; static constexpr Gpio::Signal Signal = Gpio::Signal::BitBang; };
	struct Ch3 { using Data = DataC8; static constexpr Gpio::Signal Signal = Gpio::Signal::Ch3; };
	struct Ck { using Data = DataC8; static constexpr Gpio::Signal Signal = Gpio::Signal::Ck; };
	struct D0 { using Data = DataC8; static constexpr Gpio::Signal Signal = Gpio::Signal::D0; };
	struct D2 { using Data = DataC8; static constexpr Gpio::Signal Signal = Gpio::Signal::D2; };
	struct De { using Data = DataC8; static constexpr Gpio::Signal Signal = Gpio::Signal::De; };
	struct Int { using Data = DataC8; static constexpr Gpio::Signal Signal = Gpio::Signal::Int; };
	struct Nce { using Data = DataC8; static constexpr Gpio::Signal Signal = Gpio::Signal::Nce; };
	struct Ne2 { using Data = DataC8; static constexpr Gpio::Signal Signal = Gpio::Signal::Ne2; };
	struct Rts { using Data = DataC8; static constexpr Gpio::Signal Signal = Gpio::Signal::Rts; };
	struct Rx { using Data = DataC8; static constexpr Gpio::Signal Signal = Gpio::Signal::Rx; };
	struct Traced1 { using Data = DataC8; static constexpr Gpio::Signal Signal = Gpio::Signal::Traced1; };
};
template<Peripheral p> struct SignalConnection<DataC8::BitBang, p> {
	static_assert(p == Peripheral::BitBang, "GpioC8::BitBang only connects to software drivers!"); };
template<Peripheral p> struct SignalConnection<DataC8::Ch3, p> {
	static_assert((p == Peripheral::Tim3) or (p == Peripheral::Tim8),
		"GpioC8::Ch3 only connects to Tim3 or Tim8!"); };
template<Peripheral p> struct SignalConnection<DataC8::Ck, p> {
	static_assert((p == Peripheral::Usart6),"GpioC8::Ck only connects to Usart6!"); };
template<Peripheral p> struct SignalConnection<DataC8::D0, p> {
	static_assert((p == Peripheral::Sdmmc1),"GpioC8::D0 only connects to Sdmmc1!"); };
template<Peripheral p> struct SignalConnection<DataC8::D2, p> {
	static_assert((p == Peripheral::Dcmi) or (p == Peripheral::Pssi),
		"GpioC8::D2 only connects to Dcmi or Pssi!"); };
template<Peripheral p> struct SignalConnection<DataC8::De, p> {
	static_assert((p == Peripheral::Uart5),"GpioC8::De only connects to Uart5!"); };
template<Peripheral p> struct SignalConnection<DataC8::Int, p> {
	static_assert((p == Peripheral::Fmc),"GpioC8::Int only connects to Fmc!"); };
template<Peripheral p> struct SignalConnection<DataC8::Nce, p> {
	static_assert((p == Peripheral::Fmc),"GpioC8::Nce only connects to Fmc!"); };
template<Peripheral p> struct SignalConnection<DataC8::Ne2, p> {
	static_assert((p == Peripheral::Fmc),"GpioC8::Ne2 only connects to Fmc!"); };
template<Peripheral p> struct SignalConnection<DataC8::Rts, p> {
	static_assert((p == Peripheral::Uart5),"GpioC8::Rts only connects to Uart5!"); };
template<Peripheral p> struct SignalConnection<DataC8::Rx, p> {
	static_assert((p == Peripheral::Swpmi1),"GpioC8::Rx only connects to Swpmi1!"); };
template<Peripheral p> struct SignalConnection<DataC8::Traced1, p> {
	static_assert((p == Peripheral::Debug),"GpioC8::Traced1 only connects to Debug!"); };
template<> struct SignalConnection<DataC8::BitBang, Peripheral::BitBang> { static constexpr int8_t af = -1; };
template<> struct SignalConnection<DataC8::Ch3, Peripheral::Tim3> { static constexpr int8_t af = 2; };
template<> struct SignalConnection<DataC8::Ch3, Peripheral::Tim8> { static constexpr int8_t af = 3; };
template<> struct SignalConnection<DataC8::Ck, Peripheral::Usart6> { static constexpr int8_t af = 7; };
template<> struct SignalConnection<DataC8::D0, Peripheral::Sdmmc1> { static constexpr int8_t af = 12; };
template<> struct SignalConnection<DataC8::D2, Peripheral::Dcmi> { static constexpr int8_t af = 13; };
template<> struct SignalConnection<DataC8::D2, Peripheral::Pssi> { static constexpr int8_t af = 13; };
template<> struct SignalConnection<DataC8::De, Peripheral::Uart5> { static constexpr int8_t af = 8; };
template<> struct SignalConnection<DataC8::Int, Peripheral::Fmc> { static constexpr int8_t af = 10; };
template<> struct SignalConnection<DataC8::Nce, Peripheral::Fmc> { static constexpr int8_t af = 9; };
template<> struct SignalConnection<DataC8::Ne2, Peripheral::Fmc> { static constexpr int8_t af = 9; };
template<> struct SignalConnection<DataC8::Rts, Peripheral::Uart5> { static constexpr int8_t af = 8; };
template<> struct SignalConnection<DataC8::Rx, Peripheral::Swpmi1> { static constexpr int8_t af = 11; };
template<> struct SignalConnection<DataC8::Traced1, Peripheral::Debug> { static constexpr int8_t af = 0; };

struct DataC9 {
	static constexpr Gpio::Port port = Gpio::Port::C;
	static constexpr uint8_t pin = 9;
	struct BitBang { using Data = DataC9; static constexpr Gpio::Signal Signal = Gpio::Signal::BitBang; };
	struct B2 { using Data = DataC9; static constexpr Gpio::Signal Signal = Gpio::Signal::B2; };
	struct Ch4 { using Data = DataC9; static constexpr Gpio::Signal Signal = Gpio::Signal::Ch4; };
	struct Ckin { using Data = DataC9; static constexpr Gpio::Signal Signal = Gpio::Signal::Ckin; };
	struct Cts { using Data = DataC9; static constexpr Gpio::Signal Signal = Gpio::Signal::Cts; };
	struct D1 { using Data = DataC9; static constexpr Gpio::Signal Signal = Gpio::Signal::D1; };
	struct D3 { using Data = DataC9; static constexpr Gpio::Signal Signal = Gpio::Signal::D3; };
	struct G3 { using Data = DataC9; static constexpr Gpio::Signal Signal = Gpio::Signal::G3; };
	struct Mco2 { using Data = DataC9; static constexpr Gpio::Signal Signal = Gpio::Signal::Mco2; };
	struct P1io0 { using Data = DataC9; static constexpr Gpio::Signal Signal = Gpio::Signal::P1io0; };
	struct Sda { using Data = DataC9; static constexpr Gpio::Signal Signal = Gpio::Signal::Sda; };
	struct Suspend { using Data = DataC9; static constexpr Gpio::Signal Signal = Gpio::Signal::Suspend; };
};
template<Peripheral p> struct SignalConnection<DataC9::BitBang, p> {
	static_assert(p == Peripheral::BitBang, "GpioC9::BitBang only connects to software drivers!"); };
template<Peripheral p> struct SignalConnection<DataC9::B2, p> {
	static_assert((p == Peripheral::Ltdc),"GpioC9::B2 only connects to Ltdc!"); };
template<Peripheral p> struct SignalConnection<DataC9::Ch4, p> {
	static_assert((p == Peripheral::Tim3) or (p == Peripheral::Tim8),
		"GpioC9::Ch4 only connects to Tim3 or Tim8!"); };
template<Peripheral p> struct SignalConnection<DataC9::Ckin, p> {
	static_assert((p == Peripheral::I2s),"GpioC9::Ckin only connects to I2s!"); };
template<Peripheral p> struct SignalConnection<DataC9::Cts, p> {
	static_assert((p == Peripheral::Uart5),"GpioC9::Cts only connects to Uart5!"); };
template<Peripheral p> struct SignalConnection<DataC9::D1, p> {
	static_assert((p == Peripheral::Sdmmc1),"GpioC9::D1 only connects to Sdmmc1!"); };
template<Peripheral p> struct SignalConnection<DataC9::D3, p> {
	static_assert((p == Peripheral::Dcmi) or (p == Peripheral::Pssi),
		"GpioC9::D3 only connects to Dcmi or Pssi!"); };
template<Peripheral p> struct SignalConnection<DataC9::G3, p> {
	static_assert((p == Peripheral::Ltdc),"GpioC9::G3 only connects to Ltdc!"); };
template<Peripheral p> struct SignalConnection<DataC9::Mco2, p> {
	static_assert((p == Peripheral::Rcc),"GpioC9::Mco2 only connects to Rcc!"); };
template<Peripheral p> struct SignalConnection<DataC9::P1io0, p> {
	static_assert((p == Peripheral::Octospim),"GpioC9::P1io0 only connects to Octospim!"); };
template<Peripheral p> struct SignalConnection<DataC9::Sda, p> {
	static_assert((p == Peripheral::I2c3) or (p == Peripheral::I2c5),
		"GpioC9::Sda only connects to I2c3 or I2c5!"); };
template<Peripheral p> struct SignalConnection<DataC9::Suspend, p> {
	static_assert((p == Peripheral::Swpmi1),"GpioC9::Suspend only connects to Swpmi1!"); };
template<> struct SignalConnection<DataC9::BitBang, Peripheral::BitBang> { static constexpr int8_t af = -1; };
template<> struct SignalConnection<DataC9::B2, Peripheral::Ltdc> { static constexpr int8_t af = 14; };
template<> struct SignalConnection<DataC9::Ch4, Peripheral::Tim3> { static constexpr int8_t af = 2; };
template<> struct SignalConnection<DataC9::Ch4, Peripheral::Tim8> { static constexpr int8_t af = 3; };
template<> struct SignalConnection<DataC9::Ckin, Peripheral::I2s> { static constexpr int8_t af = 5; };
template<> struct SignalConnection<DataC9::Cts, Peripheral::Uart5> { static constexpr int8_t af = 8; };
template<> struct SignalConnection<DataC9::D1, Peripheral::Sdmmc1> { static constexpr int8_t af = 12; };
template<> struct SignalConnection<DataC9::D3, Peripheral::Dcmi> { static constexpr int8_t af = 13; };
template<> struct SignalConnection<DataC9::D3, Peripheral::Pssi> { static constexpr int8_t af = 13; };
template<> struct SignalConnection<DataC9::G3, Peripheral::Ltdc> { static constexpr int8_t af = 10; };
template<> struct SignalConnection<DataC9::Mco2, Peripheral::Rcc> { static constexpr int8_t af = 0; };
template<> struct SignalConnection<DataC9::P1io0, Peripheral::Octospim> { static constexpr int8_t af = 9; };
template<> struct SignalConnection<DataC9::Sda, Peripheral::I2c3> { static constexpr int8_t af = 4; };
template<> struct SignalConnection<DataC9::Sda, Peripheral::I2c5> { static constexpr int8_t af = 6; };
template<> struct SignalConnection<DataC9::Suspend, Peripheral::Swpmi1> { static constexpr int8_t af = 11; };

struct DataC10 {
	static constexpr Gpio::Port port = Gpio::Port::C;
	static constexpr uint8_t pin = 10;
	struct BitBang { using Data = DataC10; static constexpr Gpio::Signal Signal = Gpio::Signal::BitBang; };
	struct B1 { using Data = DataC10; static constexpr Gpio::Signal Signal = Gpio::Signal::B1; };
	struct Ck { using Data = DataC10; static constexpr Gpio::Signal Signal = Gpio::Signal::Ck; };
	struct Ckin5 { using Data = DataC10; static constexpr Gpio::Signal Signal = Gpio::Signal::Ckin5; };
	struct D2 { using Data = DataC10; static constexpr Gpio::Signal Signal = Gpio::Signal::D2; };
	struct D8 { using Data = DataC10; static constexpr Gpio::Signal Signal = Gpio::Signal::D8; };
	struct P1io1 { using Data = DataC10; static constexpr Gpio::Signal Signal = Gpio::Signal::P1io1; };
	struct R2 { using Data = DataC10; static constexpr Gpio::Signal Signal = Gpio::Signal::R2; };
	struct Rx { using Data = DataC10; static constexpr Gpio::Signal Signal = Gpio::Signal::Rx; };
	struct Sck { using Data = DataC10; static constexpr Gpio::Signal Signal = Gpio::Signal::Sck; };
	struct Sda { using Data = DataC10; static constexpr Gpio::Signal Signal = Gpio::Signal::Sda; };
	struct Tx { using Data = DataC10; static constexpr Gpio::Signal Signal = Gpio::Signal::Tx; };
};
template<Peripheral p> struct SignalConnection<DataC10::BitBang, p> {
	static_assert(p == Peripheral::BitBang, "GpioC10::BitBang only connects to software drivers!"); };
template<Peripheral p> struct SignalConnection<DataC10::B1, p> {
	static_assert((p == Peripheral::Ltdc),"GpioC10::B1 only connects to Ltdc!"); };
template<Peripheral p> struct SignalConnection<DataC10::Ck, p> {
	static_assert((p == Peripheral::I2s3),"GpioC10::Ck only connects to I2s3!"); };
template<Peripheral p> struct SignalConnection<DataC10::Ckin5, p> {
	static_assert((p == Peripheral::Dfsdm1),"GpioC10::Ckin5 only connects to Dfsdm1!"); };
template<Peripheral p> struct SignalConnection<DataC10::D2, p> {
	static_assert((p == Peripheral::Sdmmc1),"GpioC10::D2 only connects to Sdmmc1!"); };
template<Peripheral p> struct SignalConnection<DataC10::D8, p> {
	static_assert((p == Peripheral::Dcmi) or (p == Peripheral::Pssi),
		"GpioC10::D8 only connects to Dcmi or Pssi!"); };
template<Peripheral p> struct SignalConnection<DataC10::P1io1, p> {
	static_assert((p == Peripheral::Octospim),"GpioC10::P1io1 only connects to Octospim!"); };
template<Peripheral p> struct SignalConnection<DataC10::R2, p> {
	static_assert((p == Peripheral::Ltdc),"GpioC10::R2 only connects to Ltdc!"); };
template<Peripheral p> struct SignalConnection<DataC10::Rx, p> {
	static_assert((p == Peripheral::Swpmi1),"GpioC10::Rx only connects to Swpmi1!"); };
template<Peripheral p> struct SignalConnection<DataC10::Sck, p> {
	static_assert((p == Peripheral::Spi3),"GpioC10::Sck only connects to Spi3!"); };
template<Peripheral p> struct SignalConnection<DataC10::Sda, p> {
	static_assert((p == Peripheral::I2c5),"GpioC10::Sda only connects to I2c5!"); };
template<Peripheral p> struct SignalConnection<DataC10::Tx, p> {
	static_assert((p == Peripheral::Usart3) or (p == Peripheral::Uart4),
		"GpioC10::Tx only connects to Usart3 or Uart4!"); };
template<> struct SignalConnection<DataC10::BitBang, Peripheral::BitBang> { static constexpr int8_t af = -1; };
template<> struct SignalConnection<DataC10::B1, Peripheral::Ltdc> { static constexpr int8_t af = 10; };
template<> struct SignalConnection<DataC10::Ck, Peripheral::I2s3> { static constexpr int8_t af = 6; };
template<> struct SignalConnection<DataC10::Ckin5, Peripheral::Dfsdm1> { static constexpr int8_t af = 3; };
template<> struct SignalConnection<DataC10::D2, Peripheral::Sdmmc1> { static constexpr int8_t af = 12; };
template<> struct SignalConnection<DataC10::D8, Peripheral::Dcmi> { static constexpr int8_t af = 13; };
template<> struct SignalConnection<DataC10::D8, Peripheral::Pssi> { static constexpr int8_t af = 13; };
template<> struct SignalConnection<DataC10::P1io1, Peripheral::Octospim> { static constexpr int8_t af = 9; };
template<> struct SignalConnection<DataC10::R2, Peripheral::Ltdc> { static constexpr int8_t af = 14; };
template<> struct SignalConnection<DataC10::Rx, Peripheral::Swpmi1> { static constexpr int8_t af = 11; };
template<> struct SignalConnection<DataC10::Sck, Peripheral::Spi3> { static constexpr int8_t af = 6; };
template<> struct SignalConnection<DataC10::Sda, Peripheral::I2c5> { static constexpr int8_t af = 4; };
template<> struct SignalConnection<DataC10::Tx, Peripheral::Usart3> { static constexpr int8_t af = 7; };
template<> struct SignalConnection<DataC10::Tx, Peripheral::Uart4> { static constexpr int8_t af = 8; };

struct DataC11 {
	static constexpr Gpio::Port port = Gpio::Port::C;
	static constexpr uint8_t pin = 11;
	struct BitBang { using Data = DataC11; static constexpr Gpio::Signal Signal = Gpio::Signal::BitBang; };
	struct B4 { using Data = DataC11; static constexpr Gpio::Signal Signal = Gpio::Signal::B4; };
	struct D3 { using Data = DataC11; static constexpr Gpio::Signal Signal = Gpio::Signal::D3; };
	struct D4 { using Data = DataC11; static constexpr Gpio::Signal Signal = Gpio::Signal::D4; };
	struct Datin5 { using Data = DataC11; static constexpr Gpio::Signal Signal = Gpio::Signal::Datin5; };
	struct Miso { using Data = DataC11; static constexpr Gpio::Signal Signal = Gpio::Signal::Miso; };
	struct P1ncs { using Data = DataC11; static constexpr Gpio::Signal Signal = Gpio::Signal::P1ncs; };
	struct Rx { using Data = DataC11; static constexpr Gpio::Signal Signal = Gpio::Signal::Rx; };
	struct Scl { using Data = DataC11; static constexpr Gpio::Signal Signal = Gpio::Signal::Scl; };
	struct Sdi { using Data = DataC11; static constexpr Gpio::Signal Signal = Gpio::Signal::Sdi; };
};
template<Peripheral p> struct SignalConnection<DataC11::BitBang, p> {
	static_assert(p == Peripheral::BitBang, "GpioC11::BitBang only connects to software drivers!"); };
template<Peripheral p> struct SignalConnection<DataC11::B4, p> {
	static_assert((p == Peripheral::Ltdc),"GpioC11::B4 only connects to Ltdc!"); };
template<Peripheral p> struct SignalConnection<DataC11::D3, p> {
	static_assert((p == Peripheral::Sdmmc1),"GpioC11::D3 only connects to Sdmmc1!"); };
template<Peripheral p> struct SignalConnection<DataC11::D4, p> {
	static_assert((p == Peripheral::Dcmi) or (p == Peripheral::Pssi),
		"GpioC11::D4 only connects to Dcmi or Pssi!"); };
template<Peripheral p> struct SignalConnection<DataC11::Datin5, p> {
	static_assert((p == Peripheral::Dfsdm1),"GpioC11::Datin5 only connects to Dfsdm1!"); };
template<Peripheral p> struct SignalConnection<DataC11::Miso, p> {
	static_assert((p == Peripheral::Spi3),"GpioC11::Miso only connects to Spi3!"); };
template<Peripheral p> struct SignalConnection<DataC11::P1ncs, p> {
	static_assert((p == Peripheral::Octospim),"GpioC11::P1ncs only connects to Octospim!"); };
template<Peripheral p> struct SignalConnection<DataC11::Rx, p> {
	static_assert((p == Peripheral::Usart3) or (p == Peripheral::Uart4),
		"GpioC11::Rx only connects to Usart3 or Uart4!"); };
template<Peripheral p> struct SignalConnection<DataC11::Scl, p> {
	static_assert((p == Peripheral::I2c5),"GpioC11::Scl only connects to I2c5!"); };
template<Peripheral p> struct SignalConnection<DataC11::Sdi, p> {
	static_assert((p == Peripheral::I2s3),"GpioC11::Sdi only connects to I2s3!"); };
template<> struct SignalConnection<DataC11::BitBang, Peripheral::BitBang> { static constexpr int8_t af = -1; };
template<> struct SignalConnection<DataC11::B4, Peripheral::Ltdc> { static constexpr int8_t af = 14; };
template<> struct SignalConnection<DataC11::D3, Peripheral::Sdmmc1> { static constexpr int8_t af = 12; };
template<> struct SignalConnection<DataC11::D4, Peripheral::Dcmi> { static constexpr int8_t af = 13; };
template<> struct SignalConnection<DataC11::D4, Peripheral::Pssi> { static constexpr int8_t af = 13; };
template<> struct SignalConnection<DataC11::Datin5, Peripheral::Dfsdm1> { static constexpr int8_t af = 3; };
template<> struct SignalConnection<DataC11::Miso, Peripheral::Spi3> { static constexpr int8_t af = 6; };
template<> struct SignalConnection<DataC11::P1ncs, Peripheral::Octospim> { static constexpr int8_t af = 9; };
template<> struct SignalConnection<DataC11::Rx, Peripheral::Usart3> { static constexpr int8_t af = 7; };
template<> struct SignalConnection<DataC11::Rx, Peripheral::Uart4> { static constexpr int8_t af = 8; };
template<> struct SignalConnection<DataC11::Scl, Peripheral::I2c5> { static constexpr int8_t af = 4; };
template<> struct SignalConnection<DataC11::Sdi, Peripheral::I2s3> { static constexpr int8_t af = 6; };

struct DataC12 {
	static constexpr Gpio::Port port = Gpio::Port::C;
	static constexpr uint8_t pin = 12;
	struct BitBang { using Data = DataC12; static constexpr Gpio::Signal Signal = Gpio::Signal::BitBang; };
	struct Ch1 { using Data = DataC12; static constexpr Gpio::Signal Signal = Gpio::Signal::Ch1; };
	struct Ck { using Data = DataC12; static constexpr Gpio::Signal Signal = Gpio::Signal::Ck; };
	struct D6 { using Data = DataC12; static constexpr Gpio::Signal Signal = Gpio::Signal::D6; };
	struct D9 { using Data = DataC12; static constexpr Gpio::Signal Signal = Gpio::Signal::D9; };
	struct Da6 { using Data = DataC12; static constexpr Gpio::Signal Signal = Gpio::Signal::Da6; };
	struct Mosi { using Data = DataC12; static constexpr Gpio::Signal Signal = Gpio::Signal::Mosi; };
	struct R6 { using Data = DataC12; static constexpr Gpio::Signal Signal = Gpio::Signal::R6; };
	struct Sck { using Data = DataC12; static constexpr Gpio::Signal Signal = Gpio::Signal::Sck; };
	struct Sdo { using Data = DataC12; static constexpr Gpio::Signal Signal = Gpio::Signal::Sdo; };
	struct Smba { using Data = DataC12; static constexpr Gpio::Signal Signal = Gpio::Signal::Smba; };
	struct Traced3 { using Data = DataC12; static constexpr Gpio::Signal Signal = Gpio::Signal::Traced3; };
	struct Tx { using Data = DataC12; static constexpr Gpio::Signal Signal = Gpio::Signal::Tx; };
};
template<Peripheral p> struct SignalConnection<DataC12::BitBang, p> {
	static_assert(p == Peripheral::BitBang, "GpioC12::BitBang only connects to software drivers!"); };
template<Peripheral p> struct SignalConnection<DataC12::Ch1, p> {
	static_assert((p == Peripheral::Tim15),"GpioC12::Ch1 only connects to Tim15!"); };
template<Peripheral p> struct SignalConnection<DataC12::Ck, p> {
	static_assert((p == Peripheral::I2s6) or (p == Peripheral::Usart3) or (p == Peripheral::Sdmmc1),
		"GpioC12::Ck only connects to I2s6 or Usart3 or Sdmmc1!"); };
template<Peripheral p> struct SignalConnection<DataC12::D6, p> {
	static_assert((p == Peripheral::Fmc),"GpioC12::D6 only connects to Fmc!"); };
template<Peripheral p> struct SignalConnection<DataC12::D9, p> {
	static_assert((p == Peripheral::Dcmi) or (p == Peripheral::Pssi),
		"GpioC12::D9 only connects to Dcmi or Pssi!"); };
template<Peripheral p> struct SignalConnection<DataC12::Da6, p> {
	static_assert((p == Peripheral::Fmc),"GpioC12::Da6 only connects to Fmc!"); };
template<Peripheral p> struct SignalConnection<DataC12::Mosi, p> {
	static_assert((p == Peripheral::Spi3),"GpioC12::Mosi only connects to Spi3!"); };
template<Peripheral p> struct SignalConnection<DataC12::R6, p> {
	static_assert((p == Peripheral::Ltdc),"GpioC12::R6 only connects to Ltdc!"); };
template<Peripheral p> struct SignalConnection<DataC12::Sck, p> {
	static_assert((p == Peripheral::Spi6),"GpioC12::Sck only connects to Spi6!"); };
template<Peripheral p> struct SignalConnection<DataC12::Sdo, p> {
	static_assert((p == Peripheral::I2s3),"GpioC12::Sdo only connects to I2s3!"); };
template<Peripheral p> struct SignalConnection<DataC12::Smba, p> {
	static_assert((p == Peripheral::I2c5),"GpioC12::Smba only connects to I2c5!"); };
template<Peripheral p> struct SignalConnection<DataC12::Traced3, p> {
	static_assert((p == Peripheral::Debug),"GpioC12::Traced3 only connects to Debug!"); };
template<Peripheral p> struct SignalConnection<DataC12::Tx, p> {
	static_assert((p == Peripheral::Uart5),"GpioC12::Tx only connects to Uart5!"); };
template<> struct SignalConnection<DataC12::BitBang, Peripheral::BitBang> { static constexpr int8_t af = -1; };
template<> struct SignalConnection<DataC12::Ch1, Peripheral::Tim15> { static constexpr int8_t af = 2; };
template<> struct SignalConnection<DataC12::Ck, Peripheral::I2s6> { static constexpr int8_t af = 5; };
template<> struct SignalConnection<DataC12::Ck, Peripheral::Usart3> { static constexpr int8_t af = 7; };
template<> struct SignalConnection<DataC12::Ck, Peripheral::Sdmmc1> { static constexpr int8_t af = 12; };
template<> struct SignalConnection<DataC12::D6, Peripheral::Fmc> { static constexpr int8_t af = 1; };
template<> struct SignalConnection<DataC12::D9, Peripheral::Dcmi> { static constexpr int8_t af = 13; };
template<> struct SignalConnection<DataC12::D9, Peripheral::Pssi> { static constexpr int8_t af = 13; };
template<> struct SignalConnection<DataC12::Da6, Peripheral::Fmc> { static constexpr int8_t af = 1; };
template<> struct SignalConnection<DataC12::Mosi, Peripheral::Spi3> { static constexpr int8_t af = 6; };
template<> struct SignalConnection<DataC12::R6, Peripheral::Ltdc> { static constexpr int8_t af = 14; };
template<> struct SignalConnection<DataC12::Sck, Peripheral::Spi6> { static constexpr int8_t af = 5; };
template<> struct SignalConnection<DataC12::Sdo, Peripheral::I2s3> { static constexpr int8_t af = 6; };
template<> struct SignalConnection<DataC12::Smba, Peripheral::I2c5> { static constexpr int8_t af = 4; };
template<> struct SignalConnection<DataC12::Traced3, Peripheral::Debug> { static constexpr int8_t af = 0; };
template<> struct SignalConnection<DataC12::Tx, Peripheral::Uart5> { static constexpr int8_t af = 8; };

struct DataC13 {
	static constexpr Gpio::Port port = Gpio::Port::C;
	static constexpr uint8_t pin = 13;
	struct BitBang { using Data = DataC13; static constexpr Gpio::Signal Signal = Gpio::Signal::BitBang; };
	struct Outalarm { using Data = DataC13; static constexpr Gpio::Signal Signal = Gpio::Signal::Outalarm; };
	struct Outcalib { using Data = DataC13; static constexpr Gpio::Signal Signal = Gpio::Signal::Outcalib; };
	struct Tamp1 { using Data = DataC13; static constexpr Gpio::Signal Signal = Gpio::Signal::Tamp1; };
	struct Ts { using Data = DataC13; static constexpr Gpio::Signal Signal = Gpio::Signal::Ts; };
	struct Wkup4 { using Data = DataC13; static constexpr Gpio::Signal Signal = Gpio::Signal::Wkup4; };
};
template<Peripheral p> struct SignalConnection<DataC13::BitBang, p> {
	static_assert(p == Peripheral::BitBang, "GpioC13::BitBang only connects to software drivers!"); };
template<Peripheral p> struct SignalConnection<DataC13::Outalarm, p> {
	static_assert((p == Peripheral::Rtc),"GpioC13::Outalarm only connects to Rtc!"); };
template<Peripheral p> struct SignalConnection<DataC13::Outcalib, p> {
	static_assert((p == Peripheral::Rtc),"GpioC13::Outcalib only connects to Rtc!"); };
template<Peripheral p> struct SignalConnection<DataC13::Tamp1, p> {
	static_assert((p == Peripheral::Rtc),"GpioC13::Tamp1 only connects to Rtc!"); };
template<Peripheral p> struct SignalConnection<DataC13::Ts, p> {
	static_assert((p == Peripheral::Rtc),"GpioC13::Ts only connects to Rtc!"); };
template<Peripheral p> struct SignalConnection<DataC13::Wkup4, p> {
	static_assert((p == Peripheral::Pwr),"GpioC13::Wkup4 only connects to Pwr!"); };
template<> struct SignalConnection<DataC13::BitBang, Peripheral::BitBang> { static constexpr int8_t af = -1; };
template<> struct SignalConnection<DataC13::Outalarm, Peripheral::Rtc> { static constexpr int8_t af = -1; };
template<> struct SignalConnection<DataC13::Outcalib, Peripheral::Rtc> { static constexpr int8_t af = -1; };
template<> struct SignalConnection<DataC13::Tamp1, Peripheral::Rtc> { static constexpr int8_t af = -1; };
template<> struct SignalConnection<DataC13::Ts, Peripheral::Rtc> { static constexpr int8_t af = -1; };
template<> struct SignalConnection<DataC13::Wkup4, Peripheral::Pwr> { static constexpr int8_t af = -1; };

struct DataC14 {
	static constexpr Gpio::Port port = Gpio::Port::C;
	static constexpr uint8_t pin = 14;
	struct BitBang { using Data = DataC14; static constexpr Gpio::Signal Signal = Gpio::Signal::BitBang; };
	struct Osc32in { using Data = DataC14; static constexpr Gpio::Signal Signal = Gpio::Signal::Osc32in; };
};
template<Peripheral p> struct SignalConnection<DataC14::BitBang, p> {
	static_assert(p == Peripheral::BitBang, "GpioC14::BitBang only connects to software drivers!"); };
template<Peripheral p> struct SignalConnection<DataC14::Osc32in, p> {
	static_assert((p == Peripheral::Rcc),"GpioC14::Osc32in only connects to Rcc!"); };
template<> struct SignalConnection<DataC14::BitBang, Peripheral::BitBang> { static constexpr int8_t af = -1; };
template<> struct SignalConnection<DataC14::Osc32in, Peripheral::Rcc> { static constexpr int8_t af = -1; };

struct DataC15 {
	static constexpr Gpio::Port port = Gpio::Port::C;
	static constexpr uint8_t pin = 15;
	struct BitBang { using Data = DataC15; static constexpr Gpio::Signal Signal = Gpio::Signal::BitBang; };
	struct Osc32out { using Data = DataC15; static constexpr Gpio::Signal Signal = Gpio::Signal::Osc32out; };
};
template<Peripheral p> struct SignalConnection<DataC15::BitBang, p> {
	static_assert(p == Peripheral::BitBang, "GpioC15::BitBang only connects to software drivers!"); };
template<Peripheral p> struct SignalConnection<DataC15::Osc32out, p> {
	static_assert((p == Peripheral::Rcc),"GpioC15::Osc32out only connects to Rcc!"); };
template<> struct SignalConnection<DataC15::BitBang, Peripheral::BitBang> { static constexpr int8_t af = -1; };
template<> struct SignalConnection<DataC15::Osc32out, Peripheral::Rcc> { static constexpr int8_t af = -1; };

struct DataD0 {
	static constexpr Gpio::Port port = Gpio::Port::D;
	static constexpr uint8_t pin = 0;
	struct BitBang { using Data = DataD0; static constexpr Gpio::Signal Signal = Gpio::Signal::BitBang; };
	struct B1 { using Data = DataD0; static constexpr Gpio::Signal Signal = Gpio::Signal::B1; };
	struct Ckin6 { using Data = DataD0; static constexpr Gpio::Signal Signal = Gpio::Signal::Ckin6; };
	struct Cts { using Data = DataD0; static constexpr Gpio::Signal Signal = Gpio::Signal::Cts; };
	struct D2 { using Data = DataD0; static constexpr Gpio::Signal Signal = Gpio::Signal::D2; };
	struct Da2 { using Data = DataD0; static constexpr Gpio::Signal Signal = Gpio::Signal::Da2; };
	struct Rx { using Data = DataD0; static constexpr Gpio::Signal Signal = Gpio::Signal::Rx; };
};
template<Peripheral p> struct SignalConnection<DataD0::BitBang, p> {
	static_assert(p == Peripheral::BitBang, "GpioD0::BitBang only connects to software drivers!"); };
template<Peripheral p> struct SignalConnection<DataD0::B1, p> {
	static_assert((p == Peripheral::Ltdc),"GpioD0::B1 only connects to Ltdc!"); };
template<Peripheral p> struct SignalConnection<DataD0::Ckin6, p> {
	static_assert((p == Peripheral::Dfsdm1),"GpioD0::Ckin6 only connects to Dfsdm1!"); };
template<Peripheral p> struct SignalConnection<DataD0::Cts, p> {
	static_assert((p == Peripheral::Uart9),"GpioD0::Cts only connects to Uart9!"); };
template<Peripheral p> struct SignalConnection<DataD0::D2, p> {
	static_assert((p == Peripheral::Fmc),"GpioD0::D2 only connects to Fmc!"); };
template<Peripheral p> struct SignalConnection<DataD0::Da2, p> {
	static_assert((p == Peripheral::Fmc),"GpioD0::Da2 only connects to Fmc!"); };
template<Peripheral p> struct SignalConnection<DataD0::Rx, p> {
	static_assert((p == Peripheral::Uart4) or (p == Peripheral::Fdcan1),
		"GpioD0::Rx only connects to Uart4 or Fdcan1!"); };
template<> struct SignalConnection<DataD0::BitBang, Peripheral::BitBang> { static constexpr int8_t af = -1; };
template<> struct SignalConnection<DataD0::B1, Peripheral::Ltdc> { static constexpr int8_t af = 14; };
template<> struct SignalConnection<DataD0::Ckin6, Peripheral::Dfsdm1> { static constexpr int8_t af = 3; };
template<> struct SignalConnection<DataD0::Cts, Peripheral::Uart9> { static constexpr int8_t af = 11; };
template<> struct SignalConnection<DataD0::D2, Peripheral::Fmc> { static constexpr int8_t af = 12; };
template<> struct SignalConnection<DataD0::Da2, Peripheral::Fmc> { static constexpr int8_t af = 12; };
template<> struct SignalConnection<DataD0::Rx, Peripheral::Uart4> { static constexpr int8_t af = 8; };
template<> struct SignalConnection<DataD0::Rx, Peripheral::Fdcan1> { static constexpr int8_t af = 9; };

struct DataD1 {
	static constexpr Gpio::Port port = Gpio::Port::D;
	static constexpr uint8_t pin = 1;
	struct BitBang { using Data = DataD1; static constexpr Gpio::Signal Signal = Gpio::Signal::BitBang; };
	struct D3 { using Data = DataD1; static constexpr Gpio::Signal Signal = Gpio::Signal::D3; };
	struct Da3 { using Data = DataD1; static constexpr Gpio::Signal Signal = Gpio::Signal::Da3; };
	struct Datin6 { using Data = DataD1; static constexpr Gpio::Signal Signal = Gpio::Signal::Datin6; };
	struct Tx { using Data = DataD1; static constexpr Gpio::Signal Signal = Gpio::Signal::Tx; };
};
template<Peripheral p> struct SignalConnection<DataD1::BitBang, p> {
	static_assert(p == Peripheral::BitBang, "GpioD1::BitBang only connects to software drivers!"); };
template<Peripheral p> struct SignalConnection<DataD1::D3, p> {
	static_assert((p == Peripheral::Fmc),"GpioD1::D3 only connects to Fmc!"); };
template<Peripheral p> struct SignalConnection<DataD1::Da3, p> {
	static_assert((p == Peripheral::Fmc),"GpioD1::Da3 only connects to Fmc!"); };
template<Peripheral p> struct SignalConnection<DataD1::Datin6, p> {
	static_assert((p == Peripheral::Dfsdm1),"GpioD1::Datin6 only connects to Dfsdm1!"); };
template<Peripheral p> struct SignalConnection<DataD1::Tx, p> {
	static_assert((p == Peripheral::Uart4) or (p == Peripheral::Fdcan1),
		"GpioD1::Tx only connects to Uart4 or Fdcan1!"); };
template<> struct SignalConnection<DataD1::BitBang, Peripheral::BitBang> { static constexpr int8_t af = -1; };
template<> struct SignalConnection<DataD1::D3, Peripheral::Fmc> { static constexpr int8_t af = 12; };
template<> struct SignalConnection<DataD1::Da3, Peripheral::Fmc> { static constexpr int8_t af = 12; };
template<> struct SignalConnection<DataD1::Datin6, Peripheral::Dfsdm1> { static constexpr int8_t af = 3; };
template<> struct SignalConnection<DataD1::Tx, Peripheral::Uart4> { static constexpr int8_t af = 8; };
template<> struct SignalConnection<DataD1::Tx, Peripheral::Fdcan1> { static constexpr int8_t af = 9; };

struct DataD2 {
	static constexpr Gpio::Port port = Gpio::Port::D;
	static constexpr uint8_t pin = 2;
	struct BitBang { using Data = DataD2; static constexpr Gpio::Signal Signal = Gpio::Signal::BitBang; };
	struct B2 { using Data = DataD2; static constexpr Gpio::Signal Signal = Gpio::Signal::B2; };
	struct B7 { using Data = DataD2; static constexpr Gpio::Signal Signal = Gpio::Signal::B7; };
	struct Bkin { using Data = DataD2; static constexpr Gpio::Signal Signal = Gpio::Signal::Bkin; };
	struct Cmd { using Data = DataD2; static constexpr Gpio::Signal Signal = Gpio::Signal::Cmd; };
	struct D11 { using Data = DataD2; static constexpr Gpio::Signal Signal = Gpio::Signal::D11; };
	struct D7 { using Data = DataD2; static constexpr Gpio::Signal Signal = Gpio::Signal::D7; };
	struct Da7 { using Data = DataD2; static constexpr Gpio::Signal Signal = Gpio::Signal::Da7; };
	struct Etr { using Data = DataD2; static constexpr Gpio::Signal Signal = Gpio::Signal::Etr; };
	struct Rx { using Data = DataD2; static constexpr Gpio::Signal Signal = Gpio::Signal::Rx; };
	struct Traced2 { using Data = DataD2; static constexpr Gpio::Signal Signal = Gpio::Signal::Traced2; };
};
template<Peripheral p> struct SignalConnection<DataD2::BitBang, p> {
	static_assert(p == Peripheral::BitBang, "GpioD2::BitBang only connects to software drivers!"); };
template<Peripheral p> struct SignalConnection<DataD2::B2, p> {
	static_assert((p == Peripheral::Ltdc),"GpioD2::B2 only connects to Ltdc!"); };
template<Peripheral p> struct SignalConnection<DataD2::B7, p> {
	static_assert((p == Peripheral::Ltdc),"GpioD2::B7 only connects to Ltdc!"); };
template<Peripheral p> struct SignalConnection<DataD2::Bkin, p> {
	static_assert((p == Peripheral::Tim15),"GpioD2::Bkin only connects to Tim15!"); };
template<Peripheral p> struct SignalConnection<DataD2::Cmd, p> {
	static_assert((p == Peripheral::Sdmmc1),"GpioD2::Cmd only connects to Sdmmc1!"); };
template<Peripheral p> struct SignalConnection<DataD2::D11, p> {
	static_assert((p == Peripheral::Dcmi) or (p == Peripheral::Pssi),
		"GpioD2::D11 only connects to Dcmi or Pssi!"); };
template<Peripheral p> struct SignalConnection<DataD2::D7, p> {
	static_assert((p == Peripheral::Fmc),"GpioD2::D7 only connects to Fmc!"); };
template<Peripheral p> struct SignalConnection<DataD2::Da7, p> {
	static_assert((p == Peripheral::Fmc),"GpioD2::Da7 only connects to Fmc!"); };
template<Peripheral p> struct SignalConnection<DataD2::Etr, p> {
	static_assert((p == Peripheral::Tim3),"GpioD2::Etr only connects to Tim3!"); };
template<Peripheral p> struct SignalConnection<DataD2::Rx, p> {
	static_assert((p == Peripheral::Uart5),"GpioD2::Rx only connects to Uart5!"); };
template<Peripheral p> struct SignalConnection<DataD2::Traced2, p> {
	static_assert((p == Peripheral::Debug),"GpioD2::Traced2 only connects to Debug!"); };
template<> struct SignalConnection<DataD2::BitBang, Peripheral::BitBang> { static constexpr int8_t af = -1; };
template<> struct SignalConnection<DataD2::B2, Peripheral::Ltdc> { static constexpr int8_t af = 14; };
template<> struct SignalConnection<DataD2::B7, Peripheral::Ltdc> { static constexpr int8_t af = 9; };
template<> struct SignalConnection<DataD2::Bkin, Peripheral::Tim15> { static constexpr int8_t af = 4; };
template<> struct SignalConnection<DataD2::Cmd, Peripheral::Sdmmc1> { static constexpr int8_t af = 12; };
template<> struct SignalConnection<DataD2::D11, Peripheral::Dcmi> { static constexpr int8_t af = 13; };
template<> struct SignalConnection<DataD2::D11, Peripheral::Pssi> { static constexpr int8_t af = 13; };
template<> struct SignalConnection<DataD2::D7, Peripheral::Fmc> { static constexpr int8_t af = 1; };
template<> struct SignalConnection<DataD2::Da7, Peripheral::Fmc> { static constexpr int8_t af = 1; };
template<> struct SignalConnection<DataD2::Etr, Peripheral::Tim3> { static constexpr int8_t af = 2; };
template<> struct SignalConnection<DataD2::Rx, Peripheral::Uart5> { static constexpr int8_t af = 8; };
template<> struct SignalConnection<DataD2::Traced2, Peripheral::Debug> { static constexpr int8_t af = 0; };

struct DataD3 {
	static constexpr Gpio::Port port = Gpio::Port::D;
	static constexpr uint8_t pin = 3;
	struct BitBang { using Data = DataD3; static constexpr Gpio::Signal Signal = Gpio::Signal::BitBang; };
	struct Ck { using Data = DataD3; static constexpr Gpio::Signal Signal = Gpio::Signal::Ck; };
	struct Ckout { using Data = DataD3; static constexpr Gpio::Signal Signal = Gpio::Signal::Ckout; };
	struct Clk { using Data = DataD3; static constexpr Gpio::Signal Signal = Gpio::Signal::Clk; };
	struct Cts { using Data = DataD3; static constexpr Gpio::Signal Signal = Gpio::Signal::Cts; };
	struct D5 { using Data = DataD3; static constexpr Gpio::Signal Signal = Gpio::Signal::D5; };
	struct G7 { using Data = DataD3; static constexpr Gpio::Signal Signal = Gpio::Signal::G7; };
	struct Nss { using Data = DataD3; static constexpr Gpio::Signal Signal = Gpio::Signal::Nss; };
	struct Sck { using Data = DataD3; static constexpr Gpio::Signal Signal = Gpio::Signal::Sck; };
};
template<Peripheral p> struct SignalConnection<DataD3::BitBang, p> {
	static_assert(p == Peripheral::BitBang, "GpioD3::BitBang only connects to software drivers!"); };
template<Peripheral p> struct SignalConnection<DataD3::Ck, p> {
	static_assert((p == Peripheral::I2s2),"GpioD3::Ck only connects to I2s2!"); };
template<Peripheral p> struct SignalConnection<DataD3::Ckout, p> {
	static_assert((p == Peripheral::Dfsdm1),"GpioD3::Ckout only connects to Dfsdm1!"); };
template<Peripheral p> struct SignalConnection<DataD3::Clk, p> {
	static_assert((p == Peripheral::Fmc),"GpioD3::Clk only connects to Fmc!"); };
template<Peripheral p> struct SignalConnection<DataD3::Cts, p> {
	static_assert((p == Peripheral::Usart2),"GpioD3::Cts only connects to Usart2!"); };
template<Peripheral p> struct SignalConnection<DataD3::D5, p> {
	static_assert((p == Peripheral::Dcmi) or (p == Peripheral::Pssi),
		"GpioD3::D5 only connects to Dcmi or Pssi!"); };
template<Peripheral p> struct SignalConnection<DataD3::G7, p> {
	static_assert((p == Peripheral::Ltdc),"GpioD3::G7 only connects to Ltdc!"); };
template<Peripheral p> struct SignalConnection<DataD3::Nss, p> {
	static_assert((p == Peripheral::Usart2),"GpioD3::Nss only connects to Usart2!"); };
template<Peripheral p> struct SignalConnection<DataD3::Sck, p> {
	static_assert((p == Peripheral::Spi2),"GpioD3::Sck only connects to Spi2!"); };
template<> struct SignalConnection<DataD3::BitBang, Peripheral::BitBang> { static constexpr int8_t af = -1; };
template<> struct SignalConnection<DataD3::Ck, Peripheral::I2s2> { static constexpr int8_t af = 5; };
template<> struct SignalConnection<DataD3::Ckout, Peripheral::Dfsdm1> { static constexpr int8_t af = 3; };
template<> struct SignalConnection<DataD3::Clk, Peripheral::Fmc> { static constexpr int8_t af = 12; };
template<> struct SignalConnection<DataD3::Cts, Peripheral::Usart2> { static constexpr int8_t af = 7; };
template<> struct SignalConnection<DataD3::D5, Peripheral::Dcmi> { static constexpr int8_t af = 13; };
template<> struct SignalConnection<DataD3::D5, Peripheral::Pssi> { static constexpr int8_t af = 13; };
template<> struct SignalConnection<DataD3::G7, Peripheral::Ltdc> { static constexpr int8_t af = 14; };
template<> struct SignalConnection<DataD3::Nss, Peripheral::Usart2> { static constexpr int8_t af = 7; };
template<> struct SignalConnection<DataD3::Sck, Peripheral::Spi2> { static constexpr int8_t af = 5; };

struct DataD4 {
	static constexpr Gpio::Port port = Gpio::Port::D;
	static constexpr uint8_t pin = 4;
	struct BitBang { using Data = DataD4; static constexpr Gpio::Signal Signal = Gpio::Signal::BitBang; };
	struct De { using Data = DataD4; static constexpr Gpio::Signal Signal = Gpio::Signal::De; };
	struct Noe { using Data = DataD4; static constexpr Gpio::Signal Signal = Gpio::Signal::Noe; };
	struct P1io4 { using Data = DataD4; static constexpr Gpio::Signal Signal = Gpio::Signal::P1io4; };
	struct Rts { using Data = DataD4; static constexpr Gpio::Signal Signal = Gpio::Signal::Rts; };
};
template<Peripheral p> struct SignalConnection<DataD4::BitBang, p> {
	static_assert(p == Peripheral::BitBang, "GpioD4::BitBang only connects to software drivers!"); };
template<Peripheral p> struct SignalConnection<DataD4::De, p> {
	static_assert((p == Peripheral::Usart2),"GpioD4::De only connects to Usart2!"); };
template<Peripheral p> struct SignalConnection<DataD4::Noe, p> {
	static_assert((p == Peripheral::Fmc),"GpioD4::Noe only connects to Fmc!"); };
template<Peripheral p> struct SignalConnection<DataD4::P1io4, p> {
	static_assert((p == Peripheral::Octospim),"GpioD4::P1io4 only connects to Octospim!"); };
template<Peripheral p> struct SignalConnection<DataD4::Rts, p> {
	static_assert((p == Peripheral::Usart2),"GpioD4::Rts only connects to Usart2!"); };
template<> struct SignalConnection<DataD4::BitBang, Peripheral::BitBang> { static constexpr int8_t af = -1; };
template<> struct SignalConnection<DataD4::De, Peripheral::Usart2> { static constexpr int8_t af = 7; };
template<> struct SignalConnection<DataD4::Noe, Peripheral::Fmc> { static constexpr int8_t af = 12; };
template<> struct SignalConnection<DataD4::P1io4, Peripheral::Octospim> { static constexpr int8_t af = 10; };
template<> struct SignalConnection<DataD4::Rts, Peripheral::Usart2> { static constexpr int8_t af = 7; };

struct DataD5 {
	static constexpr Gpio::Port port = Gpio::Port::D;
	static constexpr uint8_t pin = 5;
	struct BitBang { using Data = DataD5; static constexpr Gpio::Signal Signal = Gpio::Signal::BitBang; };
	struct Nwe { using Data = DataD5; static constexpr Gpio::Signal Signal = Gpio::Signal::Nwe; };
	struct P1io5 { using Data = DataD5; static constexpr Gpio::Signal Signal = Gpio::Signal::P1io5; };
	struct Tx { using Data = DataD5; static constexpr Gpio::Signal Signal = Gpio::Signal::Tx; };
};
template<Peripheral p> struct SignalConnection<DataD5::BitBang, p> {
	static_assert(p == Peripheral::BitBang, "GpioD5::BitBang only connects to software drivers!"); };
template<Peripheral p> struct SignalConnection<DataD5::Nwe, p> {
	static_assert((p == Peripheral::Fmc),"GpioD5::Nwe only connects to Fmc!"); };
template<Peripheral p> struct SignalConnection<DataD5::P1io5, p> {
	static_assert((p == Peripheral::Octospim),"GpioD5::P1io5 only connects to Octospim!"); };
template<Peripheral p> struct SignalConnection<DataD5::Tx, p> {
	static_assert((p == Peripheral::Usart2),"GpioD5::Tx only connects to Usart2!"); };
template<> struct SignalConnection<DataD5::BitBang, Peripheral::BitBang> { static constexpr int8_t af = -1; };
template<> struct SignalConnection<DataD5::Nwe, Peripheral::Fmc> { static constexpr int8_t af = 12; };
template<> struct SignalConnection<DataD5::P1io5, Peripheral::Octospim> { static constexpr int8_t af = 10; };
template<> struct SignalConnection<DataD5::Tx, Peripheral::Usart2> { static constexpr int8_t af = 7; };

struct DataD6 {
	static constexpr Gpio::Port port = Gpio::Port::D;
	static constexpr uint8_t pin = 6;
	struct BitBang { using Data = DataD6; static constexpr Gpio::Signal Signal = Gpio::Signal::BitBang; };
	struct B2 { using Data = DataD6; static constexpr Gpio::Signal Signal = Gpio::Signal::B2; };
	struct Ck { using Data = DataD6; static constexpr Gpio::Signal Signal = Gpio::Signal::Ck; };
	struct Ckin4 { using Data = DataD6; static constexpr Gpio::Signal Signal = Gpio::Signal::Ckin4; };
	struct D1 { using Data = DataD6; static constexpr Gpio::Signal Signal = Gpio::Signal::D1; };
	struct D10 { using Data = DataD6; static constexpr Gpio::Signal Signal = Gpio::Signal::D10; };
	struct Datin1 { using Data = DataD6; static constexpr Gpio::Signal Signal = Gpio::Signal::Datin1; };
	struct Mosi { using Data = DataD6; static constexpr Gpio::Signal Signal = Gpio::Signal::Mosi; };
	struct Nwait { using Data = DataD6; static constexpr Gpio::Signal Signal = Gpio::Signal::Nwait; };
	struct P1io6 { using Data = DataD6; static constexpr Gpio::Signal Signal = Gpio::Signal::P1io6; };
	struct Rx { using Data = DataD6; static constexpr Gpio::Signal Signal = Gpio::Signal::Rx; };
	struct Sda { using Data = DataD6; static constexpr Gpio::Signal Signal = Gpio::Signal::Sda; };
	struct Sdo { using Data = DataD6; static constexpr Gpio::Signal Signal = Gpio::Signal::Sdo; };
};
template<Peripheral p> struct SignalConnection<DataD6::BitBang, p> {
	static_assert(p == Peripheral::BitBang, "GpioD6::BitBang only connects to software drivers!"); };
template<Peripheral p> struct SignalConnection<DataD6::B2, p> {
	static_assert((p == Peripheral::Ltdc),"GpioD6::B2 only connects to Ltdc!"); };
template<Peripheral p> struct SignalConnection<DataD6::Ck, p> {
	static_assert((p == Peripheral::Sdmmc2),"GpioD6::Ck only connects to Sdmmc2!"); };
template<Peripheral p> struct SignalConnection<DataD6::Ckin4, p> {
	static_assert((p == Peripheral::Dfsdm1),"GpioD6::Ckin4 only connects to Dfsdm1!"); };
template<Peripheral p> struct SignalConnection<DataD6::D1, p> {
	static_assert((p == Peripheral::Sai4) or (p == Peripheral::Sai1),
		"GpioD6::D1 only connects to Sai4 or Sai1!"); };
template<Peripheral p> struct SignalConnection<DataD6::D10, p> {
	static_assert((p == Peripheral::Dcmi) or (p == Peripheral::Pssi),
		"GpioD6::D10 only connects to Dcmi or Pssi!"); };
template<Peripheral p> struct SignalConnection<DataD6::Datin1, p> {
	static_assert((p == Peripheral::Dfsdm1),"GpioD6::Datin1 only connects to Dfsdm1!"); };
template<Peripheral p> struct SignalConnection<DataD6::Mosi, p> {
	static_assert((p == Peripheral::Spi3),"GpioD6::Mosi only connects to Spi3!"); };
template<Peripheral p> struct SignalConnection<DataD6::Nwait, p> {
	static_assert((p == Peripheral::Fmc),"GpioD6::Nwait only connects to Fmc!"); };
template<Peripheral p> struct SignalConnection<DataD6::P1io6, p> {
	static_assert((p == Peripheral::Octospim),"GpioD6::P1io6 only connects to Octospim!"); };
template<Peripheral p> struct SignalConnection<DataD6::Rx, p> {
	static_assert((p == Peripheral::Usart2),"GpioD6::Rx only connects to Usart2!"); };
template<Peripheral p> struct SignalConnection<DataD6::Sda, p> {
	static_assert((p == Peripheral::Sai1) or (p == Peripheral::Sai4),
		"GpioD6::Sda only connects to Sai1 or Sai4!"); };
template<Peripheral p> struct SignalConnection<DataD6::Sdo, p> {
	static_assert((p == Peripheral::I2s3),"GpioD6::Sdo only connects to I2s3!"); };
template<> struct SignalConnection<DataD6::BitBang, Peripheral::BitBang> { static constexpr int8_t af = -1; };
template<> struct SignalConnection<DataD6::B2, Peripheral::Ltdc> { static constexpr int8_t af = 14; };
template<> struct SignalConnection<DataD6::Ck, Peripheral::Sdmmc2> { static constexpr int8_t af = 11; };
template<> struct SignalConnection<DataD6::Ckin4, Peripheral::Dfsdm1> { static constexpr int8_t af = 3; };
template<> struct SignalConnection<DataD6::D1, Peripheral::Sai4> { static constexpr int8_t af = 1; };
template<> struct SignalConnection<DataD6::D1, Peripheral::Sai1> { static constexpr int8_t af = 2; };
template<> struct SignalConnection<DataD6::D10, Peripheral::Dcmi> { static constexpr int8_t af = 13; };
template<> struct SignalConnection<DataD6::D10, Peripheral::Pssi> { static constexpr int8_t af = 13; };
template<> struct SignalConnection<DataD6::Datin1, Peripheral::Dfsdm1> { static constexpr int8_t af = 4; };
template<> struct SignalConnection<DataD6::Mosi, Peripheral::Spi3> { static constexpr int8_t af = 5; };
template<> struct SignalConnection<DataD6::Nwait, Peripheral::Fmc> { static constexpr int8_t af = 12; };
template<> struct SignalConnection<DataD6::P1io6, Peripheral::Octospim> { static constexpr int8_t af = 10; };
template<> struct SignalConnection<DataD6::Rx, Peripheral::Usart2> { static constexpr int8_t af = 7; };
template<> struct SignalConnection<DataD6::Sda, Peripheral::Sai1> { static constexpr int8_t af = 6; };
template<> struct SignalConnection<DataD6::Sda, Peripheral::Sai4> { static constexpr int8_t af = 8; };
template<> struct SignalConnection<DataD6::Sdo, Peripheral::I2s3> { static constexpr int8_t af = 5; };

struct DataD7 {
	static constexpr Gpio::Port port = Gpio::Port::D;
	static constexpr uint8_t pin = 7;
	struct BitBang { using Data = DataD7; static constexpr Gpio::Signal Signal = Gpio::Signal::BitBang; };
	struct Ck { using Data = DataD7; static constexpr Gpio::Signal Signal = Gpio::Signal::Ck; };
	struct Ckin1 { using Data = DataD7; static constexpr Gpio::Signal Signal = Gpio::Signal::Ckin1; };
	struct Cmd { using Data = DataD7; static constexpr Gpio::Signal Signal = Gpio::Signal::Cmd; };
	struct Datin4 { using Data = DataD7; static constexpr Gpio::Signal Signal = Gpio::Signal::Datin4; };
	struct In0 { using Data = DataD7; static constexpr Gpio::Signal Signal = Gpio::Signal::In0; };
	struct Mosi { using Data = DataD7; static constexpr Gpio::Signal Signal = Gpio::Signal::Mosi; };
	struct Ne1 { using Data = DataD7; static constexpr Gpio::Signal Signal = Gpio::Signal::Ne1; };
	struct P1io7 { using Data = DataD7; static constexpr Gpio::Signal Signal = Gpio::Signal::P1io7; };
	struct Sdo { using Data = DataD7; static constexpr Gpio::Signal Signal = Gpio::Signal::Sdo; };
};
template<Peripheral p> struct SignalConnection<DataD7::BitBang, p> {
	static_assert(p == Peripheral::BitBang, "GpioD7::BitBang only connects to software drivers!"); };
template<Peripheral p> struct SignalConnection<DataD7::Ck, p> {
	static_assert((p == Peripheral::Usart2),"GpioD7::Ck only connects to Usart2!"); };
template<Peripheral p> struct SignalConnection<DataD7::Ckin1, p> {
	static_assert((p == Peripheral::Dfsdm1),"GpioD7::Ckin1 only connects to Dfsdm1!"); };
template<Peripheral p> struct SignalConnection<DataD7::Cmd, p> {
	static_assert((p == Peripheral::Sdmmc2),"GpioD7::Cmd only connects to Sdmmc2!"); };
template<Peripheral p> struct SignalConnection<DataD7::Datin4, p> {
	static_assert((p == Peripheral::Dfsdm1),"GpioD7::Datin4 only connects to Dfsdm1!"); };
template<Peripheral p> struct SignalConnection<DataD7::In0, p> {
	static_assert((p == Peripheral::Spdifrx),"GpioD7::In0 only connects to Spdifrx!"); };
template<Peripheral p> struct SignalConnection<DataD7::Mosi, p> {
	static_assert((p == Peripheral::Spi1),"GpioD7::Mosi only connects to Spi1!"); };
template<Peripheral p> struct SignalConnection<DataD7::Ne1, p> {
	static_assert((p == Peripheral::Fmc),"GpioD7::Ne1 only connects to Fmc!"); };
template<Peripheral p> struct SignalConnection<DataD7::P1io7, p> {
	static_assert((p == Peripheral::Octospim),"GpioD7::P1io7 only connects to Octospim!"); };
template<Peripheral p> struct SignalConnection<DataD7::Sdo, p> {
	static_assert((p == Peripheral::I2s1),"GpioD7::Sdo only connects to I2s1!"); };
template<> struct SignalConnection<DataD7::BitBang, Peripheral::BitBang> { static constexpr int8_t af = -1; };
template<> struct SignalConnection<DataD7::Ck, Peripheral::Usart2> { static constexpr int8_t af = 7; };
template<> struct SignalConnection<DataD7::Ckin1, Peripheral::Dfsdm1> { static constexpr int8_t af = 6; };
template<> struct SignalConnection<DataD7::Cmd, Peripheral::Sdmmc2> { static constexpr int8_t af = 11; };
template<> struct SignalConnection<DataD7::Datin4, Peripheral::Dfsdm1> { static constexpr int8_t af = 3; };
template<> struct SignalConnection<DataD7::In0, Peripheral::Spdifrx> { static constexpr int8_t af = 9; };
template<> struct SignalConnection<DataD7::Mosi, Peripheral::Spi1> { static constexpr int8_t af = 5; };
template<> struct SignalConnection<DataD7::Ne1, Peripheral::Fmc> { static constexpr int8_t af = 12; };
template<> struct SignalConnection<DataD7::P1io7, Peripheral::Octospim> { static constexpr int8_t af = 10; };
template<> struct SignalConnection<DataD7::Sdo, Peripheral::I2s1> { static constexpr int8_t af = 5; };

struct DataD8 {
	static constexpr Gpio::Port port = Gpio::Port::D;
	static constexpr uint8_t pin = 8;
	struct BitBang { using Data = DataD8; static constexpr Gpio::Signal Signal = Gpio::Signal::BitBang; };
	struct Ckin3 { using Data = DataD8; static constexpr Gpio::Signal Signal = Gpio::Signal::Ckin3; };
	struct D13 { using Data = DataD8; static constexpr Gpio::Signal Signal = Gpio::Signal::D13; };
	struct Da13 { using Data = DataD8; static constexpr Gpio::Signal Signal = Gpio::Signal::Da13; };
	struct In1 { using Data = DataD8; static constexpr Gpio::Signal Signal = Gpio::Signal::In1; };
	struct Tx { using Data = DataD8; static constexpr Gpio::Signal Signal = Gpio::Signal::Tx; };
};
template<Peripheral p> struct SignalConnection<DataD8::BitBang, p> {
	static_assert(p == Peripheral::BitBang, "GpioD8::BitBang only connects to software drivers!"); };
template<Peripheral p> struct SignalConnection<DataD8::Ckin3, p> {
	static_assert((p == Peripheral::Dfsdm1),"GpioD8::Ckin3 only connects to Dfsdm1!"); };
template<Peripheral p> struct SignalConnection<DataD8::D13, p> {
	static_assert((p == Peripheral::Fmc),"GpioD8::D13 only connects to Fmc!"); };
template<Peripheral p> struct SignalConnection<DataD8::Da13, p> {
	static_assert((p == Peripheral::Fmc),"GpioD8::Da13 only connects to Fmc!"); };
template<Peripheral p> struct SignalConnection<DataD8::In1, p> {
	static_assert((p == Peripheral::Spdifrx),"GpioD8::In1 only connects to Spdifrx!"); };
template<Peripheral p> struct SignalConnection<DataD8::Tx, p> {
	static_assert((p == Peripheral::Usart3),"GpioD8::Tx only connects to Usart3!"); };
template<> struct SignalConnection<DataD8::BitBang, Peripheral::BitBang> { static constexpr int8_t af = -1; };
template<> struct SignalConnection<DataD8::Ckin3, Peripheral::Dfsdm1> { static constexpr int8_t af = 3; };
template<> struct SignalConnection<DataD8::D13, Peripheral::Fmc> { static constexpr int8_t af = 12; };
template<> struct SignalConnection<DataD8::Da13, Peripheral::Fmc> { static constexpr int8_t af = 12; };
template<> struct SignalConnection<DataD8::In1, Peripheral::Spdifrx> { static constexpr int8_t af = 9; };
template<> struct SignalConnection<DataD8::Tx, Peripheral::Usart3> { static constexpr int8_t af = 7; };

struct DataD9 {
	static constexpr Gpio::Port port = Gpio::Port::D;
	static constexpr uint8_t pin = 9;
	struct BitBang { using Data = DataD9; static constexpr Gpio::Signal Signal = Gpio::Signal::BitBang; };
	struct D14 { using Data = DataD9; static constexpr Gpio::Signal Signal = Gpio::Signal::D14; };
	struct Da14 { using Data = DataD9; static constexpr Gpio::Signal Signal = Gpio::Signal::Da14; };
	struct Datin3 { using Data = DataD9; static constexpr Gpio::Signal Signal = Gpio::Signal::Datin3; };
	struct Rx { using Data = DataD9; static constexpr Gpio::Signal Signal = Gpio::Signal::Rx; };
};
template<Peripheral p> struct SignalConnection<DataD9::BitBang, p> {
	static_assert(p == Peripheral::BitBang, "GpioD9::BitBang only connects to software drivers!"); };
template<Peripheral p> struct SignalConnection<DataD9::D14, p> {
	static_assert((p == Peripheral::Fmc),"GpioD9::D14 only connects to Fmc!"); };
template<Peripheral p> struct SignalConnection<DataD9::Da14, p> {
	static_assert((p == Peripheral::Fmc),"GpioD9::Da14 only connects to Fmc!"); };
template<Peripheral p> struct SignalConnection<DataD9::Datin3, p> {
	static_assert((p == Peripheral::Dfsdm1),"GpioD9::Datin3 only connects to Dfsdm1!"); };
template<Peripheral p> struct SignalConnection<DataD9::Rx, p> {
	static_assert((p == Peripheral::Usart3),"GpioD9::Rx only connects to Usart3!"); };
template<> struct SignalConnection<DataD9::BitBang, Peripheral::BitBang> { static constexpr int8_t af = -1; };
template<> struct SignalConnection<DataD9::D14, Peripheral::Fmc> { static constexpr int8_t af = 12; };
template<> struct SignalConnection<DataD9::Da14, Peripheral::Fmc> { static constexpr int8_t af = 12; };
template<> struct SignalConnection<DataD9::Datin3, Peripheral::Dfsdm1> { static constexpr int8_t af = 3; };
template<> struct SignalConnection<DataD9::Rx, Peripheral::Usart3> { static constexpr int8_t af = 7; };

struct DataD10 {
	static constexpr Gpio::Port port = Gpio::Port::D;
	static constexpr uint8_t pin = 10;
	struct BitBang { using Data = DataD10; static constexpr Gpio::Signal Signal = Gpio::Signal::BitBang; };
	struct B3 { using Data = DataD10; static constexpr Gpio::Signal Signal = Gpio::Signal::B3; };
	struct Ck { using Data = DataD10; static constexpr Gpio::Signal Signal = Gpio::Signal::Ck; };
	struct Ckout { using Data = DataD10; static constexpr Gpio::Signal Signal = Gpio::Signal::Ckout; };
	struct D15 { using Data = DataD10; static constexpr Gpio::Signal Signal = Gpio::Signal::D15; };
	struct Da15 { using Data = DataD10; static constexpr Gpio::Signal Signal = Gpio::Signal::Da15; };
};
template<Peripheral p> struct SignalConnection<DataD10::BitBang, p> {
	static_assert(p == Peripheral::BitBang, "GpioD10::BitBang only connects to software drivers!"); };
template<Peripheral p> struct SignalConnection<DataD10::B3, p> {
	static_assert((p == Peripheral::Ltdc),"GpioD10::B3 only connects to Ltdc!"); };
template<Peripheral p> struct SignalConnection<DataD10::Ck, p> {
	static_assert((p == Peripheral::Usart3),"GpioD10::Ck only connects to Usart3!"); };
template<Peripheral p> struct SignalConnection<DataD10::Ckout, p> {
	static_assert((p == Peripheral::Dfsdm1),"GpioD10::Ckout only connects to Dfsdm1!"); };
template<Peripheral p> struct SignalConnection<DataD10::D15, p> {
	static_assert((p == Peripheral::Fmc),"GpioD10::D15 only connects to Fmc!"); };
template<Peripheral p> struct SignalConnection<DataD10::Da15, p> {
	static_assert((p == Peripheral::Fmc),"GpioD10::Da15 only connects to Fmc!"); };
template<> struct SignalConnection<DataD10::BitBang, Peripheral::BitBang> { static constexpr int8_t af = -1; };
template<> struct SignalConnection<DataD10::B3, Peripheral::Ltdc> { static constexpr int8_t af = 14; };
template<> struct SignalConnection<DataD10::Ck, Peripheral::Usart3> { static constexpr int8_t af = 7; };
template<> struct SignalConnection<DataD10::Ckout, Peripheral::Dfsdm1> { static constexpr int8_t af = 3; };
template<> struct SignalConnection<DataD10::D15, Peripheral::Fmc> { static constexpr int8_t af = 12; };
template<> struct SignalConnection<DataD10::Da15, Peripheral::Fmc> { static constexpr int8_t af = 12; };

struct DataD11 {
	static constexpr Gpio::Port port = Gpio::Port::D;
	static constexpr uint8_t pin = 11;
	struct BitBang { using Data = DataD11; static constexpr Gpio::Signal Signal = Gpio::Signal::BitBang; };
	struct A16 { using Data = DataD11; static constexpr Gpio::Signal Signal = Gpio::Signal::A16; };
	struct Cle { using Data = DataD11; static constexpr Gpio::Signal Signal = Gpio::Signal::Cle; };
	struct Cts { using Data = DataD11; static constexpr Gpio::Signal Signal = Gpio::Signal::Cts; };
	struct In2 { using Data = DataD11; static constexpr Gpio::Signal Signal = Gpio::Signal::In2; };
	struct Nss { using Data = DataD11; static constexpr Gpio::Signal Signal = Gpio::Signal::Nss; };
	struct P1io0 { using Data = DataD11; static constexpr Gpio::Signal Signal = Gpio::Signal::P1io0; };
	struct Sda { using Data = DataD11; static constexpr Gpio::Signal Signal = Gpio::Signal::Sda; };
	struct Smba { using Data = DataD11; static constexpr Gpio::Signal Signal = Gpio::Signal::Smba; };
};
template<Peripheral p> struct SignalConnection<DataD11::BitBang, p> {
	static_assert(p == Peripheral::BitBang, "GpioD11::BitBang only connects to software drivers!"); };
template<Peripheral p> struct SignalConnection<DataD11::A16, p> {
	static_assert((p == Peripheral::Fmc),"GpioD11::A16 only connects to Fmc!"); };
template<Peripheral p> struct SignalConnection<DataD11::Cle, p> {
	static_assert((p == Peripheral::Fmc),"GpioD11::Cle only connects to Fmc!"); };
template<Peripheral p> struct SignalConnection<DataD11::Cts, p> {
	static_assert((p == Peripheral::Usart3),"GpioD11::Cts only connects to Usart3!"); };
template<Peripheral p> struct SignalConnection<DataD11::In2, p> {
	static_assert((p == Peripheral::Lptim2),"GpioD11::In2 only connects to Lptim2!"); };
template<Peripheral p> struct SignalConnection<DataD11::Nss, p> {
	static_assert((p == Peripheral::Usart3),"GpioD11::Nss only connects to Usart3!"); };
template<Peripheral p> struct SignalConnection<DataD11::P1io0, p> {
	static_assert((p == Peripheral::Octospim),"GpioD11::P1io0 only connects to Octospim!"); };
template<Peripheral p> struct SignalConnection<DataD11::Sda, p> {
	static_assert((p == Peripheral::Sai4),"GpioD11::Sda only connects to Sai4!"); };
template<Peripheral p> struct SignalConnection<DataD11::Smba, p> {
	static_assert((p == Peripheral::I2c4),"GpioD11::Smba only connects to I2c4!"); };
template<> struct SignalConnection<DataD11::BitBang, Peripheral::BitBang> { static constexpr int8_t af = -1; };
template<> struct SignalConnection<DataD11::A16, Peripheral::Fmc> { static constexpr int8_t af = 12; };
template<> struct SignalConnection<DataD11::Cle, Peripheral::Fmc> { static constexpr int8_t af = 12; };
template<> struct SignalConnection<DataD11::Cts, Peripheral::Usart3> { static constexpr int8_t af = 7; };
template<> struct SignalConnection<DataD11::In2, Peripheral::Lptim2> { static constexpr int8_t af = 3; };
template<> struct SignalConnection<DataD11::Nss, Peripheral::Usart3> { static constexpr int8_t af = 7; };
template<> struct SignalConnection<DataD11::P1io0, Peripheral::Octospim> { static constexpr int8_t af = 9; };
template<> struct SignalConnection<DataD11::Sda, Peripheral::Sai4> { static constexpr int8_t af = 10; };
template<> struct SignalConnection<DataD11::Smba, Peripheral::I2c4> { static constexpr int8_t af = 4; };

struct DataD12 {
	static constexpr Gpio::Port port = Gpio::Port::D;
	static constexpr uint8_t pin = 12;
	struct BitBang { using Data = DataD12; static constexpr Gpio::Signal Signal = Gpio::Signal::BitBang; };
	struct A17 { using Data = DataD12; static constexpr Gpio::Signal Signal = Gpio::Signal::A17; };
	struct Ale { using Data = DataD12; static constexpr Gpio::Signal Signal = Gpio::Signal::Ale; };
	struct Ch1 { using Data = DataD12; static constexpr Gpio::Signal Signal = Gpio::Signal::Ch1; };
	struct D12 { using Data = DataD12; static constexpr Gpio::Signal Signal = Gpio::Signal::D12; };
	struct De { using Data = DataD12; static constexpr Gpio::Signal Signal = Gpio::Signal::De; };
	struct Fsa { using Data = DataD12; static constexpr Gpio::Signal Signal = Gpio::Signal::Fsa; };
	struct In1 { using Data = DataD12; static constexpr Gpio::Signal Signal = Gpio::Signal::In1; };
	struct P1io1 { using Data = DataD12; static constexpr Gpio::Signal Signal = Gpio::Signal::P1io1; };
	struct Rts { using Data = DataD12; static constexpr Gpio::Signal Signal = Gpio::Signal::Rts; };
	struct Rx { using Data = DataD12; static constexpr Gpio::Signal Signal = Gpio::Signal::Rx; };
	struct Scl { using Data = DataD12; static constexpr Gpio::Signal Signal = Gpio::Signal::Scl; };
};
template<Peripheral p> struct SignalConnection<DataD12::BitBang, p> {
	static_assert(p == Peripheral::BitBang, "GpioD12::BitBang only connects to software drivers!"); };
template<Peripheral p> struct SignalConnection<DataD12::A17, p> {
	static_assert((p == Peripheral::Fmc),"GpioD12::A17 only connects to Fmc!"); };
template<Peripheral p> struct SignalConnection<DataD12::Ale, p> {
	static_assert((p == Peripheral::Fmc),"GpioD12::Ale only connects to Fmc!"); };
template<Peripheral p> struct SignalConnection<DataD12::Ch1, p> {
	static_assert((p == Peripheral::Tim4),"GpioD12::Ch1 only connects to Tim4!"); };
template<Peripheral p> struct SignalConnection<DataD12::D12, p> {
	static_assert((p == Peripheral::Dcmi) or (p == Peripheral::Pssi),
		"GpioD12::D12 only connects to Dcmi or Pssi!"); };
template<Peripheral p> struct SignalConnection<DataD12::De, p> {
	static_assert((p == Peripheral::Usart3),"GpioD12::De only connects to Usart3!"); };
template<Peripheral p> struct SignalConnection<DataD12::Fsa, p> {
	static_assert((p == Peripheral::Sai4),"GpioD12::Fsa only connects to Sai4!"); };
template<Peripheral p> struct SignalConnection<DataD12::In1, p> {
	static_assert((p == Peripheral::Lptim1) or (p == Peripheral::Lptim2),
		"GpioD12::In1 only connects to Lptim1 or Lptim2!"); };
template<Peripheral p> struct SignalConnection<DataD12::P1io1, p> {
	static_assert((p == Peripheral::Octospim),"GpioD12::P1io1 only connects to Octospim!"); };
template<Peripheral p> struct SignalConnection<DataD12::Rts, p> {
	static_assert((p == Peripheral::Usart3),"GpioD12::Rts only connects to Usart3!"); };
template<Peripheral p> struct SignalConnection<DataD12::Rx, p> {
	static_assert((p == Peripheral::Fdcan3),"GpioD12::Rx only connects to Fdcan3!"); };
template<Peripheral p> struct SignalConnection<DataD12::Scl, p> {
	static_assert((p == Peripheral::I2c4),"GpioD12::Scl only connects to I2c4!"); };
template<> struct SignalConnection<DataD12::BitBang, Peripheral::BitBang> { static constexpr int8_t af = -1; };
template<> struct SignalConnection<DataD12::A17, Peripheral::Fmc> { static constexpr int8_t af = 12; };
template<> struct SignalConnection<DataD12::Ale, Peripheral::Fmc> { static constexpr int8_t af = 12; };
template<> struct SignalConnection<DataD12::Ch1, Peripheral::Tim4> { static constexpr int8_t af = 2; };
template<> struct SignalConnection<DataD12::D12, Peripheral::Dcmi> { static constexpr int8_t af = 13; };
template<> struct SignalConnection<DataD12::D12, Peripheral::Pssi> { static constexpr int8_t af = 13; };
template<> struct SignalConnection<DataD12::De, Peripheral::Usart3> { static constexpr int8_t af = 7; };
template<> struct SignalConnection<DataD12::Fsa, Peripheral::Sai4> { static constexpr int8_t af = 10; };
template<> struct SignalConnection<DataD12::In1, Peripheral::Lptim1> { static constexpr int8_t af = 1; };
template<> struct SignalConnection<DataD12::In1, Peripheral::Lptim2> { static constexpr int8_t af = 3; };
template<> struct SignalConnection<DataD12::P1io1, Peripheral::Octospim> { static constexpr int8_t af = 9; };
template<> struct SignalConnection<DataD12::Rts, Peripheral::Usart3> { static constexpr int8_t af = 7; };
template<> struct SignalConnection<DataD12::Rx, Peripheral::Fdcan3> { static constexpr int8_t af = 5; };
template<> struct SignalConnection<DataD12::Scl, Peripheral::I2c4> { static constexpr int8_t af = 4; };

struct DataD13 {
	static constexpr Gpio::Port port = Gpio::Port::D;
	static constexpr uint8_t pin = 13;
	struct BitBang { using Data = DataD13; static constexpr Gpio::Signal Signal = Gpio::Signal::BitBang; };
	struct A18 { using Data = DataD13; static constexpr Gpio::Signal Signal = Gpio::Signal::A18; };
	struct Ch2 { using Data = DataD13; static constexpr Gpio::Signal Signal = Gpio::Signal::Ch2; };
	struct D13 { using Data = DataD13; static constexpr Gpio::Signal Signal = Gpio::Signal::D13; };
	struct De { using Data = DataD13; static constexpr Gpio::Signal Signal = Gpio::Signal::De; };
	struct Out { using Data = DataD13; static constexpr Gpio::Signal Signal = Gpio::Signal::Out; };
	struct P1io3 { using Data = DataD13; static constexpr Gpio::Signal Signal = Gpio::Signal::P1io3; };
	struct Rts { using Data = DataD13; static constexpr Gpio::Signal Signal = Gpio::Signal::Rts; };
	struct Scka { using Data = DataD13; static constexpr Gpio::Signal Signal = Gpio::Signal::Scka; };
	struct Sda { using Data = DataD13; static constexpr Gpio::Signal Signal = Gpio::Signal::Sda; };
	struct Tx { using Data = DataD13; static constexpr Gpio::Signal Signal = Gpio::Signal::Tx; };
};
template<Peripheral p> struct SignalConnection<DataD13::BitBang, p> {
	static_assert(p == Peripheral::BitBang, "GpioD13::BitBang only connects to software drivers!"); };
template<Peripheral p> struct SignalConnection<DataD13::A18, p> {
	static_assert((p == Peripheral::Fmc),"GpioD13::A18 only connects to Fmc!"); };
template<Peripheral p> struct SignalConnection<DataD13::Ch2, p> {
	static_assert((p == Peripheral::Tim4),"GpioD13::Ch2 only connects to Tim4!"); };
template<Peripheral p> struct SignalConnection<DataD13::D13, p> {
	static_assert((p == Peripheral::Dcmi) or (p == Peripheral::Pssi),
		"GpioD13::D13 only connects to Dcmi or Pssi!"); };
template<Peripheral p> struct SignalConnection<DataD13::De, p> {
	static_assert((p == Peripheral::Uart9),"GpioD13::De only connects to Uart9!"); };
template<Peripheral p> struct SignalConnection<DataD13::Out, p> {
	static_assert((p == Peripheral::Lptim1),"GpioD13::Out only connects to Lptim1!"); };
template<Peripheral p> struct SignalConnection<DataD13::P1io3, p> {
	static_assert((p == Peripheral::Octospim),"GpioD13::P1io3 only connects to Octospim!"); };
template<Peripheral p> struct SignalConnection<DataD13::Rts, p> {
	static_assert((p == Peripheral::Uart9),"GpioD13::Rts only connects to Uart9!"); };
template<Peripheral p> struct SignalConnection<DataD13::Scka, p> {
	static_assert((p == Peripheral::Sai4),"GpioD13::Scka only connects to Sai4!"); };
template<Peripheral p> struct SignalConnection<DataD13::Sda, p> {
	static_assert((p == Peripheral::I2c4),"GpioD13::Sda only connects to I2c4!"); };
template<Peripheral p> struct SignalConnection<DataD13::Tx, p> {
	static_assert((p == Peripheral::Fdcan3),"GpioD13::Tx only connects to Fdcan3!"); };
template<> struct SignalConnection<DataD13::BitBang, Peripheral::BitBang> { static constexpr int8_t af = -1; };
template<> struct SignalConnection<DataD13::A18, Peripheral::Fmc> { static constexpr int8_t af = 12; };
template<> struct SignalConnection<DataD13::Ch2, Peripheral::Tim4> { static constexpr int8_t af = 2; };
template<> struct SignalConnection<DataD13::D13, Peripheral::Dcmi> { static constexpr int8_t af = 13; };
template<> struct SignalConnection<DataD13::D13, Peripheral::Pssi> { static constexpr int8_t af = 13; };
template<> struct SignalConnection<DataD13::De, Peripheral::Uart9> { static constexpr int8_t af = 11; };
template<> struct SignalConnection<DataD13::Out, Peripheral::Lptim1> { static constexpr int8_t af = 1; };
template<> struct SignalConnection<DataD13::P1io3, Peripheral::Octospim> { static constexpr int8_t af = 9; };
template<> struct SignalConnection<DataD13::Rts, Peripheral::Uart9> { static constexpr int8_t af = 11; };
template<> struct SignalConnection<DataD13::Scka, Peripheral::Sai4> { static constexpr int8_t af = 10; };
template<> struct SignalConnection<DataD13::Sda, Peripheral::I2c4> { static constexpr int8_t af = 4; };
template<> struct SignalConnection<DataD13::Tx, Peripheral::Fdcan3> { static constexpr int8_t af = 5; };

struct DataD14 {
	static constexpr Gpio::Port port = Gpio::Port::D;
	static constexpr uint8_t pin = 14;
	struct BitBang { using Data = DataD14; static constexpr Gpio::Signal Signal = Gpio::Signal::BitBang; };
	struct Ch3 { using Data = DataD14; static constexpr Gpio::Signal Signal = Gpio::Signal::Ch3; };
	struct Cts { using Data = DataD14; static constexpr Gpio::Signal Signal = Gpio::Signal::Cts; };
	struct D0 { using Data = DataD14; static constexpr Gpio::Signal Signal = Gpio::Signal::D0; };
	struct Da0 { using Data = DataD14; static constexpr Gpio::Signal Signal = Gpio::Signal::Da0; };
	struct Rx { using Data = DataD14; static constexpr Gpio::Signal Signal = Gpio::Signal::Rx; };
};
template<Peripheral p> struct SignalConnection<DataD14::BitBang, p> {
	static_assert(p == Peripheral::BitBang, "GpioD14::BitBang only connects to software drivers!"); };
template<Peripheral p> struct SignalConnection<DataD14::Ch3, p> {
	static_assert((p == Peripheral::Tim4),"GpioD14::Ch3 only connects to Tim4!"); };
template<Peripheral p> struct SignalConnection<DataD14::Cts, p> {
	static_assert((p == Peripheral::Uart8),"GpioD14::Cts only connects to Uart8!"); };
template<Peripheral p> struct SignalConnection<DataD14::D0, p> {
	static_assert((p == Peripheral::Fmc),"GpioD14::D0 only connects to Fmc!"); };
template<Peripheral p> struct SignalConnection<DataD14::Da0, p> {
	static_assert((p == Peripheral::Fmc),"GpioD14::Da0 only connects to Fmc!"); };
template<Peripheral p> struct SignalConnection<DataD14::Rx, p> {
	static_assert((p == Peripheral::Uart9),"GpioD14::Rx only connects to Uart9!"); };
template<> struct SignalConnection<DataD14::BitBang, Peripheral::BitBang> { static constexpr int8_t af = -1; };
template<> struct SignalConnection<DataD14::Ch3, Peripheral::Tim4> { static constexpr int8_t af = 2; };
template<> struct SignalConnection<DataD14::Cts, Peripheral::Uart8> { static constexpr int8_t af = 8; };
template<> struct SignalConnection<DataD14::D0, Peripheral::Fmc> { static constexpr int8_t af = 12; };
template<> struct SignalConnection<DataD14::Da0, Peripheral::Fmc> { static constexpr int8_t af = 12; };
template<> struct SignalConnection<DataD14::Rx, Peripheral::Uart9> { static constexpr int8_t af = 11; };

struct DataD15 {
	static constexpr Gpio::Port port = Gpio::Port::D;
	static constexpr uint8_t pin = 15;
	struct BitBang { using Data = DataD15; static constexpr Gpio::Signal Signal = Gpio::Signal::BitBang; };
	struct Ch4 { using Data = DataD15; static constexpr Gpio::Signal Signal = Gpio::Signal::Ch4; };
	struct D1 { using Data = DataD15; static constexpr Gpio::Signal Signal = Gpio::Signal::D1; };
	struct Da1 { using Data = DataD15; static constexpr Gpio::Signal Signal = Gpio::Signal::Da1; };
	struct De { using Data = DataD15; static constexpr Gpio::Signal Signal = Gpio::Signal::De; };
	struct Rts { using Data = DataD15; static constexpr Gpio::Signal Signal = Gpio::Signal::Rts; };
	struct Tx { using Data = DataD15; static constexpr Gpio::Signal Signal = Gpio::Signal::Tx; };
};
template<Peripheral p> struct SignalConnection<DataD15::BitBang, p> {
	static_assert(p == Peripheral::BitBang, "GpioD15::BitBang only connects to software drivers!"); };
template<Peripheral p> struct SignalConnection<DataD15::Ch4, p> {
	static_assert((p == Peripheral::Tim4),"GpioD15::Ch4 only connects to Tim4!"); };
template<Peripheral p> struct SignalConnection<DataD15::D1, p> {
	static_assert((p == Peripheral::Fmc),"GpioD15::D1 only connects to Fmc!"); };
template<Peripheral p> struct SignalConnection<DataD15::Da1, p> {
	static_assert((p == Peripheral::Fmc),"GpioD15::Da1 only connects to Fmc!"); };
template<Peripheral p> struct SignalConnection<DataD15::De, p> {
	static_assert((p == Peripheral::Uart8),"GpioD15::De only connects to Uart8!"); };
template<Peripheral p> struct SignalConnection<DataD15::Rts, p> {
	static_assert((p == Peripheral::Uart8),"GpioD15::Rts only connects to Uart8!"); };
template<Peripheral p> struct SignalConnection<DataD15::Tx, p> {
	static_assert((p == Peripheral::Uart9),"GpioD15::Tx only connects to Uart9!"); };
template<> struct SignalConnection<DataD15::BitBang, Peripheral::BitBang> { static constexpr int8_t af = -1; };
template<> struct SignalConnection<DataD15::Ch4, Peripheral::Tim4> { static constexpr int8_t af = 2; };
template<> struct SignalConnection<DataD15::D1, Peripheral::Fmc> { static constexpr int8_t af = 12; };
template<> struct SignalConnection<DataD15::Da1, Peripheral::Fmc> { static constexpr int8_t af = 12; };
template<> struct SignalConnection<DataD15::De, Peripheral::Uart8> { static constexpr int8_t af = 8; };
template<> struct SignalConnection<DataD15::Rts, Peripheral::Uart8> { static constexpr int8_t af = 8; };
template<> struct SignalConnection<DataD15::Tx, Peripheral::Uart9> { static constexpr int8_t af = 11; };

struct DataE0 {
	static constexpr Gpio::Port port = Gpio::Port::E;
	static constexpr uint8_t pin = 0;
	struct BitBang { using Data = DataE0; static constexpr Gpio::Signal Signal = Gpio::Signal::BitBang; };
	struct D2 { using Data = DataE0; static constexpr Gpio::Signal Signal = Gpio::Signal::D2; };
	struct Etr { using Data = DataE0; static constexpr Gpio::Signal Signal = Gpio::Signal::Etr; };
	struct Mclka { using Data = DataE0; static constexpr Gpio::Signal Signal = Gpio::Signal::Mclka; };
	struct Nbl0 { using Data = DataE0; static constexpr Gpio::Signal Signal = Gpio::Signal::Nbl0; };
	struct R0 { using Data = DataE0; static constexpr Gpio::Signal Signal = Gpio::Signal::R0; };
	struct Rx { using Data = DataE0; static constexpr Gpio::Signal Signal = Gpio::Signal::Rx; };
};
template<Peripheral p> struct SignalConnection<DataE0::BitBang, p> {
	static_assert(p == Peripheral::BitBang, "GpioE0::BitBang only connects to software drivers!"); };
template<Peripheral p> struct SignalConnection<DataE0::D2, p> {
	static_assert((p == Peripheral::Dcmi) or (p == Peripheral::Pssi),
		"GpioE0::D2 only connects to Dcmi or Pssi!"); };
template<Peripheral p> struct SignalConnection<DataE0::Etr, p> {
	static_assert((p == Peripheral::Lptim1) or (p == Peripheral::Tim4) or (p == Peripheral::Lptim2),
		"GpioE0::Etr only connects to Lptim1 or Tim4 or Lptim2!"); };
template<Peripheral p> struct SignalConnection<DataE0::Mclka, p> {
	static_assert((p == Peripheral::Sai4),"GpioE0::Mclka only connects to Sai4!"); };
template<Peripheral p> struct SignalConnection<DataE0::Nbl0, p> {
	static_assert((p == Peripheral::Fmc),"GpioE0::Nbl0 only connects to Fmc!"); };
template<Peripheral p> struct SignalConnection<DataE0::R0, p> {
	static_assert((p == Peripheral::Ltdc),"GpioE0::R0 only connects to Ltdc!"); };
template<Peripheral p> struct SignalConnection<DataE0::Rx, p> {
	static_assert((p == Peripheral::Uart8),"GpioE0::Rx only connects to Uart8!"); };
template<> struct SignalConnection<DataE0::BitBang, Peripheral::BitBang> { static constexpr int8_t af = -1; };
template<> struct SignalConnection<DataE0::D2, Peripheral::Dcmi> { static constexpr int8_t af = 13; };
template<> struct SignalConnection<DataE0::D2, Peripheral::Pssi> { static constexpr int8_t af = 13; };
template<> struct SignalConnection<DataE0::Etr, Peripheral::Lptim1> { static constexpr int8_t af = 1; };
template<> struct SignalConnection<DataE0::Etr, Peripheral::Tim4> { static constexpr int8_t af = 2; };
template<> struct SignalConnection<DataE0::Etr, Peripheral::Lptim2> { static constexpr int8_t af = 4; };
template<> struct SignalConnection<DataE0::Mclka, Peripheral::Sai4> { static constexpr int8_t af = 10; };
template<> struct SignalConnection<DataE0::Nbl0, Peripheral::Fmc> { static constexpr int8_t af = 12; };
template<> struct SignalConnection<DataE0::R0, Peripheral::Ltdc> { static constexpr int8_t af = 14; };
template<> struct SignalConnection<DataE0::Rx, Peripheral::Uart8> { static constexpr int8_t af = 8; };

struct DataE1 {
	static constexpr Gpio::Port port = Gpio::Port::E;
	static constexpr uint8_t pin = 1;
	struct BitBang { using Data = DataE1; static constexpr Gpio::Signal Signal = Gpio::Signal::BitBang; };
	struct D3 { using Data = DataE1; static constexpr Gpio::Signal Signal = Gpio::Signal::D3; };
	struct In2 { using Data = DataE1; static constexpr Gpio::Signal Signal = Gpio::Signal::In2; };
	struct Nbl1 { using Data = DataE1; static constexpr Gpio::Signal Signal = Gpio::Signal::Nbl1; };
	struct R6 { using Data = DataE1; static constexpr Gpio::Signal Signal = Gpio::Signal::R6; };
	struct Tx { using Data = DataE1; static constexpr Gpio::Signal Signal = Gpio::Signal::Tx; };
};
template<Peripheral p> struct SignalConnection<DataE1::BitBang, p> {
	static_assert(p == Peripheral::BitBang, "GpioE1::BitBang only connects to software drivers!"); };
template<Peripheral p> struct SignalConnection<DataE1::D3, p> {
	static_assert((p == Peripheral::Dcmi) or (p == Peripheral::Pssi),
		"GpioE1::D3 only connects to Dcmi or Pssi!"); };
template<Peripheral p> struct SignalConnection<DataE1::In2, p> {
	static_assert((p == Peripheral::Lptim1),"GpioE1::In2 only connects to Lptim1!"); };
template<Peripheral p> struct SignalConnection<DataE1::Nbl1, p> {
	static_assert((p == Peripheral::Fmc),"GpioE1::Nbl1 only connects to Fmc!"); };
template<Peripheral p> struct SignalConnection<DataE1::R6, p> {
	static_assert((p == Peripheral::Ltdc),"GpioE1::R6 only connects to Ltdc!"); };
template<Peripheral p> struct SignalConnection<DataE1::Tx, p> {
	static_assert((p == Peripheral::Uart8),"GpioE1::Tx only connects to Uart8!"); };
template<> struct SignalConnection<DataE1::BitBang, Peripheral::BitBang> { static constexpr int8_t af = -1; };
template<> struct SignalConnection<DataE1::D3, Peripheral::Dcmi> { static constexpr int8_t af = 13; };
template<> struct SignalConnection<DataE1::D3, Peripheral::Pssi> { static constexpr int8_t af = 13; };
template<> struct SignalConnection<DataE1::In2, Peripheral::Lptim1> { static constexpr int8_t af = 1; };
template<> struct SignalConnection<DataE1::Nbl1, Peripheral::Fmc> { static constexpr int8_t af = 12; };
template<> struct SignalConnection<DataE1::R6, Peripheral::Ltdc> { static constexpr int8_t af = 14; };
template<> struct SignalConnection<DataE1::Tx, Peripheral::Uart8> { static constexpr int8_t af = 8; };

struct DataE2 {
	static constexpr Gpio::Port port = Gpio::Port::E;
	static constexpr uint8_t pin = 2;
	struct BitBang { using Data = DataE2; static constexpr Gpio::Signal Signal = Gpio::Signal::BitBang; };
	struct A23 { using Data = DataE2; static constexpr Gpio::Signal Signal = Gpio::Signal::A23; };
	struct Ck1 { using Data = DataE2; static constexpr Gpio::Signal Signal = Gpio::Signal::Ck1; };
	struct Mclka { using Data = DataE2; static constexpr Gpio::Signal Signal = Gpio::Signal::Mclka; };
	struct P1io2 { using Data = DataE2; static constexpr Gpio::Signal Signal = Gpio::Signal::P1io2; };
	struct Rx { using Data = DataE2; static constexpr Gpio::Signal Signal = Gpio::Signal::Rx; };
	struct Sck { using Data = DataE2; static constexpr Gpio::Signal Signal = Gpio::Signal::Sck; };
	struct Traceclk { using Data = DataE2; static constexpr Gpio::Signal Signal = Gpio::Signal::Traceclk; };
	struct Txd3 { using Data = DataE2; static constexpr Gpio::Signal Signal = Gpio::Signal::Txd3; };
};
template<Peripheral p> struct SignalConnection<DataE2::BitBang, p> {
	static_assert(p == Peripheral::BitBang, "GpioE2::BitBang only connects to software drivers!"); };
template<Peripheral p> struct SignalConnection<DataE2::A23, p> {
	static_assert((p == Peripheral::Fmc),"GpioE2::A23 only connects to Fmc!"); };
template<Peripheral p> struct SignalConnection<DataE2::Ck1, p> {
	static_assert((p == Peripheral::Sai1) or (p == Peripheral::Sai4),
		"GpioE2::Ck1 only connects to Sai1 or Sai4!"); };
template<Peripheral p> struct SignalConnection<DataE2::Mclka, p> {
	static_assert((p == Peripheral::Sai1) or (p == Peripheral::Sai4),
		"GpioE2::Mclka only connects to Sai1 or Sai4!"); };
template<Peripheral p> struct SignalConnection<DataE2::P1io2, p> {
	static_assert((p == Peripheral::Octospim),"GpioE2::P1io2 only connects to Octospim!"); };
template<Peripheral p> struct SignalConnection<DataE2::Rx, p> {
	static_assert((p == Peripheral::Usart10),"GpioE2::Rx only connects to Usart10!"); };
template<Peripheral p> struct SignalConnection<DataE2::Sck, p> {
	static_assert((p == Peripheral::Spi4),"GpioE2::Sck only connects to Spi4!"); };
template<Peripheral p> struct SignalConnection<DataE2::Traceclk, p> {
	static_assert((p == Peripheral::Debug),"GpioE2::Traceclk only connects to Debug!"); };
template<Peripheral p> struct SignalConnection<DataE2::Txd3, p> {
	static_assert((p == Peripheral::Eth),"GpioE2::Txd3 only connects to Eth!"); };
template<> struct SignalConnection<DataE2::BitBang, Peripheral::BitBang> { static constexpr int8_t af = -1; };
template<> struct SignalConnection<DataE2::A23, Peripheral::Fmc> { static constexpr int8_t af = 12; };
template<> struct SignalConnection<DataE2::Ck1, Peripheral::Sai1> { static constexpr int8_t af = 2; };
template<> struct SignalConnection<DataE2::Ck1, Peripheral::Sai4> { static constexpr int8_t af = 10; };
template<> struct SignalConnection<DataE2::Mclka, Peripheral::Sai1> { static constexpr int8_t af = 6; };
template<> struct SignalConnection<DataE2::Mclka, Peripheral::Sai4> { static constexpr int8_t af = 8; };
template<> struct SignalConnection<DataE2::P1io2, Peripheral::Octospim> { static constexpr int8_t af = 9; };
template<> struct SignalConnection<DataE2::Rx, Peripheral::Usart10> { static constexpr int8_t af = 4; };
template<> struct SignalConnection<DataE2::Sck, Peripheral::Spi4> { static constexpr int8_t af = 5; };
template<> struct SignalConnection<DataE2::Traceclk, Peripheral::Debug> { static constexpr int8_t af = 0; };
template<> struct SignalConnection<DataE2::Txd3, Peripheral::Eth> { static constexpr int8_t af = 11; };

struct DataE3 {
	static constexpr Gpio::Port port = Gpio::Port::E;
	static constexpr uint8_t pin = 3;
	struct BitBang { using Data = DataE3; static constexpr Gpio::Signal Signal = Gpio::Signal::BitBang; };
	struct A19 { using Data = DataE3; static constexpr Gpio::Signal Signal = Gpio::Signal::A19; };
	struct Bkin { using Data = DataE3; static constexpr Gpio::Signal Signal = Gpio::Signal::Bkin; };
	struct Sdb { using Data = DataE3; static constexpr Gpio::Signal Signal = Gpio::Signal::Sdb; };
	struct Traced0 { using Data = DataE3; static constexpr Gpio::Signal Signal = Gpio::Signal::Traced0; };
	struct Tx { using Data = DataE3; static constexpr Gpio::Signal Signal = Gpio::Signal::Tx; };
};
template<Peripheral p> struct SignalConnection<DataE3::BitBang, p> {
	static_assert(p == Peripheral::BitBang, "GpioE3::BitBang only connects to software drivers!"); };
template<Peripheral p> struct SignalConnection<DataE3::A19, p> {
	static_assert((p == Peripheral::Fmc),"GpioE3::A19 only connects to Fmc!"); };
template<Peripheral p> struct SignalConnection<DataE3::Bkin, p> {
	static_assert((p == Peripheral::Tim15),"GpioE3::Bkin only connects to Tim15!"); };
template<Peripheral p> struct SignalConnection<DataE3::Sdb, p> {
	static_assert((p == Peripheral::Sai1) or (p == Peripheral::Sai4),
		"GpioE3::Sdb only connects to Sai1 or Sai4!"); };
template<Peripheral p> struct SignalConnection<DataE3::Traced0, p> {
	static_assert((p == Peripheral::Debug),"GpioE3::Traced0 only connects to Debug!"); };
template<Peripheral p> struct SignalConnection<DataE3::Tx, p> {
	static_assert((p == Peripheral::Usart10),"GpioE3::Tx only connects to Usart10!"); };
template<> struct SignalConnection<DataE3::BitBang, Peripheral::BitBang> { static constexpr int8_t af = -1; };
template<> struct SignalConnection<DataE3::A19, Peripheral::Fmc> { static constexpr int8_t af = 12; };
template<> struct SignalConnection<DataE3::Bkin, Peripheral::Tim15> { static constexpr int8_t af = 4; };
template<> struct SignalConnection<DataE3::Sdb, Peripheral::Sai1> { static constexpr int8_t af = 6; };
template<> struct SignalConnection<DataE3::Sdb, Peripheral::Sai4> { static constexpr int8_t af = 8; };
template<> struct SignalConnection<DataE3::Traced0, Peripheral::Debug> { static constexpr int8_t af = 0; };
template<> struct SignalConnection<DataE3::Tx, Peripheral::Usart10> { static constexpr int8_t af = 11; };

struct DataE4 {
	static constexpr Gpio::Port port = Gpio::Port::E;
	static constexpr uint8_t pin = 4;
	struct BitBang { using Data = DataE4; static constexpr Gpio::Signal Signal = Gpio::Signal::BitBang; };
	struct A20 { using Data = DataE4; static constexpr Gpio::Signal Signal = Gpio::Signal::A20; };
	struct B0 { using Data = DataE4; static constexpr Gpio::Signal Signal = Gpio::Signal::B0; };
	struct Ch1n { using Data = DataE4; static constexpr Gpio::Signal Signal = Gpio::Signal::Ch1n; };
	struct D2 { using Data = DataE4; static constexpr Gpio::Signal Signal = Gpio::Signal::D2; };
	struct D4 { using Data = DataE4; static constexpr Gpio::Signal Signal = Gpio::Signal::D4; };
	struct Datin3 { using Data = DataE4; static constexpr Gpio::Signal Signal = Gpio::Signal::Datin3; };
	struct Fsa { using Data = DataE4; static constexpr Gpio::Signal Signal = Gpio::Signal::Fsa; };
	struct Nss { using Data = DataE4; static constexpr Gpio::Signal Signal = Gpio::Signal::Nss; };
	struct Traced1 { using Data = DataE4; static constexpr Gpio::Signal Signal = Gpio::Signal::Traced1; };
};
template<Peripheral p> struct SignalConnection<DataE4::BitBang, p> {
	static_assert(p == Peripheral::BitBang, "GpioE4::BitBang only connects to software drivers!"); };
template<Peripheral p> struct SignalConnection<DataE4::A20, p> {
	static_assert((p == Peripheral::Fmc),"GpioE4::A20 only connects to Fmc!"); };
template<Peripheral p> struct SignalConnection<DataE4::B0, p> {
	static_assert((p == Peripheral::Ltdc),"GpioE4::B0 only connects to Ltdc!"); };
template<Peripheral p> struct SignalConnection<DataE4::Ch1n, p> {
	static_assert((p == Peripheral::Tim15),"GpioE4::Ch1n only connects to Tim15!"); };
template<Peripheral p> struct SignalConnection<DataE4::D2, p> {
	static_assert((p == Peripheral::Sai1) or (p == Peripheral::Sai4),
		"GpioE4::D2 only connects to Sai1 or Sai4!"); };
template<Peripheral p> struct SignalConnection<DataE4::D4, p> {
	static_assert((p == Peripheral::Dcmi) or (p == Peripheral::Pssi),
		"GpioE4::D4 only connects to Dcmi or Pssi!"); };
template<Peripheral p> struct SignalConnection<DataE4::Datin3, p> {
	static_assert((p == Peripheral::Dfsdm1),"GpioE4::Datin3 only connects to Dfsdm1!"); };
template<Peripheral p> struct SignalConnection<DataE4::Fsa, p> {
	static_assert((p == Peripheral::Sai1) or (p == Peripheral::Sai4),
		"GpioE4::Fsa only connects to Sai1 or Sai4!"); };
template<Peripheral p> struct SignalConnection<DataE4::Nss, p> {
	static_assert((p == Peripheral::Spi4),"GpioE4::Nss only connects to Spi4!"); };
template<Peripheral p> struct SignalConnection<DataE4::Traced1, p> {
	static_assert((p == Peripheral::Debug),"GpioE4::Traced1 only connects to Debug!"); };
template<> struct SignalConnection<DataE4::BitBang, Peripheral::BitBang> { static constexpr int8_t af = -1; };
template<> struct SignalConnection<DataE4::A20, Peripheral::Fmc> { static constexpr int8_t af = 12; };
template<> struct SignalConnection<DataE4::B0, Peripheral::Ltdc> { static constexpr int8_t af = 14; };
template<> struct SignalConnection<DataE4::Ch1n, Peripheral::Tim15> { static constexpr int8_t af = 4; };
template<> struct SignalConnection<DataE4::D2, Peripheral::Sai1> { static constexpr int8_t af = 2; };
template<> struct SignalConnection<DataE4::D2, Peripheral::Sai4> { static constexpr int8_t af = 10; };
template<> struct SignalConnection<DataE4::D4, Peripheral::Dcmi> { static constexpr int8_t af = 13; };
template<> struct SignalConnection<DataE4::D4, Peripheral::Pssi> { static constexpr int8_t af = 13; };
template<> struct SignalConnection<DataE4::Datin3, Peripheral::Dfsdm1> { static constexpr int8_t af = 3; };
template<> struct SignalConnection<DataE4::Fsa, Peripheral::Sai1> { static constexpr int8_t af = 6; };
template<> struct SignalConnection<DataE4::Fsa, Peripheral::Sai4> { static constexpr int8_t af = 8; };
template<> struct SignalConnection<DataE4::Nss, Peripheral::Spi4> { static constexpr int8_t af = 5; };
template<> struct SignalConnection<DataE4::Traced1, Peripheral::Debug> { static constexpr int8_t af = 0; };

struct DataE5 {
	static constexpr Gpio::Port port = Gpio::Port::E;
	static constexpr uint8_t pin = 5;
	struct BitBang { using Data = DataE5; static constexpr Gpio::Signal Signal = Gpio::Signal::BitBang; };
	struct A21 { using Data = DataE5; static constexpr Gpio::Signal Signal = Gpio::Signal::A21; };
	struct Ch1 { using Data = DataE5; static constexpr Gpio::Signal Signal = Gpio::Signal::Ch1; };
	struct Ck2 { using Data = DataE5; static constexpr Gpio::Signal Signal = Gpio::Signal::Ck2; };
	struct Ckin3 { using Data = DataE5; static constexpr Gpio::Signal Signal = Gpio::Signal::Ckin3; };
	struct D6 { using Data = DataE5; static constexpr Gpio::Signal Signal = Gpio::Signal::D6; };
	struct G0 { using Data = DataE5; static constexpr Gpio::Signal Signal = Gpio::Signal::G0; };
	struct Miso { using Data = DataE5; static constexpr Gpio::Signal Signal = Gpio::Signal::Miso; };
	struct Scka { using Data = DataE5; static constexpr Gpio::Signal Signal = Gpio::Signal::Scka; };
	struct Traced2 { using Data = DataE5; static constexpr Gpio::Signal Signal = Gpio::Signal::Traced2; };
};
template<Peripheral p> struct SignalConnection<DataE5::BitBang, p> {
	static_assert(p == Peripheral::BitBang, "GpioE5::BitBang only connects to software drivers!"); };
template<Peripheral p> struct SignalConnection<DataE5::A21, p> {
	static_assert((p == Peripheral::Fmc),"GpioE5::A21 only connects to Fmc!"); };
template<Peripheral p> struct SignalConnection<DataE5::Ch1, p> {
	static_assert((p == Peripheral::Tim15),"GpioE5::Ch1 only connects to Tim15!"); };
template<Peripheral p> struct SignalConnection<DataE5::Ck2, p> {
	static_assert((p == Peripheral::Sai1) or (p == Peripheral::Sai4),
		"GpioE5::Ck2 only connects to Sai1 or Sai4!"); };
template<Peripheral p> struct SignalConnection<DataE5::Ckin3, p> {
	static_assert((p == Peripheral::Dfsdm1),"GpioE5::Ckin3 only connects to Dfsdm1!"); };
template<Peripheral p> struct SignalConnection<DataE5::D6, p> {
	static_assert((p == Peripheral::Dcmi) or (p == Peripheral::Pssi),
		"GpioE5::D6 only connects to Dcmi or Pssi!"); };
template<Peripheral p> struct SignalConnection<DataE5::G0, p> {
	static_assert((p == Peripheral::Ltdc),"GpioE5::G0 only connects to Ltdc!"); };
template<Peripheral p> struct SignalConnection<DataE5::Miso, p> {
	static_assert((p == Peripheral::Spi4),"GpioE5::Miso only connects to Spi4!"); };
template<Peripheral p> struct SignalConnection<DataE5::Scka, p> {
	static_assert((p == Peripheral::Sai1) or (p == Peripheral::Sai4),
		"GpioE5::Scka only connects to Sai1 or Sai4!"); };
template<Peripheral p> struct SignalConnection<DataE5::Traced2, p> {
	static_assert((p == Peripheral::Debug),"GpioE5::Traced2 only connects to Debug!"); };
template<> struct SignalConnection<DataE5::BitBang, Peripheral::BitBang> { static constexpr int8_t af = -1; };
template<> struct SignalConnection<DataE5::A21, Peripheral::Fmc> { static constexpr int8_t af = 12; };
template<> struct SignalConnection<DataE5::Ch1, Peripheral::Tim15> { static constexpr int8_t af = 4; };
template<> struct SignalConnection<DataE5::Ck2, Peripheral::Sai1> { static constexpr int8_t af = 2; };
template<> struct SignalConnection<DataE5::Ck2, Peripheral::Sai4> { static constexpr int8_t af = 10; };
template<> struct SignalConnection<DataE5::Ckin3, Peripheral::Dfsdm1> { static constexpr int8_t af = 3; };
template<> struct SignalConnection<DataE5::D6, Peripheral::Dcmi> { static constexpr int8_t af = 13; };
template<> struct SignalConnection<DataE5::D6, Peripheral::Pssi> { static constexpr int8_t af = 13; };
template<> struct SignalConnection<DataE5::G0, Peripheral::Ltdc> { static constexpr int8_t af = 14; };
template<> struct SignalConnection<DataE5::Miso, Peripheral::Spi4> { static constexpr int8_t af = 5; };
template<> struct SignalConnection<DataE5::Scka, Peripheral::Sai1> { static constexpr int8_t af = 6; };
template<> struct SignalConnection<DataE5::Scka, Peripheral::Sai4> { static constexpr int8_t af = 8; };
template<> struct SignalConnection<DataE5::Traced2, Peripheral::Debug> { static constexpr int8_t af = 0; };

struct DataE6 {
	static constexpr Gpio::Port port = Gpio::Port::E;
	static constexpr uint8_t pin = 6;
	struct BitBang { using Data = DataE6; static constexpr Gpio::Signal Signal = Gpio::Signal::BitBang; };
	struct A22 { using Data = DataE6; static constexpr Gpio::Signal Signal = Gpio::Signal::A22; };
	struct Bkin2 { using Data = DataE6; static constexpr Gpio::Signal Signal = Gpio::Signal::Bkin2; };
	struct Bkin2comp1 { using Data = DataE6; static constexpr Gpio::Signal Signal = Gpio::Signal::Bkin2comp1; };
	struct Bkin2comp2 { using Data = DataE6; static constexpr Gpio::Signal Signal = Gpio::Signal::Bkin2comp2; };
	struct Ch2 { using Data = DataE6; static constexpr Gpio::Signal Signal = Gpio::Signal::Ch2; };
	struct D1 { using Data = DataE6; static constexpr Gpio::Signal Signal = Gpio::Signal::D1; };
	struct D7 { using Data = DataE6; static constexpr Gpio::Signal Signal = Gpio::Signal::D7; };
	struct G1 { using Data = DataE6; static constexpr Gpio::Signal Signal = Gpio::Signal::G1; };
	struct Mclkb { using Data = DataE6; static constexpr Gpio::Signal Signal = Gpio::Signal::Mclkb; };
	struct Mosi { using Data = DataE6; static constexpr Gpio::Signal Signal = Gpio::Signal::Mosi; };
	struct Sda { using Data = DataE6; static constexpr Gpio::Signal Signal = Gpio::Signal::Sda; };
	struct Traced3 { using Data = DataE6; static constexpr Gpio::Signal Signal = Gpio::Signal::Traced3; };
};
template<Peripheral p> struct SignalConnection<DataE6::BitBang, p> {
	static_assert(p == Peripheral::BitBang, "GpioE6::BitBang only connects to software drivers!"); };
template<Peripheral p> struct SignalConnection<DataE6::A22, p> {
	static_assert((p == Peripheral::Fmc),"GpioE6::A22 only connects to Fmc!"); };
template<Peripheral p> struct SignalConnection<DataE6::Bkin2, p> {
	static_assert((p == Peripheral::Tim1),"GpioE6::Bkin2 only connects to Tim1!"); };
template<Peripheral p> struct SignalConnection<DataE6::Bkin2comp1, p> {
	static_assert((p == Peripheral::Tim1),"GpioE6::Bkin2comp1 only connects to Tim1!"); };
template<Peripheral p> struct SignalConnection<DataE6::Bkin2comp2, p> {
	static_assert((p == Peripheral::Tim1),"GpioE6::Bkin2comp2 only connects to Tim1!"); };
template<Peripheral p> struct SignalConnection<DataE6::Ch2, p> {
	static_assert((p == Peripheral::Tim15),"GpioE6::Ch2 only connects to Tim15!"); };
template<Peripheral p> struct SignalConnection<DataE6::D1, p> {
	static_assert((p == Peripheral::Sai1) or (p == Peripheral::Sai4),
		"GpioE6::D1 only connects to Sai1 or Sai4!"); };
template<Peripheral p> struct SignalConnection<DataE6::D7, p> {
	static_assert((p == Peripheral::Dcmi) or (p == Peripheral::Pssi),
		"GpioE6::D7 only connects to Dcmi or Pssi!"); };
template<Peripheral p> struct SignalConnection<DataE6::G1, p> {
	static_assert((p == Peripheral::Ltdc),"GpioE6::G1 only connects to Ltdc!"); };
template<Peripheral p> struct SignalConnection<DataE6::Mclkb, p> {
	static_assert((p == Peripheral::Sai4),"GpioE6::Mclkb only connects to Sai4!"); };
template<Peripheral p> struct SignalConnection<DataE6::Mosi, p> {
	static_assert((p == Peripheral::Spi4),"GpioE6::Mosi only connects to Spi4!"); };
template<Peripheral p> struct SignalConnection<DataE6::Sda, p> {
	static_assert((p == Peripheral::Sai1) or (p == Peripheral::Sai4),
		"GpioE6::Sda only connects to Sai1 or Sai4!"); };
template<Peripheral p> struct SignalConnection<DataE6::Traced3, p> {
	static_assert((p == Peripheral::Debug),"GpioE6::Traced3 only connects to Debug!"); };
template<> struct SignalConnection<DataE6::BitBang, Peripheral::BitBang> { static constexpr int8_t af = -1; };
template<> struct SignalConnection<DataE6::A22, Peripheral::Fmc> { static constexpr int8_t af = 12; };
template<> struct SignalConnection<DataE6::Bkin2, Peripheral::Tim1> { static constexpr int8_t af = 1; };
template<> struct SignalConnection<DataE6::Bkin2comp1, Peripheral::Tim1> { static constexpr int8_t af = 11; };
template<> struct SignalConnection<DataE6::Bkin2comp2, Peripheral::Tim1> { static constexpr int8_t af = 11; };
template<> struct SignalConnection<DataE6::Ch2, Peripheral::Tim15> { static constexpr int8_t af = 4; };
template<> struct SignalConnection<DataE6::D1, Peripheral::Sai1> { static constexpr int8_t af = 2; };
template<> struct SignalConnection<DataE6::D1, Peripheral::Sai4> { static constexpr int8_t af = 9; };
template<> struct SignalConnection<DataE6::D7, Peripheral::Dcmi> { static constexpr int8_t af = 13; };
template<> struct SignalConnection<DataE6::D7, Peripheral::Pssi> { static constexpr int8_t af = 13; };
template<> struct SignalConnection<DataE6::G1, Peripheral::Ltdc> { static constexpr int8_t af = 14; };
template<> struct SignalConnection<DataE6::Mclkb, Peripheral::Sai4> { static constexpr int8_t af = 10; };
template<> struct SignalConnection<DataE6::Mosi, Peripheral::Spi4> { static constexpr int8_t af = 5; };
template<> struct SignalConnection<DataE6::Sda, Peripheral::Sai1> { static constexpr int8_t af = 6; };
template<> struct SignalConnection<DataE6::Sda, Peripheral::Sai4> { static constexpr int8_t af = 8; };
template<> struct SignalConnection<DataE6::Traced3, Peripheral::Debug> { static constexpr int8_t af = 0; };

struct DataE7 {
	static constexpr Gpio::Port port = Gpio::Port::E;
	static constexpr uint8_t pin = 7;
	struct BitBang { using Data = DataE7; static constexpr Gpio::Signal Signal = Gpio::Signal::BitBang; };
	struct D4 { using Data = DataE7; static constexpr Gpio::Signal Signal = Gpio::Signal::D4; };
	struct Da4 { using Data = DataE7; static constexpr Gpio::Signal Signal = Gpio::Signal::Da4; };
	struct Datin2 { using Data = DataE7; static constexpr Gpio::Signal Signal = Gpio::Signal::Datin2; };
	struct Etr { using Data = DataE7; static constexpr Gpio::Signal Signal = Gpio::Signal::Etr; };
	struct Inm { using Data = DataE7; static constexpr Gpio::Signal Signal = Gpio::Signal::Inm; };
	struct P1io4 { using Data = DataE7; static constexpr Gpio::Signal Signal = Gpio::Signal::P1io4; };
	struct Rx { using Data = DataE7; static constexpr Gpio::Signal Signal = Gpio::Signal::Rx; };
	struct Vout { using Data = DataE7; static constexpr Gpio::Signal Signal = Gpio::Signal::Vout; };
};
template<Peripheral p> struct SignalConnection<DataE7::BitBang, p> {
	static_assert(p == Peripheral::BitBang, "GpioE7::BitBang only connects to software drivers!"); };
template<Peripheral p> struct SignalConnection<DataE7::D4, p> {
	static_assert((p == Peripheral::Fmc),"GpioE7::D4 only connects to Fmc!"); };
template<Peripheral p> struct SignalConnection<DataE7::Da4, p> {
	static_assert((p == Peripheral::Fmc),"GpioE7::Da4 only connects to Fmc!"); };
template<Peripheral p> struct SignalConnection<DataE7::Datin2, p> {
	static_assert((p == Peripheral::Dfsdm1),"GpioE7::Datin2 only connects to Dfsdm1!"); };
template<Peripheral p> struct SignalConnection<DataE7::Etr, p> {
	static_assert((p == Peripheral::Tim1),"GpioE7::Etr only connects to Tim1!"); };
template<Peripheral p> struct SignalConnection<DataE7::Inm, p> {
	static_assert((p == Peripheral::Comp2),"GpioE7::Inm only connects to Comp2!"); };
template<Peripheral p> struct SignalConnection<DataE7::P1io4, p> {
	static_assert((p == Peripheral::Octospim),"GpioE7::P1io4 only connects to Octospim!"); };
template<Peripheral p> struct SignalConnection<DataE7::Rx, p> {
	static_assert((p == Peripheral::Uart7),"GpioE7::Rx only connects to Uart7!"); };
template<Peripheral p> struct SignalConnection<DataE7::Vout, p> {
	static_assert((p == Peripheral::Opamp2),"GpioE7::Vout only connects to Opamp2!"); };
template<> struct SignalConnection<DataE7::BitBang, Peripheral::BitBang> { static constexpr int8_t af = -1; };
template<> struct SignalConnection<DataE7::D4, Peripheral::Fmc> { static constexpr int8_t af = 12; };
template<> struct SignalConnection<DataE7::Da4, Peripheral::Fmc> { static constexpr int8_t af = 12; };
template<> struct SignalConnection<DataE7::Datin2, Peripheral::Dfsdm1> { static constexpr int8_t af = 3; };
template<> struct SignalConnection<DataE7::Etr, Peripheral::Tim1> { static constexpr int8_t af = 1; };
template<> struct SignalConnection<DataE7::Inm, Peripheral::Comp2> { static constexpr int8_t af = -2; };
template<> struct SignalConnection<DataE7::P1io4, Peripheral::Octospim> { static constexpr int8_t af = 10; };
template<> struct SignalConnection<DataE7::Rx, Peripheral::Uart7> { static constexpr int8_t af = 7; };
template<> struct SignalConnection<DataE7::Vout, Peripheral::Opamp2> { static constexpr int8_t af = -1; };

struct DataE8 {
	static constexpr Gpio::Port port = Gpio::Port::E;
	static constexpr uint8_t pin = 8;
	struct BitBang { using Data = DataE8; static constexpr Gpio::Signal Signal = Gpio::Signal::BitBang; };
	struct Ch1n { using Data = DataE8; static constexpr Gpio::Signal Signal = Gpio::Signal::Ch1n; };
	struct Ckin2 { using Data = DataE8; static constexpr Gpio::Signal Signal = Gpio::Signal::Ckin2; };
	struct D5 { using Data = DataE8; static constexpr Gpio::Signal Signal = Gpio::Signal::D5; };
	struct Da5 { using Data = DataE8; static constexpr Gpio::Signal Signal = Gpio::Signal::Da5; };
	struct Out { using Data = DataE8; static constexpr Gpio::Signal Signal = Gpio::Signal::Out; };
	struct P1io5 { using Data = DataE8; static constexpr Gpio::Signal Signal = Gpio::Signal::P1io5; };
	struct Tx { using Data = DataE8; static constexpr Gpio::Signal Signal = Gpio::Signal::Tx; };
	struct Vinm { using Data = DataE8; static constexpr Gpio::Signal Signal = Gpio::Signal::Vinm; };
};
template<Peripheral p> struct SignalConnection<DataE8::BitBang, p> {
	static_assert(p == Peripheral::BitBang, "GpioE8::BitBang only connects to software drivers!"); };
template<Peripheral p> struct SignalConnection<DataE8::Ch1n, p> {
	static_assert((p == Peripheral::Tim1),"GpioE8::Ch1n only connects to Tim1!"); };
template<Peripheral p> struct SignalConnection<DataE8::Ckin2, p> {
	static_assert((p == Peripheral::Dfsdm1),"GpioE8::Ckin2 only connects to Dfsdm1!"); };
template<Peripheral p> struct SignalConnection<DataE8::D5, p> {
	static_assert((p == Peripheral::Fmc),"GpioE8::D5 only connects to Fmc!"); };
template<Peripheral p> struct SignalConnection<DataE8::Da5, p> {
	static_assert((p == Peripheral::Fmc),"GpioE8::Da5 only connects to Fmc!"); };
template<Peripheral p> struct SignalConnection<DataE8::Out, p> {
	static_assert((p == Peripheral::Comp2),"GpioE8::Out only connects to Comp2!"); };
template<Peripheral p> struct SignalConnection<DataE8::P1io5, p> {
	static_assert((p == Peripheral::Octospim),"GpioE8::P1io5 only connects to Octospim!"); };
template<Peripheral p> struct SignalConnection<DataE8::Tx, p> {
	static_assert((p == Peripheral::Uart7),"GpioE8::Tx only connects to Uart7!"); };
template<Peripheral p> struct SignalConnection<DataE8::Vinm, p> {
	static_assert((p == Peripheral::Opamp2),"GpioE8::Vinm only connects to Opamp2!"); };
template<> struct SignalConnection<DataE8::BitBang, Peripheral::BitBang> { static constexpr int8_t af = -1; };
template<> struct SignalConnection<DataE8::Ch1n, Peripheral::Tim1> { static constexpr int8_t af = 1; };
template<> struct SignalConnection<DataE8::Ckin2, Peripheral::Dfsdm1> { static constexpr int8_t af = 3; };
template<> struct SignalConnection<DataE8::D5, Peripheral::Fmc> { static constexpr int8_t af = 12; };
template<> struct SignalConnection<DataE8::Da5, Peripheral::Fmc> { static constexpr int8_t af = 12; };
template<> struct SignalConnection<DataE8::Out, Peripheral::Comp2> { static constexpr int8_t af = 13; };
template<> struct SignalConnection<DataE8::P1io5, Peripheral::Octospim> { static constexpr int8_t af = 10; };
template<> struct SignalConnection<DataE8::Tx, Peripheral::Uart7> { static constexpr int8_t af = 7; };
template<> struct SignalConnection<DataE8::Vinm, Peripheral::Opamp2> { static constexpr int8_t af = -1; };

struct DataE9 {
	static constexpr Gpio::Port port = Gpio::Port::E;
	static constexpr uint8_t pin = 9;
	struct BitBang { using Data = DataE9; static constexpr Gpio::Signal Signal = Gpio::Signal::BitBang; };
	struct Ch1 { using Data = DataE9; static constexpr Gpio::Signal Signal = Gpio::Signal::Ch1; };
	struct Ckout { using Data = DataE9; static constexpr Gpio::Signal Signal = Gpio::Signal::Ckout; };
	struct D6 { using Data = DataE9; static constexpr Gpio::Signal Signal = Gpio::Signal::D6; };
	struct Da6 { using Data = DataE9; static constexpr Gpio::Signal Signal = Gpio::Signal::Da6; };
	struct De { using Data = DataE9; static constexpr Gpio::Signal Signal = Gpio::Signal::De; };
	struct Inp { using Data = DataE9; static constexpr Gpio::Signal Signal = Gpio::Signal::Inp; };
	struct P1io6 { using Data = DataE9; static constexpr Gpio::Signal Signal = Gpio::Signal::P1io6; };
	struct Rts { using Data = DataE9; static constexpr Gpio::Signal Signal = Gpio::Signal::Rts; };
	struct Vinp { using Data = DataE9; static constexpr Gpio::Signal Signal = Gpio::Signal::Vinp; };
};
template<Peripheral p> struct SignalConnection<DataE9::BitBang, p> {
	static_assert(p == Peripheral::BitBang, "GpioE9::BitBang only connects to software drivers!"); };
template<Peripheral p> struct SignalConnection<DataE9::Ch1, p> {
	static_assert((p == Peripheral::Tim1),"GpioE9::Ch1 only connects to Tim1!"); };
template<Peripheral p> struct SignalConnection<DataE9::Ckout, p> {
	static_assert((p == Peripheral::Dfsdm1),"GpioE9::Ckout only connects to Dfsdm1!"); };
template<Peripheral p> struct SignalConnection<DataE9::D6, p> {
	static_assert((p == Peripheral::Fmc),"GpioE9::D6 only connects to Fmc!"); };
template<Peripheral p> struct SignalConnection<DataE9::Da6, p> {
	static_assert((p == Peripheral::Fmc),"GpioE9::Da6 only connects to Fmc!"); };
template<Peripheral p> struct SignalConnection<DataE9::De, p> {
	static_assert((p == Peripheral::Uart7),"GpioE9::De only connects to Uart7!"); };
template<Peripheral p> struct SignalConnection<DataE9::Inp, p> {
	static_assert((p == Peripheral::Comp2),"GpioE9::Inp only connects to Comp2!"); };
template<Peripheral p> struct SignalConnection<DataE9::P1io6, p> {
	static_assert((p == Peripheral::Octospim),"GpioE9::P1io6 only connects to Octospim!"); };
template<Peripheral p> struct SignalConnection<DataE9::Rts, p> {
	static_assert((p == Peripheral::Uart7),"GpioE9::Rts only connects to Uart7!"); };
template<Peripheral p> struct SignalConnection<DataE9::Vinp, p> {
	static_assert((p == Peripheral::Opamp2),"GpioE9::Vinp only connects to Opamp2!"); };
template<> struct SignalConnection<DataE9::BitBang, Peripheral::BitBang> { static constexpr int8_t af = -1; };
template<> struct SignalConnection<DataE9::Ch1, Peripheral::Tim1> { static constexpr int8_t af = 1; };
template<> struct SignalConnection<DataE9::Ckout, Peripheral::Dfsdm1> { static constexpr int8_t af = 3; };
template<> struct SignalConnection<DataE9::D6, Peripheral::Fmc> { static constexpr int8_t af = 12; };
template<> struct SignalConnection<DataE9::Da6, Peripheral::Fmc> { static constexpr int8_t af = 12; };
template<> struct SignalConnection<DataE9::De, Peripheral::Uart7> { static constexpr int8_t af = 7; };
template<> struct SignalConnection<DataE9::Inp, Peripheral::Comp2> { static constexpr int8_t af = -2; };
template<> struct SignalConnection<DataE9::P1io6, Peripheral::Octospim> { static constexpr int8_t af = 10; };
template<> struct SignalConnection<DataE9::Rts, Peripheral::Uart7> { static constexpr int8_t af = 7; };
template<> struct SignalConnection<DataE9::Vinp, Peripheral::Opamp2> { static constexpr int8_t af = -1; };

struct DataE10 {
	static constexpr Gpio::Port port = Gpio::Port::E;
	static constexpr uint8_t pin = 10;
	struct BitBang { using Data = DataE10; static constexpr Gpio::Signal Signal = Gpio::Signal::BitBang; };
	struct Ch2n { using Data = DataE10; static constexpr Gpio::Signal Signal = Gpio::Signal::Ch2n; };
	struct Cts { using Data = DataE10; static constexpr Gpio::Signal Signal = Gpio::Signal::Cts; };
	struct D7 { using Data = DataE10; static constexpr Gpio::Signal Signal = Gpio::Signal::D7; };
	struct Da7 { using Data = DataE10; static constexpr Gpio::Signal Signal = Gpio::Signal::Da7; };
	struct Datin4 { using Data = DataE10; static constexpr Gpio::Signal Signal = Gpio::Signal::Datin4; };
	struct Inm { using Data = DataE10; static constexpr Gpio::Signal Signal = Gpio::Signal::Inm; };
	struct P1io7 { using Data = DataE10; static constexpr Gpio::Signal Signal = Gpio::Signal::P1io7; };
};
template<Peripheral p> struct SignalConnection<DataE10::BitBang, p> {
	static_assert(p == Peripheral::BitBang, "GpioE10::BitBang only connects to software drivers!"); };
template<Peripheral p> struct SignalConnection<DataE10::Ch2n, p> {
	static_assert((p == Peripheral::Tim1),"GpioE10::Ch2n only connects to Tim1!"); };
template<Peripheral p> struct SignalConnection<DataE10::Cts, p> {
	static_assert((p == Peripheral::Uart7),"GpioE10::Cts only connects to Uart7!"); };
template<Peripheral p> struct SignalConnection<DataE10::D7, p> {
	static_assert((p == Peripheral::Fmc),"GpioE10::D7 only connects to Fmc!"); };
template<Peripheral p> struct SignalConnection<DataE10::Da7, p> {
	static_assert((p == Peripheral::Fmc),"GpioE10::Da7 only connects to Fmc!"); };
template<Peripheral p> struct SignalConnection<DataE10::Datin4, p> {
	static_assert((p == Peripheral::Dfsdm1),"GpioE10::Datin4 only connects to Dfsdm1!"); };
template<Peripheral p> struct SignalConnection<DataE10::Inm, p> {
	static_assert((p == Peripheral::Comp2),"GpioE10::Inm only connects to Comp2!"); };
template<Peripheral p> struct SignalConnection<DataE10::P1io7, p> {
	static_assert((p == Peripheral::Octospim),"GpioE10::P1io7 only connects to Octospim!"); };
template<> struct SignalConnection<DataE10::BitBang, Peripheral::BitBang> { static constexpr int8_t af = -1; };
template<> struct SignalConnection<DataE10::Ch2n, Peripheral::Tim1> { static constexpr int8_t af = 1; };
template<> struct SignalConnection<DataE10::Cts, Peripheral::Uart7> { static constexpr int8_t af = 7; };
template<> struct SignalConnection<DataE10::D7, Peripheral::Fmc> { static constexpr int8_t af = 12; };
template<> struct SignalConnection<DataE10::Da7, Peripheral::Fmc> { static constexpr int8_t af = 12; };
template<> struct SignalConnection<DataE10::Datin4, Peripheral::Dfsdm1> { static constexpr int8_t af = 3; };
template<> struct SignalConnection<DataE10::Inm, Peripheral::Comp2> { static constexpr int8_t af = -2; };
template<> struct SignalConnection<DataE10::P1io7, Peripheral::Octospim> { static constexpr int8_t af = 10; };

struct DataE11 {
	static constexpr Gpio::Port port = Gpio::Port::E;
	static constexpr uint8_t pin = 11;
	struct BitBang { using Data = DataE11; static constexpr Gpio::Signal Signal = Gpio::Signal::BitBang; };
	struct Ch2 { using Data = DataE11; static constexpr Gpio::Signal Signal = Gpio::Signal::Ch2; };
	struct Ckin4 { using Data = DataE11; static constexpr Gpio::Signal Signal = Gpio::Signal::Ckin4; };
	struct D8 { using Data = DataE11; static constexpr Gpio::Signal Signal = Gpio::Signal::D8; };
	struct Da8 { using Data = DataE11; static constexpr Gpio::Signal Signal = Gpio::Signal::Da8; };
	struct G3 { using Data = DataE11; static constexpr Gpio::Signal Signal = Gpio::Signal::G3; };
	struct Inp { using Data = DataE11; static constexpr Gpio::Signal Signal = Gpio::Signal::Inp; };
	struct Nss { using Data = DataE11; static constexpr Gpio::Signal Signal = Gpio::Signal::Nss; };
	struct P1ncs { using Data = DataE11; static constexpr Gpio::Signal Signal = Gpio::Signal::P1ncs; };
	struct Sdb { using Data = DataE11; static constexpr Gpio::Signal Signal = Gpio::Signal::Sdb; };
};
template<Peripheral p> struct SignalConnection<DataE11::BitBang, p> {
	static_assert(p == Peripheral::BitBang, "GpioE11::BitBang only connects to software drivers!"); };
template<Peripheral p> struct SignalConnection<DataE11::Ch2, p> {
	static_assert((p == Peripheral::Tim1),"GpioE11::Ch2 only connects to Tim1!"); };
template<Peripheral p> struct SignalConnection<DataE11::Ckin4, p> {
	static_assert((p == Peripheral::Dfsdm1),"GpioE11::Ckin4 only connects to Dfsdm1!"); };
template<Peripheral p> struct SignalConnection<DataE11::D8, p> {
	static_assert((p == Peripheral::Fmc),"GpioE11::D8 only connects to Fmc!"); };
template<Peripheral p> struct SignalConnection<DataE11::Da8, p> {
	static_assert((p == Peripheral::Fmc),"GpioE11::Da8 only connects to Fmc!"); };
template<Peripheral p> struct SignalConnection<DataE11::G3, p> {
	static_assert((p == Peripheral::Ltdc),"GpioE11::G3 only connects to Ltdc!"); };
template<Peripheral p> struct SignalConnection<DataE11::Inp, p> {
	static_assert((p == Peripheral::Comp2),"GpioE11::Inp only connects to Comp2!"); };
template<Peripheral p> struct SignalConnection<DataE11::Nss, p> {
	static_assert((p == Peripheral::Spi4),"GpioE11::Nss only connects to Spi4!"); };
template<Peripheral p> struct SignalConnection<DataE11::P1ncs, p> {
	static_assert((p == Peripheral::Octospim),"GpioE11::P1ncs only connects to Octospim!"); };
template<Peripheral p> struct SignalConnection<DataE11::Sdb, p> {
	static_assert((p == Peripheral::Sai4),"GpioE11::Sdb only connects to Sai4!"); };
template<> struct SignalConnection<DataE11::BitBang, Peripheral::BitBang> { static constexpr int8_t af = -1; };
template<> struct SignalConnection<DataE11::Ch2, Peripheral::Tim1> { static constexpr int8_t af = 1; };
template<> struct SignalConnection<DataE11::Ckin4, Peripheral::Dfsdm1> { static constexpr int8_t af = 3; };
template<> struct SignalConnection<DataE11::D8, Peripheral::Fmc> { static constexpr int8_t af = 12; };
template<> struct SignalConnection<DataE11::Da8, Peripheral::Fmc> { static constexpr int8_t af = 12; };
template<> struct SignalConnection<DataE11::G3, Peripheral::Ltdc> { static constexpr int8_t af = 14; };
template<> struct SignalConnection<DataE11::Inp, Peripheral::Comp2> { static constexpr int8_t af = -2; };
template<> struct SignalConnection<DataE11::Nss, Peripheral::Spi4> { static constexpr int8_t af = 5; };
template<> struct SignalConnection<DataE11::P1ncs, Peripheral::Octospim> { static constexpr int8_t af = 11; };
template<> struct SignalConnection<DataE11::Sdb, Peripheral::Sai4> { static constexpr int8_t af = 10; };

struct DataE12 {
	static constexpr Gpio::Port port = Gpio::Port::E;
	static constexpr uint8_t pin = 12;
	struct BitBang { using Data = DataE12; static constexpr Gpio::Signal Signal = Gpio::Signal::BitBang; };
	struct B4 { using Data = DataE12; static constexpr Gpio::Signal Signal = Gpio::Signal::B4; };
	struct Ch3n { using Data = DataE12; static constexpr Gpio::Signal Signal = Gpio::Signal::Ch3n; };
	struct D9 { using Data = DataE12; static constexpr Gpio::Signal Signal = Gpio::Signal::D9; };
	struct Da9 { using Data = DataE12; static constexpr Gpio::Signal Signal = Gpio::Signal::Da9; };
	struct Datin5 { using Data = DataE12; static constexpr Gpio::Signal Signal = Gpio::Signal::Datin5; };
	struct Out { using Data = DataE12; static constexpr Gpio::Signal Signal = Gpio::Signal::Out; };
	struct Sck { using Data = DataE12; static constexpr Gpio::Signal Signal = Gpio::Signal::Sck; };
	struct Sckb { using Data = DataE12; static constexpr Gpio::Signal Signal = Gpio::Signal::Sckb; };
};
template<Peripheral p> struct SignalConnection<DataE12::BitBang, p> {
	static_assert(p == Peripheral::BitBang, "GpioE12::BitBang only connects to software drivers!"); };
template<Peripheral p> struct SignalConnection<DataE12::B4, p> {
	static_assert((p == Peripheral::Ltdc),"GpioE12::B4 only connects to Ltdc!"); };
template<Peripheral p> struct SignalConnection<DataE12::Ch3n, p> {
	static_assert((p == Peripheral::Tim1),"GpioE12::Ch3n only connects to Tim1!"); };
template<Peripheral p> struct SignalConnection<DataE12::D9, p> {
	static_assert((p == Peripheral::Fmc),"GpioE12::D9 only connects to Fmc!"); };
template<Peripheral p> struct SignalConnection<DataE12::Da9, p> {
	static_assert((p == Peripheral::Fmc),"GpioE12::Da9 only connects to Fmc!"); };
template<Peripheral p> struct SignalConnection<DataE12::Datin5, p> {
	static_assert((p == Peripheral::Dfsdm1),"GpioE12::Datin5 only connects to Dfsdm1!"); };
template<Peripheral p> struct SignalConnection<DataE12::Out, p> {
	static_assert((p == Peripheral::Comp1),"GpioE12::Out only connects to Comp1!"); };
template<Peripheral p> struct SignalConnection<DataE12::Sck, p> {
	static_assert((p == Peripheral::Spi4),"GpioE12::Sck only connects to Spi4!"); };
template<Peripheral p> struct SignalConnection<DataE12::Sckb, p> {
	static_assert((p == Peripheral::Sai4),"GpioE12::Sckb only connects to Sai4!"); };
template<> struct SignalConnection<DataE12::BitBang, Peripheral::BitBang> { static constexpr int8_t af = -1; };
template<> struct SignalConnection<DataE12::B4, Peripheral::Ltdc> { static constexpr int8_t af = 14; };
template<> struct SignalConnection<DataE12::Ch3n, Peripheral::Tim1> { static constexpr int8_t af = 1; };
template<> struct SignalConnection<DataE12::D9, Peripheral::Fmc> { static constexpr int8_t af = 12; };
template<> struct SignalConnection<DataE12::Da9, Peripheral::Fmc> { static constexpr int8_t af = 12; };
template<> struct SignalConnection<DataE12::Datin5, Peripheral::Dfsdm1> { static constexpr int8_t af = 3; };
template<> struct SignalConnection<DataE12::Out, Peripheral::Comp1> { static constexpr int8_t af = 13; };
template<> struct SignalConnection<DataE12::Sck, Peripheral::Spi4> { static constexpr int8_t af = 5; };
template<> struct SignalConnection<DataE12::Sckb, Peripheral::Sai4> { static constexpr int8_t af = 10; };

struct DataE13 {
	static constexpr Gpio::Port port = Gpio::Port::E;
	static constexpr uint8_t pin = 13;
	struct BitBang { using Data = DataE13; static constexpr Gpio::Signal Signal = Gpio::Signal::BitBang; };
	struct Ch3 { using Data = DataE13; static constexpr Gpio::Signal Signal = Gpio::Signal::Ch3; };
	struct Ckin5 { using Data = DataE13; static constexpr Gpio::Signal Signal = Gpio::Signal::Ckin5; };
	struct D10 { using Data = DataE13; static constexpr Gpio::Signal Signal = Gpio::Signal::D10; };
	struct Da10 { using Data = DataE13; static constexpr Gpio::Signal Signal = Gpio::Signal::Da10; };
	struct De { using Data = DataE13; static constexpr Gpio::Signal Signal = Gpio::Signal::De; };
	struct Fsb { using Data = DataE13; static constexpr Gpio::Signal Signal = Gpio::Signal::Fsb; };
	struct Miso { using Data = DataE13; static constexpr Gpio::Signal Signal = Gpio::Signal::Miso; };
	struct Out { using Data = DataE13; static constexpr Gpio::Signal Signal = Gpio::Signal::Out; };
};
template<Peripheral p> struct SignalConnection<DataE13::BitBang, p> {
	static_assert(p == Peripheral::BitBang, "GpioE13::BitBang only connects to software drivers!"); };
template<Peripheral p> struct SignalConnection<DataE13::Ch3, p> {
	static_assert((p == Peripheral::Tim1),"GpioE13::Ch3 only connects to Tim1!"); };
template<Peripheral p> struct SignalConnection<DataE13::Ckin5, p> {
	static_assert((p == Peripheral::Dfsdm1),"GpioE13::Ckin5 only connects to Dfsdm1!"); };
template<Peripheral p> struct SignalConnection<DataE13::D10, p> {
	static_assert((p == Peripheral::Fmc),"GpioE13::D10 only connects to Fmc!"); };
template<Peripheral p> struct SignalConnection<DataE13::Da10, p> {
	static_assert((p == Peripheral::Fmc),"GpioE13::Da10 only connects to Fmc!"); };
template<Peripheral p> struct SignalConnection<DataE13::De, p> {
	static_assert((p == Peripheral::Ltdc),"GpioE13::De only connects to Ltdc!"); };
template<Peripheral p> struct SignalConnection<DataE13::Fsb, p> {
	static_assert((p == Peripheral::Sai4),"GpioE13::Fsb only connects to Sai4!"); };
template<Peripheral p> struct SignalConnection<DataE13::Miso, p> {
	static_assert((p == Peripheral::Spi4),"GpioE13::Miso only connects to Spi4!"); };
template<Peripheral p> struct SignalConnection<DataE13::Out, p> {
	static_assert((p == Peripheral::Comp2),"GpioE13::Out only connects to Comp2!"); };
template<> struct SignalConnection<DataE13::BitBang, Peripheral::BitBang> { static constexpr int8_t af = -1; };
template<> struct SignalConnection<DataE13::Ch3, Peripheral::Tim1> { static constexpr int8_t af = 1; };
template<> struct SignalConnection<DataE13::Ckin5, Peripheral::Dfsdm1> { static constexpr int8_t af = 3; };
template<> struct SignalConnection<DataE13::D10, Peripheral::Fmc> { static constexpr int8_t af = 12; };
template<> struct SignalConnection<DataE13::Da10, Peripheral::Fmc> { static constexpr int8_t af = 12; };
template<> struct SignalConnection<DataE13::De, Peripheral::Ltdc> { static constexpr int8_t af = 14; };
template<> struct SignalConnection<DataE13::Fsb, Peripheral::Sai4> { static constexpr int8_t af = 10; };
template<> struct SignalConnection<DataE13::Miso, Peripheral::Spi4> { static constexpr int8_t af = 5; };
template<> struct SignalConnection<DataE13::Out, Peripheral::Comp2> { static constexpr int8_t af = 13; };

struct DataE14 {
	static constexpr Gpio::Port port = Gpio::Port::E;
	static constexpr uint8_t pin = 14;
	struct BitBang { using Data = DataE14; static constexpr Gpio::Signal Signal = Gpio::Signal::BitBang; };
	struct Ch4 { using Data = DataE14; static constexpr Gpio::Signal Signal = Gpio::Signal::Ch4; };
	struct Clk { using Data = DataE14; static constexpr Gpio::Signal Signal = Gpio::Signal::Clk; };
	struct D11 { using Data = DataE14; static constexpr Gpio::Signal Signal = Gpio::Signal::D11; };
	struct Da11 { using Data = DataE14; static constexpr Gpio::Signal Signal = Gpio::Signal::Da11; };
	struct Mclkb { using Data = DataE14; static constexpr Gpio::Signal Signal = Gpio::Signal::Mclkb; };
	struct Mosi { using Data = DataE14; static constexpr Gpio::Signal Signal = Gpio::Signal::Mosi; };
};
template<Peripheral p> struct SignalConnection<DataE14::BitBang, p> {
	static_assert(p == Peripheral::BitBang, "GpioE14::BitBang only connects to software drivers!"); };
template<Peripheral p> struct SignalConnection<DataE14::Ch4, p> {
	static_assert((p == Peripheral::Tim1),"GpioE14::Ch4 only connects to Tim1!"); };
template<Peripheral p> struct SignalConnection<DataE14::Clk, p> {
	static_assert((p == Peripheral::Ltdc),"GpioE14::Clk only connects to Ltdc!"); };
template<Peripheral p> struct SignalConnection<DataE14::D11, p> {
	static_assert((p == Peripheral::Fmc),"GpioE14::D11 only connects to Fmc!"); };
template<Peripheral p> struct SignalConnection<DataE14::Da11, p> {
	static_assert((p == Peripheral::Fmc),"GpioE14::Da11 only connects to Fmc!"); };
template<Peripheral p> struct SignalConnection<DataE14::Mclkb, p> {
	static_assert((p == Peripheral::Sai4),"GpioE14::Mclkb only connects to Sai4!"); };
template<Peripheral p> struct SignalConnection<DataE14::Mosi, p> {
	static_assert((p == Peripheral::Spi4),"GpioE14::Mosi only connects to Spi4!"); };
template<> struct SignalConnection<DataE14::BitBang, Peripheral::BitBang> { static constexpr int8_t af = -1; };
template<> struct SignalConnection<DataE14::Ch4, Peripheral::Tim1> { static constexpr int8_t af = 1; };
template<> struct SignalConnection<DataE14::Clk, Peripheral::Ltdc> { static constexpr int8_t af = 14; };
template<> struct SignalConnection<DataE14::D11, Peripheral::Fmc> { static constexpr int8_t af = 12; };
template<> struct SignalConnection<DataE14::Da11, Peripheral::Fmc> { static constexpr int8_t af = 12; };
template<> struct SignalConnection<DataE14::Mclkb, Peripheral::Sai4> { static constexpr int8_t af = 10; };
template<> struct SignalConnection<DataE14::Mosi, Peripheral::Spi4> { static constexpr int8_t af = 5; };

struct DataE15 {
	static constexpr Gpio::Port port = Gpio::Port::E;
	static constexpr uint8_t pin = 15;
	struct BitBang { using Data = DataE15; static constexpr Gpio::Signal Signal = Gpio::Signal::BitBang; };
	struct Bkin { using Data = DataE15; static constexpr Gpio::Signal Signal = Gpio::Signal::Bkin; };
	struct Bkincomp1 { using Data = DataE15; static constexpr Gpio::Signal Signal = Gpio::Signal::Bkincomp1; };
	struct Bkincomp2 { using Data = DataE15; static constexpr Gpio::Signal Signal = Gpio::Signal::Bkincomp2; };
	struct Ck { using Data = DataE15; static constexpr Gpio::Signal Signal = Gpio::Signal::Ck; };
	struct D12 { using Data = DataE15; static constexpr Gpio::Signal Signal = Gpio::Signal::D12; };
	struct Da12 { using Data = DataE15; static constexpr Gpio::Signal Signal = Gpio::Signal::Da12; };
	struct R7 { using Data = DataE15; static constexpr Gpio::Signal Signal = Gpio::Signal::R7; };
};
template<Peripheral p> struct SignalConnection<DataE15::BitBang, p> {
	static_assert(p == Peripheral::BitBang, "GpioE15::BitBang only connects to software drivers!"); };
template<Peripheral p> struct SignalConnection<DataE15::Bkin, p> {
	static_assert((p == Peripheral::Tim1),"GpioE15::Bkin only connects to Tim1!"); };
template<Peripheral p> struct SignalConnection<DataE15::Bkincomp1, p> {
	static_assert((p == Peripheral::Tim1),"GpioE15::Bkincomp1 only connects to Tim1!"); };
template<Peripheral p> struct SignalConnection<DataE15::Bkincomp2, p> {
	static_assert((p == Peripheral::Tim1),"GpioE15::Bkincomp2 only connects to Tim1!"); };
template<Peripheral p> struct SignalConnection<DataE15::Ck, p> {
	static_assert((p == Peripheral::Usart10),"GpioE15::Ck only connects to Usart10!"); };
template<Peripheral p> struct SignalConnection<DataE15::D12, p> {
	static_assert((p == Peripheral::Fmc),"GpioE15::D12 only connects to Fmc!"); };
template<Peripheral p> struct SignalConnection<DataE15::Da12, p> {
	static_assert((p == Peripheral::Fmc),"GpioE15::Da12 only connects to Fmc!"); };
template<Peripheral p> struct SignalConnection<DataE15::R7, p> {
	static_assert((p == Peripheral::Ltdc),"GpioE15::R7 only connects to Ltdc!"); };
template<> struct SignalConnection<DataE15::BitBang, Peripheral::BitBang> { static constexpr int8_t af = -1; };
template<> struct SignalConnection<DataE15::Bkin, Peripheral::Tim1> { static constexpr int8_t af = 1; };
template<> struct SignalConnection<DataE15::Bkincomp1, Peripheral::Tim1> { static constexpr int8_t af = 13; };
template<> struct SignalConnection<DataE15::Bkincomp2, Peripheral::Tim1> { static constexpr int8_t af = 13; };
template<> struct SignalConnection<DataE15::Ck, Peripheral::Usart10> { static constexpr int8_t af = 11; };
template<> struct SignalConnection<DataE15::D12, Peripheral::Fmc> { static constexpr int8_t af = 12; };
template<> struct SignalConnection<DataE15::Da12, Peripheral::Fmc> { static constexpr int8_t af = 12; };
template<> struct SignalConnection<DataE15::R7, Peripheral::Ltdc> { static constexpr int8_t af = 14; };

struct DataH0 {
	static constexpr Gpio::Port port = Gpio::Port::H;
	static constexpr uint8_t pin = 0;
	struct BitBang { using Data = DataH0; static constexpr Gpio::Signal Signal = Gpio::Signal::BitBang; };
	struct Oscin { using Data = DataH0; static constexpr Gpio::Signal Signal = Gpio::Signal::Oscin; };
};
template<Peripheral p> struct SignalConnection<DataH0::BitBang, p> {
	static_assert(p == Peripheral::BitBang, "GpioH0::BitBang only connects to software drivers!"); };
template<Peripheral p> struct SignalConnection<DataH0::Oscin, p> {
	static_assert((p == Peripheral::Rcc),"GpioH0::Oscin only connects to Rcc!"); };
template<> struct SignalConnection<DataH0::BitBang, Peripheral::BitBang> { static constexpr int8_t af = -1; };
template<> struct SignalConnection<DataH0::Oscin, Peripheral::Rcc> { static constexpr int8_t af = -1; };

struct DataH1 {
	static constexpr Gpio::Port port = Gpio::Port::H;
	static constexpr uint8_t pin = 1;
	struct BitBang { using Data = DataH1; static constexpr Gpio::Signal Signal = Gpio::Signal::BitBang; };
	struct Oscout { using Data = DataH1; static constexpr Gpio::Signal Signal = Gpio::Signal::Oscout; };
};
template<Peripheral p> struct SignalConnection<DataH1::BitBang, p> {
	static_assert(p == Peripheral::BitBang, "GpioH1::BitBang only connects to software drivers!"); };
template<Peripheral p> struct SignalConnection<DataH1::Oscout, p> {
	static_assert((p == Peripheral::Rcc),"GpioH1::Oscout only connects to Rcc!"); };
template<> struct SignalConnection<DataH1::BitBang, Peripheral::BitBang> { static constexpr int8_t af = -1; };
template<> struct SignalConnection<DataH1::Oscout, Peripheral::Rcc> { static constexpr int8_t af = -1; };

} // namespace modm::platform::detail
/// @endcond