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

#ifndef MODM_STM32_GPIO_PORT_HPP
#define MODM_STM32_GPIO_PORT_HPP

#include "base.hpp"
#include "set.hpp"
#include <type_traits>
#include <modm/math/utils/bit_operation.hpp>

namespace modm
{

namespace platform
{

/// @cond
template< Gpio::Port port >
struct GpioPortInfo;
template< class StartGpio, int8_t Width >
struct GpioSetShim
{
	static typename GpioPortInfo<StartGpio::port>::template check<StartGpio::pin, Width> check;
};
/// @endcond

/// @ingroup modm_platform_gpio
template< class StartGpio, int8_t Width >
class GpioPort : public ::modm::GpioPort/** @cond */, public GpioSetShim<StartGpio, Width> /** @endcond */
{
	using PinSet = GpioSetShim<StartGpio, Width>;
public:
	using PinSet::width;
	static_assert(width <= 16, "Only a maximum of 16 pins are supported by GpioPort!");
	using PortType = std::conditional_t< (width > 8),
										 uint16_t,
										 uint8_t >;
	static constexpr DataOrder getDataOrder()
	{ return Width > 0 ? GpioPort::DataOrder::Normal : GpioPort::DataOrder::Reversed; }

protected:
	using PinSet::mask;
	using PinSet::inverted;
	static constexpr uint8_t StartPin = Width > 0 ? StartGpio::pin : StartGpio::pin - width + 1;
	static constexpr uint8_t StartPinReversed = (8 - StartPin - width) + 8;

public:
	static PortType isSet()
	{
		uint16_t r{0};
		if constexpr (mask(0)) r = (GPIOA->ODR & mask(0)) ^ inverted(0);
		if constexpr (mask(1)) r = (GPIOB->ODR & mask(1)) ^ inverted(1);
		if constexpr (mask(2)) r = (GPIOC->ODR & mask(2)) ^ inverted(2);
		if constexpr (mask(3)) r = (GPIOD->ODR & mask(3)) ^ inverted(3);
		if constexpr (mask(4)) r = (GPIOE->ODR & mask(4)) ^ inverted(4);
		if constexpr (mask(5)) r = (GPIOF->ODR & mask(5)) ^ inverted(5);
		if constexpr (mask(6)) r = (GPIOG->ODR & mask(6)) ^ inverted(6);
		if constexpr (mask(7)) r = (GPIOH->ODR & mask(7)) ^ inverted(7);
		if constexpr (mask(8)) r = (GPIOI->ODR & mask(8)) ^ inverted(8);
		if constexpr (getDataOrder() == modm::GpioPort::DataOrder::Reversed)
			 return bitReverse(r) >> StartPinReversed;
		else return            r  >> StartPin;
	}

	static PortType read()
	{
		uint16_t r{0};
		if constexpr (mask(0)) r = (GPIOA->IDR & mask(0)) ^ inverted(0);
		if constexpr (mask(1)) r = (GPIOB->IDR & mask(1)) ^ inverted(1);
		if constexpr (mask(2)) r = (GPIOC->IDR & mask(2)) ^ inverted(2);
		if constexpr (mask(3)) r = (GPIOD->IDR & mask(3)) ^ inverted(3);
		if constexpr (mask(4)) r = (GPIOE->IDR & mask(4)) ^ inverted(4);
		if constexpr (mask(5)) r = (GPIOF->IDR & mask(5)) ^ inverted(5);
		if constexpr (mask(6)) r = (GPIOG->IDR & mask(6)) ^ inverted(6);
		if constexpr (mask(7)) r = (GPIOH->IDR & mask(7)) ^ inverted(7);
		if constexpr (mask(8)) r = (GPIOI->IDR & mask(8)) ^ inverted(8);
		if constexpr (getDataOrder() == modm::GpioPort::DataOrder::Reversed)
			 return bitReverse(r) >> StartPinReversed;
		else return            r  >> StartPin;
	}

	static void write(PortType data)
	{
		uint16_t p;
		if constexpr (getDataOrder() == modm::GpioPort::DataOrder::Reversed)
			 p = bitReverse(uint16_t(uint16_t(data) << StartPinReversed));
		else p =            uint16_t(data) << StartPin;
		if constexpr (mask(0)) { p ^= inverted(0);
			GPIOA->BSRR = ((~p & mask(0)) << 16) | (p & mask(0));
		}
		if constexpr (mask(1)) { p ^= inverted(1);
			GPIOB->BSRR = ((~p & mask(1)) << 16) | (p & mask(1));
		}
		if constexpr (mask(2)) { p ^= inverted(2);
			GPIOC->BSRR = ((~p & mask(2)) << 16) | (p & mask(2));
		}
		if constexpr (mask(3)) { p ^= inverted(3);
			GPIOD->BSRR = ((~p & mask(3)) << 16) | (p & mask(3));
		}
		if constexpr (mask(4)) { p ^= inverted(4);
			GPIOE->BSRR = ((~p & mask(4)) << 16) | (p & mask(4));
		}
		if constexpr (mask(5)) { p ^= inverted(5);
			GPIOF->BSRR = ((~p & mask(5)) << 16) | (p & mask(5));
		}
		if constexpr (mask(6)) { p ^= inverted(6);
			GPIOG->BSRR = ((~p & mask(6)) << 16) | (p & mask(6));
		}
		if constexpr (mask(7)) { p ^= inverted(7);
			GPIOH->BSRR = ((~p & mask(7)) << 16) | (p & mask(7));
		}
		if constexpr (mask(8)) { p ^= inverted(8);
			GPIOI->BSRR = ((~p & mask(8)) << 16) | (p & mask(8));
		}
	}
};

/// @cond
// ------ Port Width Information ------
template<>
struct GpioPortInfo<Gpio::Port::A>
{
	static constexpr int8_t Width = 16;
	static constexpr int8_t StartPin = 0;
	static constexpr int8_t EndPin = StartPin + Width;

	template< uint8_t QueryStartPin, int8_t QueryWidth >
	struct check
	{
		static constexpr bool isNormal = QueryWidth > 0;
		static constexpr bool isReversed = QueryWidth < 0;
		static constexpr int8_t width = isNormal ? QueryWidth : -QueryWidth;
		static_assert(isReversed or width <= Width,
			"GpioPort Width out of bounds! Maximum is 16.");

		static_assert(isReversed or (QueryStartPin + QueryWidth <= EndPin),
			"GpioPort StartPin + Width out of bounds! Maximum is 15.");
		static_assert(isNormal or (StartPin <= QueryStartPin + QueryWidth + 1),
			"GpioPort StartPin - Width out of bounds! Minimum is 0.");
	};
};
template<>
struct GpioPortInfo<Gpio::Port::B>
{
	static constexpr int8_t Width = 16;
	static constexpr int8_t StartPin = 0;
	static constexpr int8_t EndPin = StartPin + Width;

	template< uint8_t QueryStartPin, int8_t QueryWidth >
	struct check
	{
		static constexpr bool isNormal = QueryWidth > 0;
		static constexpr bool isReversed = QueryWidth < 0;
		static constexpr int8_t width = isNormal ? QueryWidth : -QueryWidth;
		static_assert(isReversed or width <= Width,
			"GpioPort Width out of bounds! Maximum is 16.");

		static_assert(isReversed or (QueryStartPin + QueryWidth <= EndPin),
			"GpioPort StartPin + Width out of bounds! Maximum is 15.");
		static_assert(isNormal or (StartPin <= QueryStartPin + QueryWidth + 1),
			"GpioPort StartPin - Width out of bounds! Minimum is 0.");
	};
};
template<>
struct GpioPortInfo<Gpio::Port::C>
{
	static constexpr int8_t Width = 16;
	static constexpr int8_t StartPin = 0;
	static constexpr int8_t EndPin = StartPin + Width;

	template< uint8_t QueryStartPin, int8_t QueryWidth >
	struct check
	{
		static constexpr bool isNormal = QueryWidth > 0;
		static constexpr bool isReversed = QueryWidth < 0;
		static constexpr int8_t width = isNormal ? QueryWidth : -QueryWidth;
		static_assert(isReversed or width <= Width,
			"GpioPort Width out of bounds! Maximum is 16.");

		static_assert(isReversed or (QueryStartPin + QueryWidth <= EndPin),
			"GpioPort StartPin + Width out of bounds! Maximum is 15.");
		static_assert(isNormal or (StartPin <= QueryStartPin + QueryWidth + 1),
			"GpioPort StartPin - Width out of bounds! Minimum is 0.");
	};
};
template<>
struct GpioPortInfo<Gpio::Port::D>
{
	static constexpr int8_t Width = 16;
	static constexpr int8_t StartPin = 0;
	static constexpr int8_t EndPin = StartPin + Width;

	template< uint8_t QueryStartPin, int8_t QueryWidth >
	struct check
	{
		static constexpr bool isNormal = QueryWidth > 0;
		static constexpr bool isReversed = QueryWidth < 0;
		static constexpr int8_t width = isNormal ? QueryWidth : -QueryWidth;
		static_assert(isReversed or width <= Width,
			"GpioPort Width out of bounds! Maximum is 16.");

		static_assert(isReversed or (QueryStartPin + QueryWidth <= EndPin),
			"GpioPort StartPin + Width out of bounds! Maximum is 15.");
		static_assert(isNormal or (StartPin <= QueryStartPin + QueryWidth + 1),
			"GpioPort StartPin - Width out of bounds! Minimum is 0.");
	};
};
template<>
struct GpioPortInfo<Gpio::Port::E>
{
	static constexpr int8_t Width = 16;
	static constexpr int8_t StartPin = 0;
	static constexpr int8_t EndPin = StartPin + Width;

	template< uint8_t QueryStartPin, int8_t QueryWidth >
	struct check
	{
		static constexpr bool isNormal = QueryWidth > 0;
		static constexpr bool isReversed = QueryWidth < 0;
		static constexpr int8_t width = isNormal ? QueryWidth : -QueryWidth;
		static_assert(isReversed or width <= Width,
			"GpioPort Width out of bounds! Maximum is 16.");

		static_assert(isReversed or (QueryStartPin + QueryWidth <= EndPin),
			"GpioPort StartPin + Width out of bounds! Maximum is 15.");
		static_assert(isNormal or (StartPin <= QueryStartPin + QueryWidth + 1),
			"GpioPort StartPin - Width out of bounds! Minimum is 0.");
	};
};
template<>
struct GpioPortInfo<Gpio::Port::F>
{
	static constexpr int8_t Width = 16;
	static constexpr int8_t StartPin = 0;
	static constexpr int8_t EndPin = StartPin + Width;

	template< uint8_t QueryStartPin, int8_t QueryWidth >
	struct check
	{
		static constexpr bool isNormal = QueryWidth > 0;
		static constexpr bool isReversed = QueryWidth < 0;
		static constexpr int8_t width = isNormal ? QueryWidth : -QueryWidth;
		static_assert(isReversed or width <= Width,
			"GpioPort Width out of bounds! Maximum is 16.");

		static_assert(isReversed or (QueryStartPin + QueryWidth <= EndPin),
			"GpioPort StartPin + Width out of bounds! Maximum is 15.");
		static_assert(isNormal or (StartPin <= QueryStartPin + QueryWidth + 1),
			"GpioPort StartPin - Width out of bounds! Minimum is 0.");
	};
};
template<>
struct GpioPortInfo<Gpio::Port::G>
{
	static constexpr int8_t Width = 16;
	static constexpr int8_t StartPin = 0;
	static constexpr int8_t EndPin = StartPin + Width;

	template< uint8_t QueryStartPin, int8_t QueryWidth >
	struct check
	{
		static constexpr bool isNormal = QueryWidth > 0;
		static constexpr bool isReversed = QueryWidth < 0;
		static constexpr int8_t width = isNormal ? QueryWidth : -QueryWidth;
		static_assert(isReversed or width <= Width,
			"GpioPort Width out of bounds! Maximum is 16.");

		static_assert(isReversed or (QueryStartPin + QueryWidth <= EndPin),
			"GpioPort StartPin + Width out of bounds! Maximum is 15.");
		static_assert(isNormal or (StartPin <= QueryStartPin + QueryWidth + 1),
			"GpioPort StartPin - Width out of bounds! Minimum is 0.");
	};
};
template<>
struct GpioPortInfo<Gpio::Port::H>
{
	static constexpr int8_t Width = 16;
	static constexpr int8_t StartPin = 0;
	static constexpr int8_t EndPin = StartPin + Width;

	template< uint8_t QueryStartPin, int8_t QueryWidth >
	struct check
	{
		static constexpr bool isNormal = QueryWidth > 0;
		static constexpr bool isReversed = QueryWidth < 0;
		static constexpr int8_t width = isNormal ? QueryWidth : -QueryWidth;
		static_assert(isReversed or width <= Width,
			"GpioPort Width out of bounds! Maximum is 16.");

		static_assert(isReversed or (QueryStartPin + QueryWidth <= EndPin),
			"GpioPort StartPin + Width out of bounds! Maximum is 15.");
		static_assert(isNormal or (StartPin <= QueryStartPin + QueryWidth + 1),
			"GpioPort StartPin - Width out of bounds! Minimum is 0.");
	};
};
template<>
struct GpioPortInfo<Gpio::Port::I>
{
	static constexpr int8_t Width = 12;
	static constexpr int8_t StartPin = 0;
	static constexpr int8_t EndPin = StartPin + Width;

	template< uint8_t QueryStartPin, int8_t QueryWidth >
	struct check
	{
		static constexpr bool isNormal = QueryWidth > 0;
		static constexpr bool isReversed = QueryWidth < 0;
		static constexpr int8_t width = isNormal ? QueryWidth : -QueryWidth;
		static_assert(isReversed or width <= Width,
			"GpioPort Width out of bounds! Maximum is 12.");

		static_assert(isReversed or (QueryStartPin + QueryWidth <= EndPin),
			"GpioPort StartPin + Width out of bounds! Maximum is 11.");
		static_assert(isNormal or (StartPin <= QueryStartPin + QueryWidth + 1),
			"GpioPort StartPin - Width out of bounds! Minimum is 0.");
	};
};
// ------ Translator classes from Gpio + Width -> GpioSet ------
template< class StartGpio, int8_t offset >
struct GpioShim
{
	static constexpr typename StartGpio::Port port = StartGpio::port;
	static constexpr uint16_t mask = (1ul << (StartGpio::pin + offset));
	static constexpr bool isInverted = StartGpio::isInverted;
};
template< class StartGpio >
struct GpioSetShim<StartGpio, -16> : public GpioSet<
		GpioShim<StartGpio, 0>,
		GpioShim<StartGpio, -1>,
		GpioShim<StartGpio, -2>,
		GpioShim<StartGpio, -3>,
		GpioShim<StartGpio, -4>,
		GpioShim<StartGpio, -5>,
		GpioShim<StartGpio, -6>,
		GpioShim<StartGpio, -7>,
		GpioShim<StartGpio, -8>,
		GpioShim<StartGpio, -9>,
		GpioShim<StartGpio, -10>,
		GpioShim<StartGpio, -11>,
		GpioShim<StartGpio, -12>,
		GpioShim<StartGpio, -13>,
		GpioShim<StartGpio, -14>,
		GpioShim<StartGpio, -15>
		> { static typename GpioPortInfo<StartGpio::port>::template check<StartGpio::pin, -16> check; };
template< class StartGpio >
struct GpioSetShim<StartGpio, -15> : public GpioSet<
		GpioShim<StartGpio, 0>,
		GpioShim<StartGpio, -1>,
		GpioShim<StartGpio, -2>,
		GpioShim<StartGpio, -3>,
		GpioShim<StartGpio, -4>,
		GpioShim<StartGpio, -5>,
		GpioShim<StartGpio, -6>,
		GpioShim<StartGpio, -7>,
		GpioShim<StartGpio, -8>,
		GpioShim<StartGpio, -9>,
		GpioShim<StartGpio, -10>,
		GpioShim<StartGpio, -11>,
		GpioShim<StartGpio, -12>,
		GpioShim<StartGpio, -13>,
		GpioShim<StartGpio, -14>
		> { static typename GpioPortInfo<StartGpio::port>::template check<StartGpio::pin, -15> check; };
template< class StartGpio >
struct GpioSetShim<StartGpio, -14> : public GpioSet<
		GpioShim<StartGpio, 0>,
		GpioShim<StartGpio, -1>,
		GpioShim<StartGpio, -2>,
		GpioShim<StartGpio, -3>,
		GpioShim<StartGpio, -4>,
		GpioShim<StartGpio, -5>,
		GpioShim<StartGpio, -6>,
		GpioShim<StartGpio, -7>,
		GpioShim<StartGpio, -8>,
		GpioShim<StartGpio, -9>,
		GpioShim<StartGpio, -10>,
		GpioShim<StartGpio, -11>,
		GpioShim<StartGpio, -12>,
		GpioShim<StartGpio, -13>
		> { static typename GpioPortInfo<StartGpio::port>::template check<StartGpio::pin, -14> check; };
template< class StartGpio >
struct GpioSetShim<StartGpio, -13> : public GpioSet<
		GpioShim<StartGpio, 0>,
		GpioShim<StartGpio, -1>,
		GpioShim<StartGpio, -2>,
		GpioShim<StartGpio, -3>,
		GpioShim<StartGpio, -4>,
		GpioShim<StartGpio, -5>,
		GpioShim<StartGpio, -6>,
		GpioShim<StartGpio, -7>,
		GpioShim<StartGpio, -8>,
		GpioShim<StartGpio, -9>,
		GpioShim<StartGpio, -10>,
		GpioShim<StartGpio, -11>,
		GpioShim<StartGpio, -12>
		> { static typename GpioPortInfo<StartGpio::port>::template check<StartGpio::pin, -13> check; };
template< class StartGpio >
struct GpioSetShim<StartGpio, -12> : public GpioSet<
		GpioShim<StartGpio, 0>,
		GpioShim<StartGpio, -1>,
		GpioShim<StartGpio, -2>,
		GpioShim<StartGpio, -3>,
		GpioShim<StartGpio, -4>,
		GpioShim<StartGpio, -5>,
		GpioShim<StartGpio, -6>,
		GpioShim<StartGpio, -7>,
		GpioShim<StartGpio, -8>,
		GpioShim<StartGpio, -9>,
		GpioShim<StartGpio, -10>,
		GpioShim<StartGpio, -11>
		> { static typename GpioPortInfo<StartGpio::port>::template check<StartGpio::pin, -12> check; };
template< class StartGpio >
struct GpioSetShim<StartGpio, -11> : public GpioSet<
		GpioShim<StartGpio, 0>,
		GpioShim<StartGpio, -1>,
		GpioShim<StartGpio, -2>,
		GpioShim<StartGpio, -3>,
		GpioShim<StartGpio, -4>,
		GpioShim<StartGpio, -5>,
		GpioShim<StartGpio, -6>,
		GpioShim<StartGpio, -7>,
		GpioShim<StartGpio, -8>,
		GpioShim<StartGpio, -9>,
		GpioShim<StartGpio, -10>
		> { static typename GpioPortInfo<StartGpio::port>::template check<StartGpio::pin, -11> check; };
template< class StartGpio >
struct GpioSetShim<StartGpio, -10> : public GpioSet<
		GpioShim<StartGpio, 0>,
		GpioShim<StartGpio, -1>,
		GpioShim<StartGpio, -2>,
		GpioShim<StartGpio, -3>,
		GpioShim<StartGpio, -4>,
		GpioShim<StartGpio, -5>,
		GpioShim<StartGpio, -6>,
		GpioShim<StartGpio, -7>,
		GpioShim<StartGpio, -8>,
		GpioShim<StartGpio, -9>
		> { static typename GpioPortInfo<StartGpio::port>::template check<StartGpio::pin, -10> check; };
template< class StartGpio >
struct GpioSetShim<StartGpio, -9> : public GpioSet<
		GpioShim<StartGpio, 0>,
		GpioShim<StartGpio, -1>,
		GpioShim<StartGpio, -2>,
		GpioShim<StartGpio, -3>,
		GpioShim<StartGpio, -4>,
		GpioShim<StartGpio, -5>,
		GpioShim<StartGpio, -6>,
		GpioShim<StartGpio, -7>,
		GpioShim<StartGpio, -8>
		> { static typename GpioPortInfo<StartGpio::port>::template check<StartGpio::pin, -9> check; };
template< class StartGpio >
struct GpioSetShim<StartGpio, -8> : public GpioSet<
		GpioShim<StartGpio, 0>,
		GpioShim<StartGpio, -1>,
		GpioShim<StartGpio, -2>,
		GpioShim<StartGpio, -3>,
		GpioShim<StartGpio, -4>,
		GpioShim<StartGpio, -5>,
		GpioShim<StartGpio, -6>,
		GpioShim<StartGpio, -7>
		> { static typename GpioPortInfo<StartGpio::port>::template check<StartGpio::pin, -8> check; };
template< class StartGpio >
struct GpioSetShim<StartGpio, -7> : public GpioSet<
		GpioShim<StartGpio, 0>,
		GpioShim<StartGpio, -1>,
		GpioShim<StartGpio, -2>,
		GpioShim<StartGpio, -3>,
		GpioShim<StartGpio, -4>,
		GpioShim<StartGpio, -5>,
		GpioShim<StartGpio, -6>
		> { static typename GpioPortInfo<StartGpio::port>::template check<StartGpio::pin, -7> check; };
template< class StartGpio >
struct GpioSetShim<StartGpio, -6> : public GpioSet<
		GpioShim<StartGpio, 0>,
		GpioShim<StartGpio, -1>,
		GpioShim<StartGpio, -2>,
		GpioShim<StartGpio, -3>,
		GpioShim<StartGpio, -4>,
		GpioShim<StartGpio, -5>
		> { static typename GpioPortInfo<StartGpio::port>::template check<StartGpio::pin, -6> check; };
template< class StartGpio >
struct GpioSetShim<StartGpio, -5> : public GpioSet<
		GpioShim<StartGpio, 0>,
		GpioShim<StartGpio, -1>,
		GpioShim<StartGpio, -2>,
		GpioShim<StartGpio, -3>,
		GpioShim<StartGpio, -4>
		> { static typename GpioPortInfo<StartGpio::port>::template check<StartGpio::pin, -5> check; };
template< class StartGpio >
struct GpioSetShim<StartGpio, -4> : public GpioSet<
		GpioShim<StartGpio, 0>,
		GpioShim<StartGpio, -1>,
		GpioShim<StartGpio, -2>,
		GpioShim<StartGpio, -3>
		> { static typename GpioPortInfo<StartGpio::port>::template check<StartGpio::pin, -4> check; };
template< class StartGpio >
struct GpioSetShim<StartGpio, -3> : public GpioSet<
		GpioShim<StartGpio, 0>,
		GpioShim<StartGpio, -1>,
		GpioShim<StartGpio, -2>
		> { static typename GpioPortInfo<StartGpio::port>::template check<StartGpio::pin, -3> check; };
template< class StartGpio >
struct GpioSetShim<StartGpio, -2> : public GpioSet<
		GpioShim<StartGpio, 0>,
		GpioShim<StartGpio, -1>
		> { static typename GpioPortInfo<StartGpio::port>::template check<StartGpio::pin, -2> check; };
template< class StartGpio >
struct GpioSetShim<StartGpio, -1> : public GpioSet<
		GpioShim<StartGpio, 0>
		> { static typename GpioPortInfo<StartGpio::port>::template check<StartGpio::pin, -1> check; };
template< class StartGpio >
struct GpioSetShim<StartGpio, 0> : public GpioSet<
		> { static typename GpioPortInfo<StartGpio::port>::template check<StartGpio::pin, 0> check; };
template< class StartGpio >
struct GpioSetShim<StartGpio, 1> : public GpioSet<
		GpioShim<StartGpio, 0>
		> { static typename GpioPortInfo<StartGpio::port>::template check<StartGpio::pin, 1> check; };
template< class StartGpio >
struct GpioSetShim<StartGpio, 2> : public GpioSet<
		GpioShim<StartGpio, 1>,
		GpioShim<StartGpio, 0>
		> { static typename GpioPortInfo<StartGpio::port>::template check<StartGpio::pin, 2> check; };
template< class StartGpio >
struct GpioSetShim<StartGpio, 3> : public GpioSet<
		GpioShim<StartGpio, 2>,
		GpioShim<StartGpio, 1>,
		GpioShim<StartGpio, 0>
		> { static typename GpioPortInfo<StartGpio::port>::template check<StartGpio::pin, 3> check; };
template< class StartGpio >
struct GpioSetShim<StartGpio, 4> : public GpioSet<
		GpioShim<StartGpio, 3>,
		GpioShim<StartGpio, 2>,
		GpioShim<StartGpio, 1>,
		GpioShim<StartGpio, 0>
		> { static typename GpioPortInfo<StartGpio::port>::template check<StartGpio::pin, 4> check; };
template< class StartGpio >
struct GpioSetShim<StartGpio, 5> : public GpioSet<
		GpioShim<StartGpio, 4>,
		GpioShim<StartGpio, 3>,
		GpioShim<StartGpio, 2>,
		GpioShim<StartGpio, 1>,
		GpioShim<StartGpio, 0>
		> { static typename GpioPortInfo<StartGpio::port>::template check<StartGpio::pin, 5> check; };
template< class StartGpio >
struct GpioSetShim<StartGpio, 6> : public GpioSet<
		GpioShim<StartGpio, 5>,
		GpioShim<StartGpio, 4>,
		GpioShim<StartGpio, 3>,
		GpioShim<StartGpio, 2>,
		GpioShim<StartGpio, 1>,
		GpioShim<StartGpio, 0>
		> { static typename GpioPortInfo<StartGpio::port>::template check<StartGpio::pin, 6> check; };
template< class StartGpio >
struct GpioSetShim<StartGpio, 7> : public GpioSet<
		GpioShim<StartGpio, 6>,
		GpioShim<StartGpio, 5>,
		GpioShim<StartGpio, 4>,
		GpioShim<StartGpio, 3>,
		GpioShim<StartGpio, 2>,
		GpioShim<StartGpio, 1>,
		GpioShim<StartGpio, 0>
		> { static typename GpioPortInfo<StartGpio::port>::template check<StartGpio::pin, 7> check; };
template< class StartGpio >
struct GpioSetShim<StartGpio, 8> : public GpioSet<
		GpioShim<StartGpio, 7>,
		GpioShim<StartGpio, 6>,
		GpioShim<StartGpio, 5>,
		GpioShim<StartGpio, 4>,
		GpioShim<StartGpio, 3>,
		GpioShim<StartGpio, 2>,
		GpioShim<StartGpio, 1>,
		GpioShim<StartGpio, 0>
		> { static typename GpioPortInfo<StartGpio::port>::template check<StartGpio::pin, 8> check; };
template< class StartGpio >
struct GpioSetShim<StartGpio, 9> : public GpioSet<
		GpioShim<StartGpio, 8>,
		GpioShim<StartGpio, 7>,
		GpioShim<StartGpio, 6>,
		GpioShim<StartGpio, 5>,
		GpioShim<StartGpio, 4>,
		GpioShim<StartGpio, 3>,
		GpioShim<StartGpio, 2>,
		GpioShim<StartGpio, 1>,
		GpioShim<StartGpio, 0>
		> { static typename GpioPortInfo<StartGpio::port>::template check<StartGpio::pin, 9> check; };
template< class StartGpio >
struct GpioSetShim<StartGpio, 10> : public GpioSet<
		GpioShim<StartGpio, 9>,
		GpioShim<StartGpio, 8>,
		GpioShim<StartGpio, 7>,
		GpioShim<StartGpio, 6>,
		GpioShim<StartGpio, 5>,
		GpioShim<StartGpio, 4>,
		GpioShim<StartGpio, 3>,
		GpioShim<StartGpio, 2>,
		GpioShim<StartGpio, 1>,
		GpioShim<StartGpio, 0>
		> { static typename GpioPortInfo<StartGpio::port>::template check<StartGpio::pin, 10> check; };
template< class StartGpio >
struct GpioSetShim<StartGpio, 11> : public GpioSet<
		GpioShim<StartGpio, 10>,
		GpioShim<StartGpio, 9>,
		GpioShim<StartGpio, 8>,
		GpioShim<StartGpio, 7>,
		GpioShim<StartGpio, 6>,
		GpioShim<StartGpio, 5>,
		GpioShim<StartGpio, 4>,
		GpioShim<StartGpio, 3>,
		GpioShim<StartGpio, 2>,
		GpioShim<StartGpio, 1>,
		GpioShim<StartGpio, 0>
		> { static typename GpioPortInfo<StartGpio::port>::template check<StartGpio::pin, 11> check; };
template< class StartGpio >
struct GpioSetShim<StartGpio, 12> : public GpioSet<
		GpioShim<StartGpio, 11>,
		GpioShim<StartGpio, 10>,
		GpioShim<StartGpio, 9>,
		GpioShim<StartGpio, 8>,
		GpioShim<StartGpio, 7>,
		GpioShim<StartGpio, 6>,
		GpioShim<StartGpio, 5>,
		GpioShim<StartGpio, 4>,
		GpioShim<StartGpio, 3>,
		GpioShim<StartGpio, 2>,
		GpioShim<StartGpio, 1>,
		GpioShim<StartGpio, 0>
		> { static typename GpioPortInfo<StartGpio::port>::template check<StartGpio::pin, 12> check; };
template< class StartGpio >
struct GpioSetShim<StartGpio, 13> : public GpioSet<
		GpioShim<StartGpio, 12>,
		GpioShim<StartGpio, 11>,
		GpioShim<StartGpio, 10>,
		GpioShim<StartGpio, 9>,
		GpioShim<StartGpio, 8>,
		GpioShim<StartGpio, 7>,
		GpioShim<StartGpio, 6>,
		GpioShim<StartGpio, 5>,
		GpioShim<StartGpio, 4>,
		GpioShim<StartGpio, 3>,
		GpioShim<StartGpio, 2>,
		GpioShim<StartGpio, 1>,
		GpioShim<StartGpio, 0>
		> { static typename GpioPortInfo<StartGpio::port>::template check<StartGpio::pin, 13> check; };
template< class StartGpio >
struct GpioSetShim<StartGpio, 14> : public GpioSet<
		GpioShim<StartGpio, 13>,
		GpioShim<StartGpio, 12>,
		GpioShim<StartGpio, 11>,
		GpioShim<StartGpio, 10>,
		GpioShim<StartGpio, 9>,
		GpioShim<StartGpio, 8>,
		GpioShim<StartGpio, 7>,
		GpioShim<StartGpio, 6>,
		GpioShim<StartGpio, 5>,
		GpioShim<StartGpio, 4>,
		GpioShim<StartGpio, 3>,
		GpioShim<StartGpio, 2>,
		GpioShim<StartGpio, 1>,
		GpioShim<StartGpio, 0>
		> { static typename GpioPortInfo<StartGpio::port>::template check<StartGpio::pin, 14> check; };
template< class StartGpio >
struct GpioSetShim<StartGpio, 15> : public GpioSet<
		GpioShim<StartGpio, 14>,
		GpioShim<StartGpio, 13>,
		GpioShim<StartGpio, 12>,
		GpioShim<StartGpio, 11>,
		GpioShim<StartGpio, 10>,
		GpioShim<StartGpio, 9>,
		GpioShim<StartGpio, 8>,
		GpioShim<StartGpio, 7>,
		GpioShim<StartGpio, 6>,
		GpioShim<StartGpio, 5>,
		GpioShim<StartGpio, 4>,
		GpioShim<StartGpio, 3>,
		GpioShim<StartGpio, 2>,
		GpioShim<StartGpio, 1>,
		GpioShim<StartGpio, 0>
		> { static typename GpioPortInfo<StartGpio::port>::template check<StartGpio::pin, 15> check; };
template< class StartGpio >
struct GpioSetShim<StartGpio, 16> : public GpioSet<
		GpioShim<StartGpio, 15>,
		GpioShim<StartGpio, 14>,
		GpioShim<StartGpio, 13>,
		GpioShim<StartGpio, 12>,
		GpioShim<StartGpio, 11>,
		GpioShim<StartGpio, 10>,
		GpioShim<StartGpio, 9>,
		GpioShim<StartGpio, 8>,
		GpioShim<StartGpio, 7>,
		GpioShim<StartGpio, 6>,
		GpioShim<StartGpio, 5>,
		GpioShim<StartGpio, 4>,
		GpioShim<StartGpio, 3>,
		GpioShim<StartGpio, 2>,
		GpioShim<StartGpio, 1>,
		GpioShim<StartGpio, 0>
		> { static typename GpioPortInfo<StartGpio::port>::template check<StartGpio::pin, 16> check; };
/// @endcond

} // namespace platform

} // namespace modm

#endif // MODM_STM32_GPIO_PORT_HPP