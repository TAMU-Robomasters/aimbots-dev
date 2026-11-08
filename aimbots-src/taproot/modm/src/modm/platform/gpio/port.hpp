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

#pragma once

#include "port_shim.hpp"
#include "set.hpp"
#include <type_traits>
#include <modm/math/utils/bit_operation.hpp>

namespace modm::platform
{

/// @ingroup modm_platform_gpio
template< class StartGpio, int8_t Width >
class GpioPort : public ::modm::GpioPort/** @cond */, public detail::GpioSetShim<StartGpio, Width> /** @endcond */
{
	using PinSet = detail::GpioSetShim<StartGpio, Width>;
public:
	using PinSet::width;
	static_assert(width <= 16, "Only a maximum of 16 pins are supported by GpioPort!");
	using PortType = std::conditional_t< (width > 8), uint16_t, uint8_t >;
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
		if constexpr (mask(5)) r = (GPIOH->ODR & mask(5)) ^ inverted(5);
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
		if constexpr (mask(5)) r = (GPIOH->IDR & mask(5)) ^ inverted(5);
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
		if constexpr (mask(0)) {
			p ^= inverted(0);
			GPIOA->BSRR = ((~p & mask(0)) << 16) | (p & mask(0));
		}
		if constexpr (mask(1)) {
			p ^= inverted(1);
			GPIOB->BSRR = ((~p & mask(1)) << 16) | (p & mask(1));
		}
		if constexpr (mask(2)) {
			p ^= inverted(2);
			GPIOC->BSRR = ((~p & mask(2)) << 16) | (p & mask(2));
		}
		if constexpr (mask(3)) {
			p ^= inverted(3);
			GPIOD->BSRR = ((~p & mask(3)) << 16) | (p & mask(3));
		}
		if constexpr (mask(4)) {
			p ^= inverted(4);
			GPIOE->BSRR = ((~p & mask(4)) << 16) | (p & mask(4));
		}
		if constexpr (mask(5)) {
			p ^= inverted(5);
			GPIOH->BSRR = ((~p & mask(5)) << 16) | (p & mask(5));
		}
	}
};

} // namespace modm::platform