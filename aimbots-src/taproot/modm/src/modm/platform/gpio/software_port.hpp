/*
 * Copyright (c) 2018, Niklas Hauser
 *
 * This file is part of the modm project.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
// ----------------------------------------------------------------------------

#pragma once

#include "set.hpp"
#include <type_traits>

namespace modm::platform
{

/**
 * Create an up to 32-bit port from arbitrary pins.
 *
 * This class optimizes the data type for the `read()` and `write()` methods.
 * Supplying up to 8 Gpios will use `uint8_t`, up to 16 Gpios `uint16_t` and
 * up to 32 Gpios `uint32_t`.
 *
 * @note Since the bit order is explicitly given by the order of template arguments,
 *       this class only supports `DataOrder::Normal`.
 *       If you need reverse bit order, reverse the order of `Gpios`!
 *
 * @tparam Gpios	Up to 32 GpioIO classes, ordered MSB to LSB
 *
 * @author	Niklas Hauser
 * @ingroup	modm_platform_gpio
 */
template< class... Gpios >
class SoftwareGpioPort : public ::modm::GpioPort, public GpioSet<Gpios...>
{
	using Set = GpioSet<Gpios...>;
public:
	using Set::width;
	static_assert(width <= 32, "Only a maximum of 32 pins are supported by this Port!");
	using PortType = std::conditional_t< (width > 8),
					 std::conditional_t< (width > 16),
										 uint32_t,
										 uint16_t >,
										 uint8_t >;
	static constexpr DataOrder getDataOrder()
	{ return ::modm::GpioPort::DataOrder::Normal; }

protected:
	static constexpr int8_t shift_masks[6][width] = {
		{(Gpios::port == Set::Port::A ? Gpios::pin : -1)...},
		{(Gpios::port == Set::Port::B ? Gpios::pin : -1)...},
		{(Gpios::port == Set::Port::C ? Gpios::pin : -1)...},
		{(Gpios::port == Set::Port::D ? Gpios::pin : -1)...},
		{(Gpios::port == Set::Port::E ? Gpios::pin : -1)...},
		{(Gpios::port == Set::Port::H ? Gpios::pin : -1)...},
	};
	static constexpr int8_t shift_mask(uint8_t id, uint8_t pos) { return shift_masks[id][width - 1 - pos]; }
	using Set::mask;
	using Set::inverted;

public:
	static PortType isSet()
	{
		PortType r{0};
		if constexpr (mask(0)) {
			const uint16_t p = (GPIOA->ODR & mask(0)) ^ inverted(0);
			if constexpr (0 < width) if constexpr (constexpr auto pos = shift_mask(0, 0); pos >= 0) r |= ((p >> pos) & 1) << 0;
			if constexpr (1 < width) if constexpr (constexpr auto pos = shift_mask(0, 1); pos >= 0) r |= ((p >> pos) & 1) << 1;
			if constexpr (2 < width) if constexpr (constexpr auto pos = shift_mask(0, 2); pos >= 0) r |= ((p >> pos) & 1) << 2;
			if constexpr (3 < width) if constexpr (constexpr auto pos = shift_mask(0, 3); pos >= 0) r |= ((p >> pos) & 1) << 3;
			if constexpr (4 < width) if constexpr (constexpr auto pos = shift_mask(0, 4); pos >= 0) r |= ((p >> pos) & 1) << 4;
			if constexpr (5 < width) if constexpr (constexpr auto pos = shift_mask(0, 5); pos >= 0) r |= ((p >> pos) & 1) << 5;
			if constexpr (6 < width) if constexpr (constexpr auto pos = shift_mask(0, 6); pos >= 0) r |= ((p >> pos) & 1) << 6;
			if constexpr (7 < width) if constexpr (constexpr auto pos = shift_mask(0, 7); pos >= 0) r |= ((p >> pos) & 1) << 7;
			if constexpr (8 < width) if constexpr (constexpr auto pos = shift_mask(0, 8); pos >= 0) r |= ((p >> pos) & 1) << 8;
			if constexpr (9 < width) if constexpr (constexpr auto pos = shift_mask(0, 9); pos >= 0) r |= ((p >> pos) & 1) << 9;
			if constexpr (10 < width) if constexpr (constexpr auto pos = shift_mask(0, 10); pos >= 0) r |= ((p >> pos) & 1) << 10;
			if constexpr (11 < width) if constexpr (constexpr auto pos = shift_mask(0, 11); pos >= 0) r |= ((p >> pos) & 1) << 11;
			if constexpr (12 < width) if constexpr (constexpr auto pos = shift_mask(0, 12); pos >= 0) r |= ((p >> pos) & 1) << 12;
			if constexpr (13 < width) if constexpr (constexpr auto pos = shift_mask(0, 13); pos >= 0) r |= ((p >> pos) & 1) << 13;
			if constexpr (14 < width) if constexpr (constexpr auto pos = shift_mask(0, 14); pos >= 0) r |= ((p >> pos) & 1) << 14;
			if constexpr (15 < width) if constexpr (constexpr auto pos = shift_mask(0, 15); pos >= 0) r |= ((p >> pos) & 1) << 15;
			if constexpr (16 < width) if constexpr (constexpr auto pos = shift_mask(0, 16); pos >= 0) r |= ((p >> pos) & 1) << 16;
			if constexpr (17 < width) if constexpr (constexpr auto pos = shift_mask(0, 17); pos >= 0) r |= ((p >> pos) & 1) << 17;
			if constexpr (18 < width) if constexpr (constexpr auto pos = shift_mask(0, 18); pos >= 0) r |= ((p >> pos) & 1) << 18;
			if constexpr (19 < width) if constexpr (constexpr auto pos = shift_mask(0, 19); pos >= 0) r |= ((p >> pos) & 1) << 19;
			if constexpr (20 < width) if constexpr (constexpr auto pos = shift_mask(0, 20); pos >= 0) r |= ((p >> pos) & 1) << 20;
			if constexpr (21 < width) if constexpr (constexpr auto pos = shift_mask(0, 21); pos >= 0) r |= ((p >> pos) & 1) << 21;
			if constexpr (22 < width) if constexpr (constexpr auto pos = shift_mask(0, 22); pos >= 0) r |= ((p >> pos) & 1) << 22;
			if constexpr (23 < width) if constexpr (constexpr auto pos = shift_mask(0, 23); pos >= 0) r |= ((p >> pos) & 1) << 23;
			if constexpr (24 < width) if constexpr (constexpr auto pos = shift_mask(0, 24); pos >= 0) r |= ((p >> pos) & 1) << 24;
			if constexpr (25 < width) if constexpr (constexpr auto pos = shift_mask(0, 25); pos >= 0) r |= ((p >> pos) & 1) << 25;
			if constexpr (26 < width) if constexpr (constexpr auto pos = shift_mask(0, 26); pos >= 0) r |= ((p >> pos) & 1) << 26;
			if constexpr (27 < width) if constexpr (constexpr auto pos = shift_mask(0, 27); pos >= 0) r |= ((p >> pos) & 1) << 27;
			if constexpr (28 < width) if constexpr (constexpr auto pos = shift_mask(0, 28); pos >= 0) r |= ((p >> pos) & 1) << 28;
			if constexpr (29 < width) if constexpr (constexpr auto pos = shift_mask(0, 29); pos >= 0) r |= ((p >> pos) & 1) << 29;
			if constexpr (30 < width) if constexpr (constexpr auto pos = shift_mask(0, 30); pos >= 0) r |= ((p >> pos) & 1) << 30;
			if constexpr (31 < width) if constexpr (constexpr auto pos = shift_mask(0, 31); pos >= 0) r |= ((p >> pos) & 1) << 31;
		}
		if constexpr (mask(1)) {
			const uint16_t p = (GPIOB->ODR & mask(1)) ^ inverted(1);
			if constexpr (0 < width) if constexpr (constexpr auto pos = shift_mask(1, 0); pos >= 0) r |= ((p >> pos) & 1) << 0;
			if constexpr (1 < width) if constexpr (constexpr auto pos = shift_mask(1, 1); pos >= 0) r |= ((p >> pos) & 1) << 1;
			if constexpr (2 < width) if constexpr (constexpr auto pos = shift_mask(1, 2); pos >= 0) r |= ((p >> pos) & 1) << 2;
			if constexpr (3 < width) if constexpr (constexpr auto pos = shift_mask(1, 3); pos >= 0) r |= ((p >> pos) & 1) << 3;
			if constexpr (4 < width) if constexpr (constexpr auto pos = shift_mask(1, 4); pos >= 0) r |= ((p >> pos) & 1) << 4;
			if constexpr (5 < width) if constexpr (constexpr auto pos = shift_mask(1, 5); pos >= 0) r |= ((p >> pos) & 1) << 5;
			if constexpr (6 < width) if constexpr (constexpr auto pos = shift_mask(1, 6); pos >= 0) r |= ((p >> pos) & 1) << 6;
			if constexpr (7 < width) if constexpr (constexpr auto pos = shift_mask(1, 7); pos >= 0) r |= ((p >> pos) & 1) << 7;
			if constexpr (8 < width) if constexpr (constexpr auto pos = shift_mask(1, 8); pos >= 0) r |= ((p >> pos) & 1) << 8;
			if constexpr (9 < width) if constexpr (constexpr auto pos = shift_mask(1, 9); pos >= 0) r |= ((p >> pos) & 1) << 9;
			if constexpr (10 < width) if constexpr (constexpr auto pos = shift_mask(1, 10); pos >= 0) r |= ((p >> pos) & 1) << 10;
			if constexpr (11 < width) if constexpr (constexpr auto pos = shift_mask(1, 11); pos >= 0) r |= ((p >> pos) & 1) << 11;
			if constexpr (12 < width) if constexpr (constexpr auto pos = shift_mask(1, 12); pos >= 0) r |= ((p >> pos) & 1) << 12;
			if constexpr (13 < width) if constexpr (constexpr auto pos = shift_mask(1, 13); pos >= 0) r |= ((p >> pos) & 1) << 13;
			if constexpr (14 < width) if constexpr (constexpr auto pos = shift_mask(1, 14); pos >= 0) r |= ((p >> pos) & 1) << 14;
			if constexpr (15 < width) if constexpr (constexpr auto pos = shift_mask(1, 15); pos >= 0) r |= ((p >> pos) & 1) << 15;
			if constexpr (16 < width) if constexpr (constexpr auto pos = shift_mask(1, 16); pos >= 0) r |= ((p >> pos) & 1) << 16;
			if constexpr (17 < width) if constexpr (constexpr auto pos = shift_mask(1, 17); pos >= 0) r |= ((p >> pos) & 1) << 17;
			if constexpr (18 < width) if constexpr (constexpr auto pos = shift_mask(1, 18); pos >= 0) r |= ((p >> pos) & 1) << 18;
			if constexpr (19 < width) if constexpr (constexpr auto pos = shift_mask(1, 19); pos >= 0) r |= ((p >> pos) & 1) << 19;
			if constexpr (20 < width) if constexpr (constexpr auto pos = shift_mask(1, 20); pos >= 0) r |= ((p >> pos) & 1) << 20;
			if constexpr (21 < width) if constexpr (constexpr auto pos = shift_mask(1, 21); pos >= 0) r |= ((p >> pos) & 1) << 21;
			if constexpr (22 < width) if constexpr (constexpr auto pos = shift_mask(1, 22); pos >= 0) r |= ((p >> pos) & 1) << 22;
			if constexpr (23 < width) if constexpr (constexpr auto pos = shift_mask(1, 23); pos >= 0) r |= ((p >> pos) & 1) << 23;
			if constexpr (24 < width) if constexpr (constexpr auto pos = shift_mask(1, 24); pos >= 0) r |= ((p >> pos) & 1) << 24;
			if constexpr (25 < width) if constexpr (constexpr auto pos = shift_mask(1, 25); pos >= 0) r |= ((p >> pos) & 1) << 25;
			if constexpr (26 < width) if constexpr (constexpr auto pos = shift_mask(1, 26); pos >= 0) r |= ((p >> pos) & 1) << 26;
			if constexpr (27 < width) if constexpr (constexpr auto pos = shift_mask(1, 27); pos >= 0) r |= ((p >> pos) & 1) << 27;
			if constexpr (28 < width) if constexpr (constexpr auto pos = shift_mask(1, 28); pos >= 0) r |= ((p >> pos) & 1) << 28;
			if constexpr (29 < width) if constexpr (constexpr auto pos = shift_mask(1, 29); pos >= 0) r |= ((p >> pos) & 1) << 29;
			if constexpr (30 < width) if constexpr (constexpr auto pos = shift_mask(1, 30); pos >= 0) r |= ((p >> pos) & 1) << 30;
			if constexpr (31 < width) if constexpr (constexpr auto pos = shift_mask(1, 31); pos >= 0) r |= ((p >> pos) & 1) << 31;
		}
		if constexpr (mask(2)) {
			const uint16_t p = (GPIOC->ODR & mask(2)) ^ inverted(2);
			if constexpr (0 < width) if constexpr (constexpr auto pos = shift_mask(2, 0); pos >= 0) r |= ((p >> pos) & 1) << 0;
			if constexpr (1 < width) if constexpr (constexpr auto pos = shift_mask(2, 1); pos >= 0) r |= ((p >> pos) & 1) << 1;
			if constexpr (2 < width) if constexpr (constexpr auto pos = shift_mask(2, 2); pos >= 0) r |= ((p >> pos) & 1) << 2;
			if constexpr (3 < width) if constexpr (constexpr auto pos = shift_mask(2, 3); pos >= 0) r |= ((p >> pos) & 1) << 3;
			if constexpr (4 < width) if constexpr (constexpr auto pos = shift_mask(2, 4); pos >= 0) r |= ((p >> pos) & 1) << 4;
			if constexpr (5 < width) if constexpr (constexpr auto pos = shift_mask(2, 5); pos >= 0) r |= ((p >> pos) & 1) << 5;
			if constexpr (6 < width) if constexpr (constexpr auto pos = shift_mask(2, 6); pos >= 0) r |= ((p >> pos) & 1) << 6;
			if constexpr (7 < width) if constexpr (constexpr auto pos = shift_mask(2, 7); pos >= 0) r |= ((p >> pos) & 1) << 7;
			if constexpr (8 < width) if constexpr (constexpr auto pos = shift_mask(2, 8); pos >= 0) r |= ((p >> pos) & 1) << 8;
			if constexpr (9 < width) if constexpr (constexpr auto pos = shift_mask(2, 9); pos >= 0) r |= ((p >> pos) & 1) << 9;
			if constexpr (10 < width) if constexpr (constexpr auto pos = shift_mask(2, 10); pos >= 0) r |= ((p >> pos) & 1) << 10;
			if constexpr (11 < width) if constexpr (constexpr auto pos = shift_mask(2, 11); pos >= 0) r |= ((p >> pos) & 1) << 11;
			if constexpr (12 < width) if constexpr (constexpr auto pos = shift_mask(2, 12); pos >= 0) r |= ((p >> pos) & 1) << 12;
			if constexpr (13 < width) if constexpr (constexpr auto pos = shift_mask(2, 13); pos >= 0) r |= ((p >> pos) & 1) << 13;
			if constexpr (14 < width) if constexpr (constexpr auto pos = shift_mask(2, 14); pos >= 0) r |= ((p >> pos) & 1) << 14;
			if constexpr (15 < width) if constexpr (constexpr auto pos = shift_mask(2, 15); pos >= 0) r |= ((p >> pos) & 1) << 15;
			if constexpr (16 < width) if constexpr (constexpr auto pos = shift_mask(2, 16); pos >= 0) r |= ((p >> pos) & 1) << 16;
			if constexpr (17 < width) if constexpr (constexpr auto pos = shift_mask(2, 17); pos >= 0) r |= ((p >> pos) & 1) << 17;
			if constexpr (18 < width) if constexpr (constexpr auto pos = shift_mask(2, 18); pos >= 0) r |= ((p >> pos) & 1) << 18;
			if constexpr (19 < width) if constexpr (constexpr auto pos = shift_mask(2, 19); pos >= 0) r |= ((p >> pos) & 1) << 19;
			if constexpr (20 < width) if constexpr (constexpr auto pos = shift_mask(2, 20); pos >= 0) r |= ((p >> pos) & 1) << 20;
			if constexpr (21 < width) if constexpr (constexpr auto pos = shift_mask(2, 21); pos >= 0) r |= ((p >> pos) & 1) << 21;
			if constexpr (22 < width) if constexpr (constexpr auto pos = shift_mask(2, 22); pos >= 0) r |= ((p >> pos) & 1) << 22;
			if constexpr (23 < width) if constexpr (constexpr auto pos = shift_mask(2, 23); pos >= 0) r |= ((p >> pos) & 1) << 23;
			if constexpr (24 < width) if constexpr (constexpr auto pos = shift_mask(2, 24); pos >= 0) r |= ((p >> pos) & 1) << 24;
			if constexpr (25 < width) if constexpr (constexpr auto pos = shift_mask(2, 25); pos >= 0) r |= ((p >> pos) & 1) << 25;
			if constexpr (26 < width) if constexpr (constexpr auto pos = shift_mask(2, 26); pos >= 0) r |= ((p >> pos) & 1) << 26;
			if constexpr (27 < width) if constexpr (constexpr auto pos = shift_mask(2, 27); pos >= 0) r |= ((p >> pos) & 1) << 27;
			if constexpr (28 < width) if constexpr (constexpr auto pos = shift_mask(2, 28); pos >= 0) r |= ((p >> pos) & 1) << 28;
			if constexpr (29 < width) if constexpr (constexpr auto pos = shift_mask(2, 29); pos >= 0) r |= ((p >> pos) & 1) << 29;
			if constexpr (30 < width) if constexpr (constexpr auto pos = shift_mask(2, 30); pos >= 0) r |= ((p >> pos) & 1) << 30;
			if constexpr (31 < width) if constexpr (constexpr auto pos = shift_mask(2, 31); pos >= 0) r |= ((p >> pos) & 1) << 31;
		}
		if constexpr (mask(3)) {
			const uint16_t p = (GPIOD->ODR & mask(3)) ^ inverted(3);
			if constexpr (0 < width) if constexpr (constexpr auto pos = shift_mask(3, 0); pos >= 0) r |= ((p >> pos) & 1) << 0;
			if constexpr (1 < width) if constexpr (constexpr auto pos = shift_mask(3, 1); pos >= 0) r |= ((p >> pos) & 1) << 1;
			if constexpr (2 < width) if constexpr (constexpr auto pos = shift_mask(3, 2); pos >= 0) r |= ((p >> pos) & 1) << 2;
			if constexpr (3 < width) if constexpr (constexpr auto pos = shift_mask(3, 3); pos >= 0) r |= ((p >> pos) & 1) << 3;
			if constexpr (4 < width) if constexpr (constexpr auto pos = shift_mask(3, 4); pos >= 0) r |= ((p >> pos) & 1) << 4;
			if constexpr (5 < width) if constexpr (constexpr auto pos = shift_mask(3, 5); pos >= 0) r |= ((p >> pos) & 1) << 5;
			if constexpr (6 < width) if constexpr (constexpr auto pos = shift_mask(3, 6); pos >= 0) r |= ((p >> pos) & 1) << 6;
			if constexpr (7 < width) if constexpr (constexpr auto pos = shift_mask(3, 7); pos >= 0) r |= ((p >> pos) & 1) << 7;
			if constexpr (8 < width) if constexpr (constexpr auto pos = shift_mask(3, 8); pos >= 0) r |= ((p >> pos) & 1) << 8;
			if constexpr (9 < width) if constexpr (constexpr auto pos = shift_mask(3, 9); pos >= 0) r |= ((p >> pos) & 1) << 9;
			if constexpr (10 < width) if constexpr (constexpr auto pos = shift_mask(3, 10); pos >= 0) r |= ((p >> pos) & 1) << 10;
			if constexpr (11 < width) if constexpr (constexpr auto pos = shift_mask(3, 11); pos >= 0) r |= ((p >> pos) & 1) << 11;
			if constexpr (12 < width) if constexpr (constexpr auto pos = shift_mask(3, 12); pos >= 0) r |= ((p >> pos) & 1) << 12;
			if constexpr (13 < width) if constexpr (constexpr auto pos = shift_mask(3, 13); pos >= 0) r |= ((p >> pos) & 1) << 13;
			if constexpr (14 < width) if constexpr (constexpr auto pos = shift_mask(3, 14); pos >= 0) r |= ((p >> pos) & 1) << 14;
			if constexpr (15 < width) if constexpr (constexpr auto pos = shift_mask(3, 15); pos >= 0) r |= ((p >> pos) & 1) << 15;
			if constexpr (16 < width) if constexpr (constexpr auto pos = shift_mask(3, 16); pos >= 0) r |= ((p >> pos) & 1) << 16;
			if constexpr (17 < width) if constexpr (constexpr auto pos = shift_mask(3, 17); pos >= 0) r |= ((p >> pos) & 1) << 17;
			if constexpr (18 < width) if constexpr (constexpr auto pos = shift_mask(3, 18); pos >= 0) r |= ((p >> pos) & 1) << 18;
			if constexpr (19 < width) if constexpr (constexpr auto pos = shift_mask(3, 19); pos >= 0) r |= ((p >> pos) & 1) << 19;
			if constexpr (20 < width) if constexpr (constexpr auto pos = shift_mask(3, 20); pos >= 0) r |= ((p >> pos) & 1) << 20;
			if constexpr (21 < width) if constexpr (constexpr auto pos = shift_mask(3, 21); pos >= 0) r |= ((p >> pos) & 1) << 21;
			if constexpr (22 < width) if constexpr (constexpr auto pos = shift_mask(3, 22); pos >= 0) r |= ((p >> pos) & 1) << 22;
			if constexpr (23 < width) if constexpr (constexpr auto pos = shift_mask(3, 23); pos >= 0) r |= ((p >> pos) & 1) << 23;
			if constexpr (24 < width) if constexpr (constexpr auto pos = shift_mask(3, 24); pos >= 0) r |= ((p >> pos) & 1) << 24;
			if constexpr (25 < width) if constexpr (constexpr auto pos = shift_mask(3, 25); pos >= 0) r |= ((p >> pos) & 1) << 25;
			if constexpr (26 < width) if constexpr (constexpr auto pos = shift_mask(3, 26); pos >= 0) r |= ((p >> pos) & 1) << 26;
			if constexpr (27 < width) if constexpr (constexpr auto pos = shift_mask(3, 27); pos >= 0) r |= ((p >> pos) & 1) << 27;
			if constexpr (28 < width) if constexpr (constexpr auto pos = shift_mask(3, 28); pos >= 0) r |= ((p >> pos) & 1) << 28;
			if constexpr (29 < width) if constexpr (constexpr auto pos = shift_mask(3, 29); pos >= 0) r |= ((p >> pos) & 1) << 29;
			if constexpr (30 < width) if constexpr (constexpr auto pos = shift_mask(3, 30); pos >= 0) r |= ((p >> pos) & 1) << 30;
			if constexpr (31 < width) if constexpr (constexpr auto pos = shift_mask(3, 31); pos >= 0) r |= ((p >> pos) & 1) << 31;
		}
		if constexpr (mask(4)) {
			const uint16_t p = (GPIOE->ODR & mask(4)) ^ inverted(4);
			if constexpr (0 < width) if constexpr (constexpr auto pos = shift_mask(4, 0); pos >= 0) r |= ((p >> pos) & 1) << 0;
			if constexpr (1 < width) if constexpr (constexpr auto pos = shift_mask(4, 1); pos >= 0) r |= ((p >> pos) & 1) << 1;
			if constexpr (2 < width) if constexpr (constexpr auto pos = shift_mask(4, 2); pos >= 0) r |= ((p >> pos) & 1) << 2;
			if constexpr (3 < width) if constexpr (constexpr auto pos = shift_mask(4, 3); pos >= 0) r |= ((p >> pos) & 1) << 3;
			if constexpr (4 < width) if constexpr (constexpr auto pos = shift_mask(4, 4); pos >= 0) r |= ((p >> pos) & 1) << 4;
			if constexpr (5 < width) if constexpr (constexpr auto pos = shift_mask(4, 5); pos >= 0) r |= ((p >> pos) & 1) << 5;
			if constexpr (6 < width) if constexpr (constexpr auto pos = shift_mask(4, 6); pos >= 0) r |= ((p >> pos) & 1) << 6;
			if constexpr (7 < width) if constexpr (constexpr auto pos = shift_mask(4, 7); pos >= 0) r |= ((p >> pos) & 1) << 7;
			if constexpr (8 < width) if constexpr (constexpr auto pos = shift_mask(4, 8); pos >= 0) r |= ((p >> pos) & 1) << 8;
			if constexpr (9 < width) if constexpr (constexpr auto pos = shift_mask(4, 9); pos >= 0) r |= ((p >> pos) & 1) << 9;
			if constexpr (10 < width) if constexpr (constexpr auto pos = shift_mask(4, 10); pos >= 0) r |= ((p >> pos) & 1) << 10;
			if constexpr (11 < width) if constexpr (constexpr auto pos = shift_mask(4, 11); pos >= 0) r |= ((p >> pos) & 1) << 11;
			if constexpr (12 < width) if constexpr (constexpr auto pos = shift_mask(4, 12); pos >= 0) r |= ((p >> pos) & 1) << 12;
			if constexpr (13 < width) if constexpr (constexpr auto pos = shift_mask(4, 13); pos >= 0) r |= ((p >> pos) & 1) << 13;
			if constexpr (14 < width) if constexpr (constexpr auto pos = shift_mask(4, 14); pos >= 0) r |= ((p >> pos) & 1) << 14;
			if constexpr (15 < width) if constexpr (constexpr auto pos = shift_mask(4, 15); pos >= 0) r |= ((p >> pos) & 1) << 15;
			if constexpr (16 < width) if constexpr (constexpr auto pos = shift_mask(4, 16); pos >= 0) r |= ((p >> pos) & 1) << 16;
			if constexpr (17 < width) if constexpr (constexpr auto pos = shift_mask(4, 17); pos >= 0) r |= ((p >> pos) & 1) << 17;
			if constexpr (18 < width) if constexpr (constexpr auto pos = shift_mask(4, 18); pos >= 0) r |= ((p >> pos) & 1) << 18;
			if constexpr (19 < width) if constexpr (constexpr auto pos = shift_mask(4, 19); pos >= 0) r |= ((p >> pos) & 1) << 19;
			if constexpr (20 < width) if constexpr (constexpr auto pos = shift_mask(4, 20); pos >= 0) r |= ((p >> pos) & 1) << 20;
			if constexpr (21 < width) if constexpr (constexpr auto pos = shift_mask(4, 21); pos >= 0) r |= ((p >> pos) & 1) << 21;
			if constexpr (22 < width) if constexpr (constexpr auto pos = shift_mask(4, 22); pos >= 0) r |= ((p >> pos) & 1) << 22;
			if constexpr (23 < width) if constexpr (constexpr auto pos = shift_mask(4, 23); pos >= 0) r |= ((p >> pos) & 1) << 23;
			if constexpr (24 < width) if constexpr (constexpr auto pos = shift_mask(4, 24); pos >= 0) r |= ((p >> pos) & 1) << 24;
			if constexpr (25 < width) if constexpr (constexpr auto pos = shift_mask(4, 25); pos >= 0) r |= ((p >> pos) & 1) << 25;
			if constexpr (26 < width) if constexpr (constexpr auto pos = shift_mask(4, 26); pos >= 0) r |= ((p >> pos) & 1) << 26;
			if constexpr (27 < width) if constexpr (constexpr auto pos = shift_mask(4, 27); pos >= 0) r |= ((p >> pos) & 1) << 27;
			if constexpr (28 < width) if constexpr (constexpr auto pos = shift_mask(4, 28); pos >= 0) r |= ((p >> pos) & 1) << 28;
			if constexpr (29 < width) if constexpr (constexpr auto pos = shift_mask(4, 29); pos >= 0) r |= ((p >> pos) & 1) << 29;
			if constexpr (30 < width) if constexpr (constexpr auto pos = shift_mask(4, 30); pos >= 0) r |= ((p >> pos) & 1) << 30;
			if constexpr (31 < width) if constexpr (constexpr auto pos = shift_mask(4, 31); pos >= 0) r |= ((p >> pos) & 1) << 31;
		}
		if constexpr (mask(5)) {
			const uint16_t p = (GPIOH->ODR & mask(5)) ^ inverted(5);
			if constexpr (0 < width) if constexpr (constexpr auto pos = shift_mask(5, 0); pos >= 0) r |= ((p >> pos) & 1) << 0;
			if constexpr (1 < width) if constexpr (constexpr auto pos = shift_mask(5, 1); pos >= 0) r |= ((p >> pos) & 1) << 1;
			if constexpr (2 < width) if constexpr (constexpr auto pos = shift_mask(5, 2); pos >= 0) r |= ((p >> pos) & 1) << 2;
			if constexpr (3 < width) if constexpr (constexpr auto pos = shift_mask(5, 3); pos >= 0) r |= ((p >> pos) & 1) << 3;
			if constexpr (4 < width) if constexpr (constexpr auto pos = shift_mask(5, 4); pos >= 0) r |= ((p >> pos) & 1) << 4;
			if constexpr (5 < width) if constexpr (constexpr auto pos = shift_mask(5, 5); pos >= 0) r |= ((p >> pos) & 1) << 5;
			if constexpr (6 < width) if constexpr (constexpr auto pos = shift_mask(5, 6); pos >= 0) r |= ((p >> pos) & 1) << 6;
			if constexpr (7 < width) if constexpr (constexpr auto pos = shift_mask(5, 7); pos >= 0) r |= ((p >> pos) & 1) << 7;
			if constexpr (8 < width) if constexpr (constexpr auto pos = shift_mask(5, 8); pos >= 0) r |= ((p >> pos) & 1) << 8;
			if constexpr (9 < width) if constexpr (constexpr auto pos = shift_mask(5, 9); pos >= 0) r |= ((p >> pos) & 1) << 9;
			if constexpr (10 < width) if constexpr (constexpr auto pos = shift_mask(5, 10); pos >= 0) r |= ((p >> pos) & 1) << 10;
			if constexpr (11 < width) if constexpr (constexpr auto pos = shift_mask(5, 11); pos >= 0) r |= ((p >> pos) & 1) << 11;
			if constexpr (12 < width) if constexpr (constexpr auto pos = shift_mask(5, 12); pos >= 0) r |= ((p >> pos) & 1) << 12;
			if constexpr (13 < width) if constexpr (constexpr auto pos = shift_mask(5, 13); pos >= 0) r |= ((p >> pos) & 1) << 13;
			if constexpr (14 < width) if constexpr (constexpr auto pos = shift_mask(5, 14); pos >= 0) r |= ((p >> pos) & 1) << 14;
			if constexpr (15 < width) if constexpr (constexpr auto pos = shift_mask(5, 15); pos >= 0) r |= ((p >> pos) & 1) << 15;
			if constexpr (16 < width) if constexpr (constexpr auto pos = shift_mask(5, 16); pos >= 0) r |= ((p >> pos) & 1) << 16;
			if constexpr (17 < width) if constexpr (constexpr auto pos = shift_mask(5, 17); pos >= 0) r |= ((p >> pos) & 1) << 17;
			if constexpr (18 < width) if constexpr (constexpr auto pos = shift_mask(5, 18); pos >= 0) r |= ((p >> pos) & 1) << 18;
			if constexpr (19 < width) if constexpr (constexpr auto pos = shift_mask(5, 19); pos >= 0) r |= ((p >> pos) & 1) << 19;
			if constexpr (20 < width) if constexpr (constexpr auto pos = shift_mask(5, 20); pos >= 0) r |= ((p >> pos) & 1) << 20;
			if constexpr (21 < width) if constexpr (constexpr auto pos = shift_mask(5, 21); pos >= 0) r |= ((p >> pos) & 1) << 21;
			if constexpr (22 < width) if constexpr (constexpr auto pos = shift_mask(5, 22); pos >= 0) r |= ((p >> pos) & 1) << 22;
			if constexpr (23 < width) if constexpr (constexpr auto pos = shift_mask(5, 23); pos >= 0) r |= ((p >> pos) & 1) << 23;
			if constexpr (24 < width) if constexpr (constexpr auto pos = shift_mask(5, 24); pos >= 0) r |= ((p >> pos) & 1) << 24;
			if constexpr (25 < width) if constexpr (constexpr auto pos = shift_mask(5, 25); pos >= 0) r |= ((p >> pos) & 1) << 25;
			if constexpr (26 < width) if constexpr (constexpr auto pos = shift_mask(5, 26); pos >= 0) r |= ((p >> pos) & 1) << 26;
			if constexpr (27 < width) if constexpr (constexpr auto pos = shift_mask(5, 27); pos >= 0) r |= ((p >> pos) & 1) << 27;
			if constexpr (28 < width) if constexpr (constexpr auto pos = shift_mask(5, 28); pos >= 0) r |= ((p >> pos) & 1) << 28;
			if constexpr (29 < width) if constexpr (constexpr auto pos = shift_mask(5, 29); pos >= 0) r |= ((p >> pos) & 1) << 29;
			if constexpr (30 < width) if constexpr (constexpr auto pos = shift_mask(5, 30); pos >= 0) r |= ((p >> pos) & 1) << 30;
			if constexpr (31 < width) if constexpr (constexpr auto pos = shift_mask(5, 31); pos >= 0) r |= ((p >> pos) & 1) << 31;
		}
		return r;
	}

	static void write(PortType data)
	{
		if constexpr (mask(0)) { uint32_t p{0};
			if constexpr (0 < width) if constexpr (constexpr auto pos = shift_mask(0, 0); pos >= 0) p |= (data & (1ul << 0)) ? (1ul << pos) : (1ul << (pos + 16));
			if constexpr (1 < width) if constexpr (constexpr auto pos = shift_mask(0, 1); pos >= 0) p |= (data & (1ul << 1)) ? (1ul << pos) : (1ul << (pos + 16));
			if constexpr (2 < width) if constexpr (constexpr auto pos = shift_mask(0, 2); pos >= 0) p |= (data & (1ul << 2)) ? (1ul << pos) : (1ul << (pos + 16));
			if constexpr (3 < width) if constexpr (constexpr auto pos = shift_mask(0, 3); pos >= 0) p |= (data & (1ul << 3)) ? (1ul << pos) : (1ul << (pos + 16));
			if constexpr (4 < width) if constexpr (constexpr auto pos = shift_mask(0, 4); pos >= 0) p |= (data & (1ul << 4)) ? (1ul << pos) : (1ul << (pos + 16));
			if constexpr (5 < width) if constexpr (constexpr auto pos = shift_mask(0, 5); pos >= 0) p |= (data & (1ul << 5)) ? (1ul << pos) : (1ul << (pos + 16));
			if constexpr (6 < width) if constexpr (constexpr auto pos = shift_mask(0, 6); pos >= 0) p |= (data & (1ul << 6)) ? (1ul << pos) : (1ul << (pos + 16));
			if constexpr (7 < width) if constexpr (constexpr auto pos = shift_mask(0, 7); pos >= 0) p |= (data & (1ul << 7)) ? (1ul << pos) : (1ul << (pos + 16));
			if constexpr (8 < width) if constexpr (constexpr auto pos = shift_mask(0, 8); pos >= 0) p |= (data & (1ul << 8)) ? (1ul << pos) : (1ul << (pos + 16));
			if constexpr (9 < width) if constexpr (constexpr auto pos = shift_mask(0, 9); pos >= 0) p |= (data & (1ul << 9)) ? (1ul << pos) : (1ul << (pos + 16));
			if constexpr (10 < width) if constexpr (constexpr auto pos = shift_mask(0, 10); pos >= 0) p |= (data & (1ul << 10)) ? (1ul << pos) : (1ul << (pos + 16));
			if constexpr (11 < width) if constexpr (constexpr auto pos = shift_mask(0, 11); pos >= 0) p |= (data & (1ul << 11)) ? (1ul << pos) : (1ul << (pos + 16));
			if constexpr (12 < width) if constexpr (constexpr auto pos = shift_mask(0, 12); pos >= 0) p |= (data & (1ul << 12)) ? (1ul << pos) : (1ul << (pos + 16));
			if constexpr (13 < width) if constexpr (constexpr auto pos = shift_mask(0, 13); pos >= 0) p |= (data & (1ul << 13)) ? (1ul << pos) : (1ul << (pos + 16));
			if constexpr (14 < width) if constexpr (constexpr auto pos = shift_mask(0, 14); pos >= 0) p |= (data & (1ul << 14)) ? (1ul << pos) : (1ul << (pos + 16));
			if constexpr (15 < width) if constexpr (constexpr auto pos = shift_mask(0, 15); pos >= 0) p |= (data & (1ul << 15)) ? (1ul << pos) : (1ul << (pos + 16));
			if constexpr (16 < width) if constexpr (constexpr auto pos = shift_mask(0, 16); pos >= 0) p |= (data & (1ul << 16)) ? (1ul << pos) : (1ul << (pos + 16));
			if constexpr (17 < width) if constexpr (constexpr auto pos = shift_mask(0, 17); pos >= 0) p |= (data & (1ul << 17)) ? (1ul << pos) : (1ul << (pos + 16));
			if constexpr (18 < width) if constexpr (constexpr auto pos = shift_mask(0, 18); pos >= 0) p |= (data & (1ul << 18)) ? (1ul << pos) : (1ul << (pos + 16));
			if constexpr (19 < width) if constexpr (constexpr auto pos = shift_mask(0, 19); pos >= 0) p |= (data & (1ul << 19)) ? (1ul << pos) : (1ul << (pos + 16));
			if constexpr (20 < width) if constexpr (constexpr auto pos = shift_mask(0, 20); pos >= 0) p |= (data & (1ul << 20)) ? (1ul << pos) : (1ul << (pos + 16));
			if constexpr (21 < width) if constexpr (constexpr auto pos = shift_mask(0, 21); pos >= 0) p |= (data & (1ul << 21)) ? (1ul << pos) : (1ul << (pos + 16));
			if constexpr (22 < width) if constexpr (constexpr auto pos = shift_mask(0, 22); pos >= 0) p |= (data & (1ul << 22)) ? (1ul << pos) : (1ul << (pos + 16));
			if constexpr (23 < width) if constexpr (constexpr auto pos = shift_mask(0, 23); pos >= 0) p |= (data & (1ul << 23)) ? (1ul << pos) : (1ul << (pos + 16));
			if constexpr (24 < width) if constexpr (constexpr auto pos = shift_mask(0, 24); pos >= 0) p |= (data & (1ul << 24)) ? (1ul << pos) : (1ul << (pos + 16));
			if constexpr (25 < width) if constexpr (constexpr auto pos = shift_mask(0, 25); pos >= 0) p |= (data & (1ul << 25)) ? (1ul << pos) : (1ul << (pos + 16));
			if constexpr (26 < width) if constexpr (constexpr auto pos = shift_mask(0, 26); pos >= 0) p |= (data & (1ul << 26)) ? (1ul << pos) : (1ul << (pos + 16));
			if constexpr (27 < width) if constexpr (constexpr auto pos = shift_mask(0, 27); pos >= 0) p |= (data & (1ul << 27)) ? (1ul << pos) : (1ul << (pos + 16));
			if constexpr (28 < width) if constexpr (constexpr auto pos = shift_mask(0, 28); pos >= 0) p |= (data & (1ul << 28)) ? (1ul << pos) : (1ul << (pos + 16));
			if constexpr (29 < width) if constexpr (constexpr auto pos = shift_mask(0, 29); pos >= 0) p |= (data & (1ul << 29)) ? (1ul << pos) : (1ul << (pos + 16));
			if constexpr (30 < width) if constexpr (constexpr auto pos = shift_mask(0, 30); pos >= 0) p |= (data & (1ul << 30)) ? (1ul << pos) : (1ul << (pos + 16));
			if constexpr (31 < width) if constexpr (constexpr auto pos = shift_mask(0, 31); pos >= 0) p |= (data & (1ul << 31)) ? (1ul << pos) : (1ul << (pos + 16));
			p ^= inverted(0);
			GPIOA->BSRR = ((~p & mask(0)) << 16) | p;
		}
		if constexpr (mask(1)) { uint32_t p{0};
			if constexpr (0 < width) if constexpr (constexpr auto pos = shift_mask(1, 0); pos >= 0) p |= (data & (1ul << 0)) ? (1ul << pos) : (1ul << (pos + 16));
			if constexpr (1 < width) if constexpr (constexpr auto pos = shift_mask(1, 1); pos >= 0) p |= (data & (1ul << 1)) ? (1ul << pos) : (1ul << (pos + 16));
			if constexpr (2 < width) if constexpr (constexpr auto pos = shift_mask(1, 2); pos >= 0) p |= (data & (1ul << 2)) ? (1ul << pos) : (1ul << (pos + 16));
			if constexpr (3 < width) if constexpr (constexpr auto pos = shift_mask(1, 3); pos >= 0) p |= (data & (1ul << 3)) ? (1ul << pos) : (1ul << (pos + 16));
			if constexpr (4 < width) if constexpr (constexpr auto pos = shift_mask(1, 4); pos >= 0) p |= (data & (1ul << 4)) ? (1ul << pos) : (1ul << (pos + 16));
			if constexpr (5 < width) if constexpr (constexpr auto pos = shift_mask(1, 5); pos >= 0) p |= (data & (1ul << 5)) ? (1ul << pos) : (1ul << (pos + 16));
			if constexpr (6 < width) if constexpr (constexpr auto pos = shift_mask(1, 6); pos >= 0) p |= (data & (1ul << 6)) ? (1ul << pos) : (1ul << (pos + 16));
			if constexpr (7 < width) if constexpr (constexpr auto pos = shift_mask(1, 7); pos >= 0) p |= (data & (1ul << 7)) ? (1ul << pos) : (1ul << (pos + 16));
			if constexpr (8 < width) if constexpr (constexpr auto pos = shift_mask(1, 8); pos >= 0) p |= (data & (1ul << 8)) ? (1ul << pos) : (1ul << (pos + 16));
			if constexpr (9 < width) if constexpr (constexpr auto pos = shift_mask(1, 9); pos >= 0) p |= (data & (1ul << 9)) ? (1ul << pos) : (1ul << (pos + 16));
			if constexpr (10 < width) if constexpr (constexpr auto pos = shift_mask(1, 10); pos >= 0) p |= (data & (1ul << 10)) ? (1ul << pos) : (1ul << (pos + 16));
			if constexpr (11 < width) if constexpr (constexpr auto pos = shift_mask(1, 11); pos >= 0) p |= (data & (1ul << 11)) ? (1ul << pos) : (1ul << (pos + 16));
			if constexpr (12 < width) if constexpr (constexpr auto pos = shift_mask(1, 12); pos >= 0) p |= (data & (1ul << 12)) ? (1ul << pos) : (1ul << (pos + 16));
			if constexpr (13 < width) if constexpr (constexpr auto pos = shift_mask(1, 13); pos >= 0) p |= (data & (1ul << 13)) ? (1ul << pos) : (1ul << (pos + 16));
			if constexpr (14 < width) if constexpr (constexpr auto pos = shift_mask(1, 14); pos >= 0) p |= (data & (1ul << 14)) ? (1ul << pos) : (1ul << (pos + 16));
			if constexpr (15 < width) if constexpr (constexpr auto pos = shift_mask(1, 15); pos >= 0) p |= (data & (1ul << 15)) ? (1ul << pos) : (1ul << (pos + 16));
			if constexpr (16 < width) if constexpr (constexpr auto pos = shift_mask(1, 16); pos >= 0) p |= (data & (1ul << 16)) ? (1ul << pos) : (1ul << (pos + 16));
			if constexpr (17 < width) if constexpr (constexpr auto pos = shift_mask(1, 17); pos >= 0) p |= (data & (1ul << 17)) ? (1ul << pos) : (1ul << (pos + 16));
			if constexpr (18 < width) if constexpr (constexpr auto pos = shift_mask(1, 18); pos >= 0) p |= (data & (1ul << 18)) ? (1ul << pos) : (1ul << (pos + 16));
			if constexpr (19 < width) if constexpr (constexpr auto pos = shift_mask(1, 19); pos >= 0) p |= (data & (1ul << 19)) ? (1ul << pos) : (1ul << (pos + 16));
			if constexpr (20 < width) if constexpr (constexpr auto pos = shift_mask(1, 20); pos >= 0) p |= (data & (1ul << 20)) ? (1ul << pos) : (1ul << (pos + 16));
			if constexpr (21 < width) if constexpr (constexpr auto pos = shift_mask(1, 21); pos >= 0) p |= (data & (1ul << 21)) ? (1ul << pos) : (1ul << (pos + 16));
			if constexpr (22 < width) if constexpr (constexpr auto pos = shift_mask(1, 22); pos >= 0) p |= (data & (1ul << 22)) ? (1ul << pos) : (1ul << (pos + 16));
			if constexpr (23 < width) if constexpr (constexpr auto pos = shift_mask(1, 23); pos >= 0) p |= (data & (1ul << 23)) ? (1ul << pos) : (1ul << (pos + 16));
			if constexpr (24 < width) if constexpr (constexpr auto pos = shift_mask(1, 24); pos >= 0) p |= (data & (1ul << 24)) ? (1ul << pos) : (1ul << (pos + 16));
			if constexpr (25 < width) if constexpr (constexpr auto pos = shift_mask(1, 25); pos >= 0) p |= (data & (1ul << 25)) ? (1ul << pos) : (1ul << (pos + 16));
			if constexpr (26 < width) if constexpr (constexpr auto pos = shift_mask(1, 26); pos >= 0) p |= (data & (1ul << 26)) ? (1ul << pos) : (1ul << (pos + 16));
			if constexpr (27 < width) if constexpr (constexpr auto pos = shift_mask(1, 27); pos >= 0) p |= (data & (1ul << 27)) ? (1ul << pos) : (1ul << (pos + 16));
			if constexpr (28 < width) if constexpr (constexpr auto pos = shift_mask(1, 28); pos >= 0) p |= (data & (1ul << 28)) ? (1ul << pos) : (1ul << (pos + 16));
			if constexpr (29 < width) if constexpr (constexpr auto pos = shift_mask(1, 29); pos >= 0) p |= (data & (1ul << 29)) ? (1ul << pos) : (1ul << (pos + 16));
			if constexpr (30 < width) if constexpr (constexpr auto pos = shift_mask(1, 30); pos >= 0) p |= (data & (1ul << 30)) ? (1ul << pos) : (1ul << (pos + 16));
			if constexpr (31 < width) if constexpr (constexpr auto pos = shift_mask(1, 31); pos >= 0) p |= (data & (1ul << 31)) ? (1ul << pos) : (1ul << (pos + 16));
			p ^= inverted(1);
			GPIOB->BSRR = ((~p & mask(1)) << 16) | p;
		}
		if constexpr (mask(2)) { uint32_t p{0};
			if constexpr (0 < width) if constexpr (constexpr auto pos = shift_mask(2, 0); pos >= 0) p |= (data & (1ul << 0)) ? (1ul << pos) : (1ul << (pos + 16));
			if constexpr (1 < width) if constexpr (constexpr auto pos = shift_mask(2, 1); pos >= 0) p |= (data & (1ul << 1)) ? (1ul << pos) : (1ul << (pos + 16));
			if constexpr (2 < width) if constexpr (constexpr auto pos = shift_mask(2, 2); pos >= 0) p |= (data & (1ul << 2)) ? (1ul << pos) : (1ul << (pos + 16));
			if constexpr (3 < width) if constexpr (constexpr auto pos = shift_mask(2, 3); pos >= 0) p |= (data & (1ul << 3)) ? (1ul << pos) : (1ul << (pos + 16));
			if constexpr (4 < width) if constexpr (constexpr auto pos = shift_mask(2, 4); pos >= 0) p |= (data & (1ul << 4)) ? (1ul << pos) : (1ul << (pos + 16));
			if constexpr (5 < width) if constexpr (constexpr auto pos = shift_mask(2, 5); pos >= 0) p |= (data & (1ul << 5)) ? (1ul << pos) : (1ul << (pos + 16));
			if constexpr (6 < width) if constexpr (constexpr auto pos = shift_mask(2, 6); pos >= 0) p |= (data & (1ul << 6)) ? (1ul << pos) : (1ul << (pos + 16));
			if constexpr (7 < width) if constexpr (constexpr auto pos = shift_mask(2, 7); pos >= 0) p |= (data & (1ul << 7)) ? (1ul << pos) : (1ul << (pos + 16));
			if constexpr (8 < width) if constexpr (constexpr auto pos = shift_mask(2, 8); pos >= 0) p |= (data & (1ul << 8)) ? (1ul << pos) : (1ul << (pos + 16));
			if constexpr (9 < width) if constexpr (constexpr auto pos = shift_mask(2, 9); pos >= 0) p |= (data & (1ul << 9)) ? (1ul << pos) : (1ul << (pos + 16));
			if constexpr (10 < width) if constexpr (constexpr auto pos = shift_mask(2, 10); pos >= 0) p |= (data & (1ul << 10)) ? (1ul << pos) : (1ul << (pos + 16));
			if constexpr (11 < width) if constexpr (constexpr auto pos = shift_mask(2, 11); pos >= 0) p |= (data & (1ul << 11)) ? (1ul << pos) : (1ul << (pos + 16));
			if constexpr (12 < width) if constexpr (constexpr auto pos = shift_mask(2, 12); pos >= 0) p |= (data & (1ul << 12)) ? (1ul << pos) : (1ul << (pos + 16));
			if constexpr (13 < width) if constexpr (constexpr auto pos = shift_mask(2, 13); pos >= 0) p |= (data & (1ul << 13)) ? (1ul << pos) : (1ul << (pos + 16));
			if constexpr (14 < width) if constexpr (constexpr auto pos = shift_mask(2, 14); pos >= 0) p |= (data & (1ul << 14)) ? (1ul << pos) : (1ul << (pos + 16));
			if constexpr (15 < width) if constexpr (constexpr auto pos = shift_mask(2, 15); pos >= 0) p |= (data & (1ul << 15)) ? (1ul << pos) : (1ul << (pos + 16));
			if constexpr (16 < width) if constexpr (constexpr auto pos = shift_mask(2, 16); pos >= 0) p |= (data & (1ul << 16)) ? (1ul << pos) : (1ul << (pos + 16));
			if constexpr (17 < width) if constexpr (constexpr auto pos = shift_mask(2, 17); pos >= 0) p |= (data & (1ul << 17)) ? (1ul << pos) : (1ul << (pos + 16));
			if constexpr (18 < width) if constexpr (constexpr auto pos = shift_mask(2, 18); pos >= 0) p |= (data & (1ul << 18)) ? (1ul << pos) : (1ul << (pos + 16));
			if constexpr (19 < width) if constexpr (constexpr auto pos = shift_mask(2, 19); pos >= 0) p |= (data & (1ul << 19)) ? (1ul << pos) : (1ul << (pos + 16));
			if constexpr (20 < width) if constexpr (constexpr auto pos = shift_mask(2, 20); pos >= 0) p |= (data & (1ul << 20)) ? (1ul << pos) : (1ul << (pos + 16));
			if constexpr (21 < width) if constexpr (constexpr auto pos = shift_mask(2, 21); pos >= 0) p |= (data & (1ul << 21)) ? (1ul << pos) : (1ul << (pos + 16));
			if constexpr (22 < width) if constexpr (constexpr auto pos = shift_mask(2, 22); pos >= 0) p |= (data & (1ul << 22)) ? (1ul << pos) : (1ul << (pos + 16));
			if constexpr (23 < width) if constexpr (constexpr auto pos = shift_mask(2, 23); pos >= 0) p |= (data & (1ul << 23)) ? (1ul << pos) : (1ul << (pos + 16));
			if constexpr (24 < width) if constexpr (constexpr auto pos = shift_mask(2, 24); pos >= 0) p |= (data & (1ul << 24)) ? (1ul << pos) : (1ul << (pos + 16));
			if constexpr (25 < width) if constexpr (constexpr auto pos = shift_mask(2, 25); pos >= 0) p |= (data & (1ul << 25)) ? (1ul << pos) : (1ul << (pos + 16));
			if constexpr (26 < width) if constexpr (constexpr auto pos = shift_mask(2, 26); pos >= 0) p |= (data & (1ul << 26)) ? (1ul << pos) : (1ul << (pos + 16));
			if constexpr (27 < width) if constexpr (constexpr auto pos = shift_mask(2, 27); pos >= 0) p |= (data & (1ul << 27)) ? (1ul << pos) : (1ul << (pos + 16));
			if constexpr (28 < width) if constexpr (constexpr auto pos = shift_mask(2, 28); pos >= 0) p |= (data & (1ul << 28)) ? (1ul << pos) : (1ul << (pos + 16));
			if constexpr (29 < width) if constexpr (constexpr auto pos = shift_mask(2, 29); pos >= 0) p |= (data & (1ul << 29)) ? (1ul << pos) : (1ul << (pos + 16));
			if constexpr (30 < width) if constexpr (constexpr auto pos = shift_mask(2, 30); pos >= 0) p |= (data & (1ul << 30)) ? (1ul << pos) : (1ul << (pos + 16));
			if constexpr (31 < width) if constexpr (constexpr auto pos = shift_mask(2, 31); pos >= 0) p |= (data & (1ul << 31)) ? (1ul << pos) : (1ul << (pos + 16));
			p ^= inverted(2);
			GPIOC->BSRR = ((~p & mask(2)) << 16) | p;
		}
		if constexpr (mask(3)) { uint32_t p{0};
			if constexpr (0 < width) if constexpr (constexpr auto pos = shift_mask(3, 0); pos >= 0) p |= (data & (1ul << 0)) ? (1ul << pos) : (1ul << (pos + 16));
			if constexpr (1 < width) if constexpr (constexpr auto pos = shift_mask(3, 1); pos >= 0) p |= (data & (1ul << 1)) ? (1ul << pos) : (1ul << (pos + 16));
			if constexpr (2 < width) if constexpr (constexpr auto pos = shift_mask(3, 2); pos >= 0) p |= (data & (1ul << 2)) ? (1ul << pos) : (1ul << (pos + 16));
			if constexpr (3 < width) if constexpr (constexpr auto pos = shift_mask(3, 3); pos >= 0) p |= (data & (1ul << 3)) ? (1ul << pos) : (1ul << (pos + 16));
			if constexpr (4 < width) if constexpr (constexpr auto pos = shift_mask(3, 4); pos >= 0) p |= (data & (1ul << 4)) ? (1ul << pos) : (1ul << (pos + 16));
			if constexpr (5 < width) if constexpr (constexpr auto pos = shift_mask(3, 5); pos >= 0) p |= (data & (1ul << 5)) ? (1ul << pos) : (1ul << (pos + 16));
			if constexpr (6 < width) if constexpr (constexpr auto pos = shift_mask(3, 6); pos >= 0) p |= (data & (1ul << 6)) ? (1ul << pos) : (1ul << (pos + 16));
			if constexpr (7 < width) if constexpr (constexpr auto pos = shift_mask(3, 7); pos >= 0) p |= (data & (1ul << 7)) ? (1ul << pos) : (1ul << (pos + 16));
			if constexpr (8 < width) if constexpr (constexpr auto pos = shift_mask(3, 8); pos >= 0) p |= (data & (1ul << 8)) ? (1ul << pos) : (1ul << (pos + 16));
			if constexpr (9 < width) if constexpr (constexpr auto pos = shift_mask(3, 9); pos >= 0) p |= (data & (1ul << 9)) ? (1ul << pos) : (1ul << (pos + 16));
			if constexpr (10 < width) if constexpr (constexpr auto pos = shift_mask(3, 10); pos >= 0) p |= (data & (1ul << 10)) ? (1ul << pos) : (1ul << (pos + 16));
			if constexpr (11 < width) if constexpr (constexpr auto pos = shift_mask(3, 11); pos >= 0) p |= (data & (1ul << 11)) ? (1ul << pos) : (1ul << (pos + 16));
			if constexpr (12 < width) if constexpr (constexpr auto pos = shift_mask(3, 12); pos >= 0) p |= (data & (1ul << 12)) ? (1ul << pos) : (1ul << (pos + 16));
			if constexpr (13 < width) if constexpr (constexpr auto pos = shift_mask(3, 13); pos >= 0) p |= (data & (1ul << 13)) ? (1ul << pos) : (1ul << (pos + 16));
			if constexpr (14 < width) if constexpr (constexpr auto pos = shift_mask(3, 14); pos >= 0) p |= (data & (1ul << 14)) ? (1ul << pos) : (1ul << (pos + 16));
			if constexpr (15 < width) if constexpr (constexpr auto pos = shift_mask(3, 15); pos >= 0) p |= (data & (1ul << 15)) ? (1ul << pos) : (1ul << (pos + 16));
			if constexpr (16 < width) if constexpr (constexpr auto pos = shift_mask(3, 16); pos >= 0) p |= (data & (1ul << 16)) ? (1ul << pos) : (1ul << (pos + 16));
			if constexpr (17 < width) if constexpr (constexpr auto pos = shift_mask(3, 17); pos >= 0) p |= (data & (1ul << 17)) ? (1ul << pos) : (1ul << (pos + 16));
			if constexpr (18 < width) if constexpr (constexpr auto pos = shift_mask(3, 18); pos >= 0) p |= (data & (1ul << 18)) ? (1ul << pos) : (1ul << (pos + 16));
			if constexpr (19 < width) if constexpr (constexpr auto pos = shift_mask(3, 19); pos >= 0) p |= (data & (1ul << 19)) ? (1ul << pos) : (1ul << (pos + 16));
			if constexpr (20 < width) if constexpr (constexpr auto pos = shift_mask(3, 20); pos >= 0) p |= (data & (1ul << 20)) ? (1ul << pos) : (1ul << (pos + 16));
			if constexpr (21 < width) if constexpr (constexpr auto pos = shift_mask(3, 21); pos >= 0) p |= (data & (1ul << 21)) ? (1ul << pos) : (1ul << (pos + 16));
			if constexpr (22 < width) if constexpr (constexpr auto pos = shift_mask(3, 22); pos >= 0) p |= (data & (1ul << 22)) ? (1ul << pos) : (1ul << (pos + 16));
			if constexpr (23 < width) if constexpr (constexpr auto pos = shift_mask(3, 23); pos >= 0) p |= (data & (1ul << 23)) ? (1ul << pos) : (1ul << (pos + 16));
			if constexpr (24 < width) if constexpr (constexpr auto pos = shift_mask(3, 24); pos >= 0) p |= (data & (1ul << 24)) ? (1ul << pos) : (1ul << (pos + 16));
			if constexpr (25 < width) if constexpr (constexpr auto pos = shift_mask(3, 25); pos >= 0) p |= (data & (1ul << 25)) ? (1ul << pos) : (1ul << (pos + 16));
			if constexpr (26 < width) if constexpr (constexpr auto pos = shift_mask(3, 26); pos >= 0) p |= (data & (1ul << 26)) ? (1ul << pos) : (1ul << (pos + 16));
			if constexpr (27 < width) if constexpr (constexpr auto pos = shift_mask(3, 27); pos >= 0) p |= (data & (1ul << 27)) ? (1ul << pos) : (1ul << (pos + 16));
			if constexpr (28 < width) if constexpr (constexpr auto pos = shift_mask(3, 28); pos >= 0) p |= (data & (1ul << 28)) ? (1ul << pos) : (1ul << (pos + 16));
			if constexpr (29 < width) if constexpr (constexpr auto pos = shift_mask(3, 29); pos >= 0) p |= (data & (1ul << 29)) ? (1ul << pos) : (1ul << (pos + 16));
			if constexpr (30 < width) if constexpr (constexpr auto pos = shift_mask(3, 30); pos >= 0) p |= (data & (1ul << 30)) ? (1ul << pos) : (1ul << (pos + 16));
			if constexpr (31 < width) if constexpr (constexpr auto pos = shift_mask(3, 31); pos >= 0) p |= (data & (1ul << 31)) ? (1ul << pos) : (1ul << (pos + 16));
			p ^= inverted(3);
			GPIOD->BSRR = ((~p & mask(3)) << 16) | p;
		}
		if constexpr (mask(4)) { uint32_t p{0};
			if constexpr (0 < width) if constexpr (constexpr auto pos = shift_mask(4, 0); pos >= 0) p |= (data & (1ul << 0)) ? (1ul << pos) : (1ul << (pos + 16));
			if constexpr (1 < width) if constexpr (constexpr auto pos = shift_mask(4, 1); pos >= 0) p |= (data & (1ul << 1)) ? (1ul << pos) : (1ul << (pos + 16));
			if constexpr (2 < width) if constexpr (constexpr auto pos = shift_mask(4, 2); pos >= 0) p |= (data & (1ul << 2)) ? (1ul << pos) : (1ul << (pos + 16));
			if constexpr (3 < width) if constexpr (constexpr auto pos = shift_mask(4, 3); pos >= 0) p |= (data & (1ul << 3)) ? (1ul << pos) : (1ul << (pos + 16));
			if constexpr (4 < width) if constexpr (constexpr auto pos = shift_mask(4, 4); pos >= 0) p |= (data & (1ul << 4)) ? (1ul << pos) : (1ul << (pos + 16));
			if constexpr (5 < width) if constexpr (constexpr auto pos = shift_mask(4, 5); pos >= 0) p |= (data & (1ul << 5)) ? (1ul << pos) : (1ul << (pos + 16));
			if constexpr (6 < width) if constexpr (constexpr auto pos = shift_mask(4, 6); pos >= 0) p |= (data & (1ul << 6)) ? (1ul << pos) : (1ul << (pos + 16));
			if constexpr (7 < width) if constexpr (constexpr auto pos = shift_mask(4, 7); pos >= 0) p |= (data & (1ul << 7)) ? (1ul << pos) : (1ul << (pos + 16));
			if constexpr (8 < width) if constexpr (constexpr auto pos = shift_mask(4, 8); pos >= 0) p |= (data & (1ul << 8)) ? (1ul << pos) : (1ul << (pos + 16));
			if constexpr (9 < width) if constexpr (constexpr auto pos = shift_mask(4, 9); pos >= 0) p |= (data & (1ul << 9)) ? (1ul << pos) : (1ul << (pos + 16));
			if constexpr (10 < width) if constexpr (constexpr auto pos = shift_mask(4, 10); pos >= 0) p |= (data & (1ul << 10)) ? (1ul << pos) : (1ul << (pos + 16));
			if constexpr (11 < width) if constexpr (constexpr auto pos = shift_mask(4, 11); pos >= 0) p |= (data & (1ul << 11)) ? (1ul << pos) : (1ul << (pos + 16));
			if constexpr (12 < width) if constexpr (constexpr auto pos = shift_mask(4, 12); pos >= 0) p |= (data & (1ul << 12)) ? (1ul << pos) : (1ul << (pos + 16));
			if constexpr (13 < width) if constexpr (constexpr auto pos = shift_mask(4, 13); pos >= 0) p |= (data & (1ul << 13)) ? (1ul << pos) : (1ul << (pos + 16));
			if constexpr (14 < width) if constexpr (constexpr auto pos = shift_mask(4, 14); pos >= 0) p |= (data & (1ul << 14)) ? (1ul << pos) : (1ul << (pos + 16));
			if constexpr (15 < width) if constexpr (constexpr auto pos = shift_mask(4, 15); pos >= 0) p |= (data & (1ul << 15)) ? (1ul << pos) : (1ul << (pos + 16));
			if constexpr (16 < width) if constexpr (constexpr auto pos = shift_mask(4, 16); pos >= 0) p |= (data & (1ul << 16)) ? (1ul << pos) : (1ul << (pos + 16));
			if constexpr (17 < width) if constexpr (constexpr auto pos = shift_mask(4, 17); pos >= 0) p |= (data & (1ul << 17)) ? (1ul << pos) : (1ul << (pos + 16));
			if constexpr (18 < width) if constexpr (constexpr auto pos = shift_mask(4, 18); pos >= 0) p |= (data & (1ul << 18)) ? (1ul << pos) : (1ul << (pos + 16));
			if constexpr (19 < width) if constexpr (constexpr auto pos = shift_mask(4, 19); pos >= 0) p |= (data & (1ul << 19)) ? (1ul << pos) : (1ul << (pos + 16));
			if constexpr (20 < width) if constexpr (constexpr auto pos = shift_mask(4, 20); pos >= 0) p |= (data & (1ul << 20)) ? (1ul << pos) : (1ul << (pos + 16));
			if constexpr (21 < width) if constexpr (constexpr auto pos = shift_mask(4, 21); pos >= 0) p |= (data & (1ul << 21)) ? (1ul << pos) : (1ul << (pos + 16));
			if constexpr (22 < width) if constexpr (constexpr auto pos = shift_mask(4, 22); pos >= 0) p |= (data & (1ul << 22)) ? (1ul << pos) : (1ul << (pos + 16));
			if constexpr (23 < width) if constexpr (constexpr auto pos = shift_mask(4, 23); pos >= 0) p |= (data & (1ul << 23)) ? (1ul << pos) : (1ul << (pos + 16));
			if constexpr (24 < width) if constexpr (constexpr auto pos = shift_mask(4, 24); pos >= 0) p |= (data & (1ul << 24)) ? (1ul << pos) : (1ul << (pos + 16));
			if constexpr (25 < width) if constexpr (constexpr auto pos = shift_mask(4, 25); pos >= 0) p |= (data & (1ul << 25)) ? (1ul << pos) : (1ul << (pos + 16));
			if constexpr (26 < width) if constexpr (constexpr auto pos = shift_mask(4, 26); pos >= 0) p |= (data & (1ul << 26)) ? (1ul << pos) : (1ul << (pos + 16));
			if constexpr (27 < width) if constexpr (constexpr auto pos = shift_mask(4, 27); pos >= 0) p |= (data & (1ul << 27)) ? (1ul << pos) : (1ul << (pos + 16));
			if constexpr (28 < width) if constexpr (constexpr auto pos = shift_mask(4, 28); pos >= 0) p |= (data & (1ul << 28)) ? (1ul << pos) : (1ul << (pos + 16));
			if constexpr (29 < width) if constexpr (constexpr auto pos = shift_mask(4, 29); pos >= 0) p |= (data & (1ul << 29)) ? (1ul << pos) : (1ul << (pos + 16));
			if constexpr (30 < width) if constexpr (constexpr auto pos = shift_mask(4, 30); pos >= 0) p |= (data & (1ul << 30)) ? (1ul << pos) : (1ul << (pos + 16));
			if constexpr (31 < width) if constexpr (constexpr auto pos = shift_mask(4, 31); pos >= 0) p |= (data & (1ul << 31)) ? (1ul << pos) : (1ul << (pos + 16));
			p ^= inverted(4);
			GPIOE->BSRR = ((~p & mask(4)) << 16) | p;
		}
		if constexpr (mask(5)) { uint32_t p{0};
			if constexpr (0 < width) if constexpr (constexpr auto pos = shift_mask(5, 0); pos >= 0) p |= (data & (1ul << 0)) ? (1ul << pos) : (1ul << (pos + 16));
			if constexpr (1 < width) if constexpr (constexpr auto pos = shift_mask(5, 1); pos >= 0) p |= (data & (1ul << 1)) ? (1ul << pos) : (1ul << (pos + 16));
			if constexpr (2 < width) if constexpr (constexpr auto pos = shift_mask(5, 2); pos >= 0) p |= (data & (1ul << 2)) ? (1ul << pos) : (1ul << (pos + 16));
			if constexpr (3 < width) if constexpr (constexpr auto pos = shift_mask(5, 3); pos >= 0) p |= (data & (1ul << 3)) ? (1ul << pos) : (1ul << (pos + 16));
			if constexpr (4 < width) if constexpr (constexpr auto pos = shift_mask(5, 4); pos >= 0) p |= (data & (1ul << 4)) ? (1ul << pos) : (1ul << (pos + 16));
			if constexpr (5 < width) if constexpr (constexpr auto pos = shift_mask(5, 5); pos >= 0) p |= (data & (1ul << 5)) ? (1ul << pos) : (1ul << (pos + 16));
			if constexpr (6 < width) if constexpr (constexpr auto pos = shift_mask(5, 6); pos >= 0) p |= (data & (1ul << 6)) ? (1ul << pos) : (1ul << (pos + 16));
			if constexpr (7 < width) if constexpr (constexpr auto pos = shift_mask(5, 7); pos >= 0) p |= (data & (1ul << 7)) ? (1ul << pos) : (1ul << (pos + 16));
			if constexpr (8 < width) if constexpr (constexpr auto pos = shift_mask(5, 8); pos >= 0) p |= (data & (1ul << 8)) ? (1ul << pos) : (1ul << (pos + 16));
			if constexpr (9 < width) if constexpr (constexpr auto pos = shift_mask(5, 9); pos >= 0) p |= (data & (1ul << 9)) ? (1ul << pos) : (1ul << (pos + 16));
			if constexpr (10 < width) if constexpr (constexpr auto pos = shift_mask(5, 10); pos >= 0) p |= (data & (1ul << 10)) ? (1ul << pos) : (1ul << (pos + 16));
			if constexpr (11 < width) if constexpr (constexpr auto pos = shift_mask(5, 11); pos >= 0) p |= (data & (1ul << 11)) ? (1ul << pos) : (1ul << (pos + 16));
			if constexpr (12 < width) if constexpr (constexpr auto pos = shift_mask(5, 12); pos >= 0) p |= (data & (1ul << 12)) ? (1ul << pos) : (1ul << (pos + 16));
			if constexpr (13 < width) if constexpr (constexpr auto pos = shift_mask(5, 13); pos >= 0) p |= (data & (1ul << 13)) ? (1ul << pos) : (1ul << (pos + 16));
			if constexpr (14 < width) if constexpr (constexpr auto pos = shift_mask(5, 14); pos >= 0) p |= (data & (1ul << 14)) ? (1ul << pos) : (1ul << (pos + 16));
			if constexpr (15 < width) if constexpr (constexpr auto pos = shift_mask(5, 15); pos >= 0) p |= (data & (1ul << 15)) ? (1ul << pos) : (1ul << (pos + 16));
			if constexpr (16 < width) if constexpr (constexpr auto pos = shift_mask(5, 16); pos >= 0) p |= (data & (1ul << 16)) ? (1ul << pos) : (1ul << (pos + 16));
			if constexpr (17 < width) if constexpr (constexpr auto pos = shift_mask(5, 17); pos >= 0) p |= (data & (1ul << 17)) ? (1ul << pos) : (1ul << (pos + 16));
			if constexpr (18 < width) if constexpr (constexpr auto pos = shift_mask(5, 18); pos >= 0) p |= (data & (1ul << 18)) ? (1ul << pos) : (1ul << (pos + 16));
			if constexpr (19 < width) if constexpr (constexpr auto pos = shift_mask(5, 19); pos >= 0) p |= (data & (1ul << 19)) ? (1ul << pos) : (1ul << (pos + 16));
			if constexpr (20 < width) if constexpr (constexpr auto pos = shift_mask(5, 20); pos >= 0) p |= (data & (1ul << 20)) ? (1ul << pos) : (1ul << (pos + 16));
			if constexpr (21 < width) if constexpr (constexpr auto pos = shift_mask(5, 21); pos >= 0) p |= (data & (1ul << 21)) ? (1ul << pos) : (1ul << (pos + 16));
			if constexpr (22 < width) if constexpr (constexpr auto pos = shift_mask(5, 22); pos >= 0) p |= (data & (1ul << 22)) ? (1ul << pos) : (1ul << (pos + 16));
			if constexpr (23 < width) if constexpr (constexpr auto pos = shift_mask(5, 23); pos >= 0) p |= (data & (1ul << 23)) ? (1ul << pos) : (1ul << (pos + 16));
			if constexpr (24 < width) if constexpr (constexpr auto pos = shift_mask(5, 24); pos >= 0) p |= (data & (1ul << 24)) ? (1ul << pos) : (1ul << (pos + 16));
			if constexpr (25 < width) if constexpr (constexpr auto pos = shift_mask(5, 25); pos >= 0) p |= (data & (1ul << 25)) ? (1ul << pos) : (1ul << (pos + 16));
			if constexpr (26 < width) if constexpr (constexpr auto pos = shift_mask(5, 26); pos >= 0) p |= (data & (1ul << 26)) ? (1ul << pos) : (1ul << (pos + 16));
			if constexpr (27 < width) if constexpr (constexpr auto pos = shift_mask(5, 27); pos >= 0) p |= (data & (1ul << 27)) ? (1ul << pos) : (1ul << (pos + 16));
			if constexpr (28 < width) if constexpr (constexpr auto pos = shift_mask(5, 28); pos >= 0) p |= (data & (1ul << 28)) ? (1ul << pos) : (1ul << (pos + 16));
			if constexpr (29 < width) if constexpr (constexpr auto pos = shift_mask(5, 29); pos >= 0) p |= (data & (1ul << 29)) ? (1ul << pos) : (1ul << (pos + 16));
			if constexpr (30 < width) if constexpr (constexpr auto pos = shift_mask(5, 30); pos >= 0) p |= (data & (1ul << 30)) ? (1ul << pos) : (1ul << (pos + 16));
			if constexpr (31 < width) if constexpr (constexpr auto pos = shift_mask(5, 31); pos >= 0) p |= (data & (1ul << 31)) ? (1ul << pos) : (1ul << (pos + 16));
			p ^= inverted(5);
			GPIOH->BSRR = ((~p & mask(5)) << 16) | p;
		}
	}

	static PortType read()
	{
		PortType r{0};
		if constexpr (mask(0)) {
			const uint16_t p = (GPIOA->IDR & mask(0)) ^ inverted(0);
			if constexpr (0 < width) if constexpr (constexpr auto pos = shift_mask(0, 0); pos >= 0) r |= ((p >> pos) & 1) << 0;
			if constexpr (1 < width) if constexpr (constexpr auto pos = shift_mask(0, 1); pos >= 0) r |= ((p >> pos) & 1) << 1;
			if constexpr (2 < width) if constexpr (constexpr auto pos = shift_mask(0, 2); pos >= 0) r |= ((p >> pos) & 1) << 2;
			if constexpr (3 < width) if constexpr (constexpr auto pos = shift_mask(0, 3); pos >= 0) r |= ((p >> pos) & 1) << 3;
			if constexpr (4 < width) if constexpr (constexpr auto pos = shift_mask(0, 4); pos >= 0) r |= ((p >> pos) & 1) << 4;
			if constexpr (5 < width) if constexpr (constexpr auto pos = shift_mask(0, 5); pos >= 0) r |= ((p >> pos) & 1) << 5;
			if constexpr (6 < width) if constexpr (constexpr auto pos = shift_mask(0, 6); pos >= 0) r |= ((p >> pos) & 1) << 6;
			if constexpr (7 < width) if constexpr (constexpr auto pos = shift_mask(0, 7); pos >= 0) r |= ((p >> pos) & 1) << 7;
			if constexpr (8 < width) if constexpr (constexpr auto pos = shift_mask(0, 8); pos >= 0) r |= ((p >> pos) & 1) << 8;
			if constexpr (9 < width) if constexpr (constexpr auto pos = shift_mask(0, 9); pos >= 0) r |= ((p >> pos) & 1) << 9;
			if constexpr (10 < width) if constexpr (constexpr auto pos = shift_mask(0, 10); pos >= 0) r |= ((p >> pos) & 1) << 10;
			if constexpr (11 < width) if constexpr (constexpr auto pos = shift_mask(0, 11); pos >= 0) r |= ((p >> pos) & 1) << 11;
			if constexpr (12 < width) if constexpr (constexpr auto pos = shift_mask(0, 12); pos >= 0) r |= ((p >> pos) & 1) << 12;
			if constexpr (13 < width) if constexpr (constexpr auto pos = shift_mask(0, 13); pos >= 0) r |= ((p >> pos) & 1) << 13;
			if constexpr (14 < width) if constexpr (constexpr auto pos = shift_mask(0, 14); pos >= 0) r |= ((p >> pos) & 1) << 14;
			if constexpr (15 < width) if constexpr (constexpr auto pos = shift_mask(0, 15); pos >= 0) r |= ((p >> pos) & 1) << 15;
			if constexpr (16 < width) if constexpr (constexpr auto pos = shift_mask(0, 16); pos >= 0) r |= ((p >> pos) & 1) << 16;
			if constexpr (17 < width) if constexpr (constexpr auto pos = shift_mask(0, 17); pos >= 0) r |= ((p >> pos) & 1) << 17;
			if constexpr (18 < width) if constexpr (constexpr auto pos = shift_mask(0, 18); pos >= 0) r |= ((p >> pos) & 1) << 18;
			if constexpr (19 < width) if constexpr (constexpr auto pos = shift_mask(0, 19); pos >= 0) r |= ((p >> pos) & 1) << 19;
			if constexpr (20 < width) if constexpr (constexpr auto pos = shift_mask(0, 20); pos >= 0) r |= ((p >> pos) & 1) << 20;
			if constexpr (21 < width) if constexpr (constexpr auto pos = shift_mask(0, 21); pos >= 0) r |= ((p >> pos) & 1) << 21;
			if constexpr (22 < width) if constexpr (constexpr auto pos = shift_mask(0, 22); pos >= 0) r |= ((p >> pos) & 1) << 22;
			if constexpr (23 < width) if constexpr (constexpr auto pos = shift_mask(0, 23); pos >= 0) r |= ((p >> pos) & 1) << 23;
			if constexpr (24 < width) if constexpr (constexpr auto pos = shift_mask(0, 24); pos >= 0) r |= ((p >> pos) & 1) << 24;
			if constexpr (25 < width) if constexpr (constexpr auto pos = shift_mask(0, 25); pos >= 0) r |= ((p >> pos) & 1) << 25;
			if constexpr (26 < width) if constexpr (constexpr auto pos = shift_mask(0, 26); pos >= 0) r |= ((p >> pos) & 1) << 26;
			if constexpr (27 < width) if constexpr (constexpr auto pos = shift_mask(0, 27); pos >= 0) r |= ((p >> pos) & 1) << 27;
			if constexpr (28 < width) if constexpr (constexpr auto pos = shift_mask(0, 28); pos >= 0) r |= ((p >> pos) & 1) << 28;
			if constexpr (29 < width) if constexpr (constexpr auto pos = shift_mask(0, 29); pos >= 0) r |= ((p >> pos) & 1) << 29;
			if constexpr (30 < width) if constexpr (constexpr auto pos = shift_mask(0, 30); pos >= 0) r |= ((p >> pos) & 1) << 30;
			if constexpr (31 < width) if constexpr (constexpr auto pos = shift_mask(0, 31); pos >= 0) r |= ((p >> pos) & 1) << 31;
		}
		if constexpr (mask(1)) {
			const uint16_t p = (GPIOB->IDR & mask(1)) ^ inverted(1);
			if constexpr (0 < width) if constexpr (constexpr auto pos = shift_mask(1, 0); pos >= 0) r |= ((p >> pos) & 1) << 0;
			if constexpr (1 < width) if constexpr (constexpr auto pos = shift_mask(1, 1); pos >= 0) r |= ((p >> pos) & 1) << 1;
			if constexpr (2 < width) if constexpr (constexpr auto pos = shift_mask(1, 2); pos >= 0) r |= ((p >> pos) & 1) << 2;
			if constexpr (3 < width) if constexpr (constexpr auto pos = shift_mask(1, 3); pos >= 0) r |= ((p >> pos) & 1) << 3;
			if constexpr (4 < width) if constexpr (constexpr auto pos = shift_mask(1, 4); pos >= 0) r |= ((p >> pos) & 1) << 4;
			if constexpr (5 < width) if constexpr (constexpr auto pos = shift_mask(1, 5); pos >= 0) r |= ((p >> pos) & 1) << 5;
			if constexpr (6 < width) if constexpr (constexpr auto pos = shift_mask(1, 6); pos >= 0) r |= ((p >> pos) & 1) << 6;
			if constexpr (7 < width) if constexpr (constexpr auto pos = shift_mask(1, 7); pos >= 0) r |= ((p >> pos) & 1) << 7;
			if constexpr (8 < width) if constexpr (constexpr auto pos = shift_mask(1, 8); pos >= 0) r |= ((p >> pos) & 1) << 8;
			if constexpr (9 < width) if constexpr (constexpr auto pos = shift_mask(1, 9); pos >= 0) r |= ((p >> pos) & 1) << 9;
			if constexpr (10 < width) if constexpr (constexpr auto pos = shift_mask(1, 10); pos >= 0) r |= ((p >> pos) & 1) << 10;
			if constexpr (11 < width) if constexpr (constexpr auto pos = shift_mask(1, 11); pos >= 0) r |= ((p >> pos) & 1) << 11;
			if constexpr (12 < width) if constexpr (constexpr auto pos = shift_mask(1, 12); pos >= 0) r |= ((p >> pos) & 1) << 12;
			if constexpr (13 < width) if constexpr (constexpr auto pos = shift_mask(1, 13); pos >= 0) r |= ((p >> pos) & 1) << 13;
			if constexpr (14 < width) if constexpr (constexpr auto pos = shift_mask(1, 14); pos >= 0) r |= ((p >> pos) & 1) << 14;
			if constexpr (15 < width) if constexpr (constexpr auto pos = shift_mask(1, 15); pos >= 0) r |= ((p >> pos) & 1) << 15;
			if constexpr (16 < width) if constexpr (constexpr auto pos = shift_mask(1, 16); pos >= 0) r |= ((p >> pos) & 1) << 16;
			if constexpr (17 < width) if constexpr (constexpr auto pos = shift_mask(1, 17); pos >= 0) r |= ((p >> pos) & 1) << 17;
			if constexpr (18 < width) if constexpr (constexpr auto pos = shift_mask(1, 18); pos >= 0) r |= ((p >> pos) & 1) << 18;
			if constexpr (19 < width) if constexpr (constexpr auto pos = shift_mask(1, 19); pos >= 0) r |= ((p >> pos) & 1) << 19;
			if constexpr (20 < width) if constexpr (constexpr auto pos = shift_mask(1, 20); pos >= 0) r |= ((p >> pos) & 1) << 20;
			if constexpr (21 < width) if constexpr (constexpr auto pos = shift_mask(1, 21); pos >= 0) r |= ((p >> pos) & 1) << 21;
			if constexpr (22 < width) if constexpr (constexpr auto pos = shift_mask(1, 22); pos >= 0) r |= ((p >> pos) & 1) << 22;
			if constexpr (23 < width) if constexpr (constexpr auto pos = shift_mask(1, 23); pos >= 0) r |= ((p >> pos) & 1) << 23;
			if constexpr (24 < width) if constexpr (constexpr auto pos = shift_mask(1, 24); pos >= 0) r |= ((p >> pos) & 1) << 24;
			if constexpr (25 < width) if constexpr (constexpr auto pos = shift_mask(1, 25); pos >= 0) r |= ((p >> pos) & 1) << 25;
			if constexpr (26 < width) if constexpr (constexpr auto pos = shift_mask(1, 26); pos >= 0) r |= ((p >> pos) & 1) << 26;
			if constexpr (27 < width) if constexpr (constexpr auto pos = shift_mask(1, 27); pos >= 0) r |= ((p >> pos) & 1) << 27;
			if constexpr (28 < width) if constexpr (constexpr auto pos = shift_mask(1, 28); pos >= 0) r |= ((p >> pos) & 1) << 28;
			if constexpr (29 < width) if constexpr (constexpr auto pos = shift_mask(1, 29); pos >= 0) r |= ((p >> pos) & 1) << 29;
			if constexpr (30 < width) if constexpr (constexpr auto pos = shift_mask(1, 30); pos >= 0) r |= ((p >> pos) & 1) << 30;
			if constexpr (31 < width) if constexpr (constexpr auto pos = shift_mask(1, 31); pos >= 0) r |= ((p >> pos) & 1) << 31;
		}
		if constexpr (mask(2)) {
			const uint16_t p = (GPIOC->IDR & mask(2)) ^ inverted(2);
			if constexpr (0 < width) if constexpr (constexpr auto pos = shift_mask(2, 0); pos >= 0) r |= ((p >> pos) & 1) << 0;
			if constexpr (1 < width) if constexpr (constexpr auto pos = shift_mask(2, 1); pos >= 0) r |= ((p >> pos) & 1) << 1;
			if constexpr (2 < width) if constexpr (constexpr auto pos = shift_mask(2, 2); pos >= 0) r |= ((p >> pos) & 1) << 2;
			if constexpr (3 < width) if constexpr (constexpr auto pos = shift_mask(2, 3); pos >= 0) r |= ((p >> pos) & 1) << 3;
			if constexpr (4 < width) if constexpr (constexpr auto pos = shift_mask(2, 4); pos >= 0) r |= ((p >> pos) & 1) << 4;
			if constexpr (5 < width) if constexpr (constexpr auto pos = shift_mask(2, 5); pos >= 0) r |= ((p >> pos) & 1) << 5;
			if constexpr (6 < width) if constexpr (constexpr auto pos = shift_mask(2, 6); pos >= 0) r |= ((p >> pos) & 1) << 6;
			if constexpr (7 < width) if constexpr (constexpr auto pos = shift_mask(2, 7); pos >= 0) r |= ((p >> pos) & 1) << 7;
			if constexpr (8 < width) if constexpr (constexpr auto pos = shift_mask(2, 8); pos >= 0) r |= ((p >> pos) & 1) << 8;
			if constexpr (9 < width) if constexpr (constexpr auto pos = shift_mask(2, 9); pos >= 0) r |= ((p >> pos) & 1) << 9;
			if constexpr (10 < width) if constexpr (constexpr auto pos = shift_mask(2, 10); pos >= 0) r |= ((p >> pos) & 1) << 10;
			if constexpr (11 < width) if constexpr (constexpr auto pos = shift_mask(2, 11); pos >= 0) r |= ((p >> pos) & 1) << 11;
			if constexpr (12 < width) if constexpr (constexpr auto pos = shift_mask(2, 12); pos >= 0) r |= ((p >> pos) & 1) << 12;
			if constexpr (13 < width) if constexpr (constexpr auto pos = shift_mask(2, 13); pos >= 0) r |= ((p >> pos) & 1) << 13;
			if constexpr (14 < width) if constexpr (constexpr auto pos = shift_mask(2, 14); pos >= 0) r |= ((p >> pos) & 1) << 14;
			if constexpr (15 < width) if constexpr (constexpr auto pos = shift_mask(2, 15); pos >= 0) r |= ((p >> pos) & 1) << 15;
			if constexpr (16 < width) if constexpr (constexpr auto pos = shift_mask(2, 16); pos >= 0) r |= ((p >> pos) & 1) << 16;
			if constexpr (17 < width) if constexpr (constexpr auto pos = shift_mask(2, 17); pos >= 0) r |= ((p >> pos) & 1) << 17;
			if constexpr (18 < width) if constexpr (constexpr auto pos = shift_mask(2, 18); pos >= 0) r |= ((p >> pos) & 1) << 18;
			if constexpr (19 < width) if constexpr (constexpr auto pos = shift_mask(2, 19); pos >= 0) r |= ((p >> pos) & 1) << 19;
			if constexpr (20 < width) if constexpr (constexpr auto pos = shift_mask(2, 20); pos >= 0) r |= ((p >> pos) & 1) << 20;
			if constexpr (21 < width) if constexpr (constexpr auto pos = shift_mask(2, 21); pos >= 0) r |= ((p >> pos) & 1) << 21;
			if constexpr (22 < width) if constexpr (constexpr auto pos = shift_mask(2, 22); pos >= 0) r |= ((p >> pos) & 1) << 22;
			if constexpr (23 < width) if constexpr (constexpr auto pos = shift_mask(2, 23); pos >= 0) r |= ((p >> pos) & 1) << 23;
			if constexpr (24 < width) if constexpr (constexpr auto pos = shift_mask(2, 24); pos >= 0) r |= ((p >> pos) & 1) << 24;
			if constexpr (25 < width) if constexpr (constexpr auto pos = shift_mask(2, 25); pos >= 0) r |= ((p >> pos) & 1) << 25;
			if constexpr (26 < width) if constexpr (constexpr auto pos = shift_mask(2, 26); pos >= 0) r |= ((p >> pos) & 1) << 26;
			if constexpr (27 < width) if constexpr (constexpr auto pos = shift_mask(2, 27); pos >= 0) r |= ((p >> pos) & 1) << 27;
			if constexpr (28 < width) if constexpr (constexpr auto pos = shift_mask(2, 28); pos >= 0) r |= ((p >> pos) & 1) << 28;
			if constexpr (29 < width) if constexpr (constexpr auto pos = shift_mask(2, 29); pos >= 0) r |= ((p >> pos) & 1) << 29;
			if constexpr (30 < width) if constexpr (constexpr auto pos = shift_mask(2, 30); pos >= 0) r |= ((p >> pos) & 1) << 30;
			if constexpr (31 < width) if constexpr (constexpr auto pos = shift_mask(2, 31); pos >= 0) r |= ((p >> pos) & 1) << 31;
		}
		if constexpr (mask(3)) {
			const uint16_t p = (GPIOD->IDR & mask(3)) ^ inverted(3);
			if constexpr (0 < width) if constexpr (constexpr auto pos = shift_mask(3, 0); pos >= 0) r |= ((p >> pos) & 1) << 0;
			if constexpr (1 < width) if constexpr (constexpr auto pos = shift_mask(3, 1); pos >= 0) r |= ((p >> pos) & 1) << 1;
			if constexpr (2 < width) if constexpr (constexpr auto pos = shift_mask(3, 2); pos >= 0) r |= ((p >> pos) & 1) << 2;
			if constexpr (3 < width) if constexpr (constexpr auto pos = shift_mask(3, 3); pos >= 0) r |= ((p >> pos) & 1) << 3;
			if constexpr (4 < width) if constexpr (constexpr auto pos = shift_mask(3, 4); pos >= 0) r |= ((p >> pos) & 1) << 4;
			if constexpr (5 < width) if constexpr (constexpr auto pos = shift_mask(3, 5); pos >= 0) r |= ((p >> pos) & 1) << 5;
			if constexpr (6 < width) if constexpr (constexpr auto pos = shift_mask(3, 6); pos >= 0) r |= ((p >> pos) & 1) << 6;
			if constexpr (7 < width) if constexpr (constexpr auto pos = shift_mask(3, 7); pos >= 0) r |= ((p >> pos) & 1) << 7;
			if constexpr (8 < width) if constexpr (constexpr auto pos = shift_mask(3, 8); pos >= 0) r |= ((p >> pos) & 1) << 8;
			if constexpr (9 < width) if constexpr (constexpr auto pos = shift_mask(3, 9); pos >= 0) r |= ((p >> pos) & 1) << 9;
			if constexpr (10 < width) if constexpr (constexpr auto pos = shift_mask(3, 10); pos >= 0) r |= ((p >> pos) & 1) << 10;
			if constexpr (11 < width) if constexpr (constexpr auto pos = shift_mask(3, 11); pos >= 0) r |= ((p >> pos) & 1) << 11;
			if constexpr (12 < width) if constexpr (constexpr auto pos = shift_mask(3, 12); pos >= 0) r |= ((p >> pos) & 1) << 12;
			if constexpr (13 < width) if constexpr (constexpr auto pos = shift_mask(3, 13); pos >= 0) r |= ((p >> pos) & 1) << 13;
			if constexpr (14 < width) if constexpr (constexpr auto pos = shift_mask(3, 14); pos >= 0) r |= ((p >> pos) & 1) << 14;
			if constexpr (15 < width) if constexpr (constexpr auto pos = shift_mask(3, 15); pos >= 0) r |= ((p >> pos) & 1) << 15;
			if constexpr (16 < width) if constexpr (constexpr auto pos = shift_mask(3, 16); pos >= 0) r |= ((p >> pos) & 1) << 16;
			if constexpr (17 < width) if constexpr (constexpr auto pos = shift_mask(3, 17); pos >= 0) r |= ((p >> pos) & 1) << 17;
			if constexpr (18 < width) if constexpr (constexpr auto pos = shift_mask(3, 18); pos >= 0) r |= ((p >> pos) & 1) << 18;
			if constexpr (19 < width) if constexpr (constexpr auto pos = shift_mask(3, 19); pos >= 0) r |= ((p >> pos) & 1) << 19;
			if constexpr (20 < width) if constexpr (constexpr auto pos = shift_mask(3, 20); pos >= 0) r |= ((p >> pos) & 1) << 20;
			if constexpr (21 < width) if constexpr (constexpr auto pos = shift_mask(3, 21); pos >= 0) r |= ((p >> pos) & 1) << 21;
			if constexpr (22 < width) if constexpr (constexpr auto pos = shift_mask(3, 22); pos >= 0) r |= ((p >> pos) & 1) << 22;
			if constexpr (23 < width) if constexpr (constexpr auto pos = shift_mask(3, 23); pos >= 0) r |= ((p >> pos) & 1) << 23;
			if constexpr (24 < width) if constexpr (constexpr auto pos = shift_mask(3, 24); pos >= 0) r |= ((p >> pos) & 1) << 24;
			if constexpr (25 < width) if constexpr (constexpr auto pos = shift_mask(3, 25); pos >= 0) r |= ((p >> pos) & 1) << 25;
			if constexpr (26 < width) if constexpr (constexpr auto pos = shift_mask(3, 26); pos >= 0) r |= ((p >> pos) & 1) << 26;
			if constexpr (27 < width) if constexpr (constexpr auto pos = shift_mask(3, 27); pos >= 0) r |= ((p >> pos) & 1) << 27;
			if constexpr (28 < width) if constexpr (constexpr auto pos = shift_mask(3, 28); pos >= 0) r |= ((p >> pos) & 1) << 28;
			if constexpr (29 < width) if constexpr (constexpr auto pos = shift_mask(3, 29); pos >= 0) r |= ((p >> pos) & 1) << 29;
			if constexpr (30 < width) if constexpr (constexpr auto pos = shift_mask(3, 30); pos >= 0) r |= ((p >> pos) & 1) << 30;
			if constexpr (31 < width) if constexpr (constexpr auto pos = shift_mask(3, 31); pos >= 0) r |= ((p >> pos) & 1) << 31;
		}
		if constexpr (mask(4)) {
			const uint16_t p = (GPIOE->IDR & mask(4)) ^ inverted(4);
			if constexpr (0 < width) if constexpr (constexpr auto pos = shift_mask(4, 0); pos >= 0) r |= ((p >> pos) & 1) << 0;
			if constexpr (1 < width) if constexpr (constexpr auto pos = shift_mask(4, 1); pos >= 0) r |= ((p >> pos) & 1) << 1;
			if constexpr (2 < width) if constexpr (constexpr auto pos = shift_mask(4, 2); pos >= 0) r |= ((p >> pos) & 1) << 2;
			if constexpr (3 < width) if constexpr (constexpr auto pos = shift_mask(4, 3); pos >= 0) r |= ((p >> pos) & 1) << 3;
			if constexpr (4 < width) if constexpr (constexpr auto pos = shift_mask(4, 4); pos >= 0) r |= ((p >> pos) & 1) << 4;
			if constexpr (5 < width) if constexpr (constexpr auto pos = shift_mask(4, 5); pos >= 0) r |= ((p >> pos) & 1) << 5;
			if constexpr (6 < width) if constexpr (constexpr auto pos = shift_mask(4, 6); pos >= 0) r |= ((p >> pos) & 1) << 6;
			if constexpr (7 < width) if constexpr (constexpr auto pos = shift_mask(4, 7); pos >= 0) r |= ((p >> pos) & 1) << 7;
			if constexpr (8 < width) if constexpr (constexpr auto pos = shift_mask(4, 8); pos >= 0) r |= ((p >> pos) & 1) << 8;
			if constexpr (9 < width) if constexpr (constexpr auto pos = shift_mask(4, 9); pos >= 0) r |= ((p >> pos) & 1) << 9;
			if constexpr (10 < width) if constexpr (constexpr auto pos = shift_mask(4, 10); pos >= 0) r |= ((p >> pos) & 1) << 10;
			if constexpr (11 < width) if constexpr (constexpr auto pos = shift_mask(4, 11); pos >= 0) r |= ((p >> pos) & 1) << 11;
			if constexpr (12 < width) if constexpr (constexpr auto pos = shift_mask(4, 12); pos >= 0) r |= ((p >> pos) & 1) << 12;
			if constexpr (13 < width) if constexpr (constexpr auto pos = shift_mask(4, 13); pos >= 0) r |= ((p >> pos) & 1) << 13;
			if constexpr (14 < width) if constexpr (constexpr auto pos = shift_mask(4, 14); pos >= 0) r |= ((p >> pos) & 1) << 14;
			if constexpr (15 < width) if constexpr (constexpr auto pos = shift_mask(4, 15); pos >= 0) r |= ((p >> pos) & 1) << 15;
			if constexpr (16 < width) if constexpr (constexpr auto pos = shift_mask(4, 16); pos >= 0) r |= ((p >> pos) & 1) << 16;
			if constexpr (17 < width) if constexpr (constexpr auto pos = shift_mask(4, 17); pos >= 0) r |= ((p >> pos) & 1) << 17;
			if constexpr (18 < width) if constexpr (constexpr auto pos = shift_mask(4, 18); pos >= 0) r |= ((p >> pos) & 1) << 18;
			if constexpr (19 < width) if constexpr (constexpr auto pos = shift_mask(4, 19); pos >= 0) r |= ((p >> pos) & 1) << 19;
			if constexpr (20 < width) if constexpr (constexpr auto pos = shift_mask(4, 20); pos >= 0) r |= ((p >> pos) & 1) << 20;
			if constexpr (21 < width) if constexpr (constexpr auto pos = shift_mask(4, 21); pos >= 0) r |= ((p >> pos) & 1) << 21;
			if constexpr (22 < width) if constexpr (constexpr auto pos = shift_mask(4, 22); pos >= 0) r |= ((p >> pos) & 1) << 22;
			if constexpr (23 < width) if constexpr (constexpr auto pos = shift_mask(4, 23); pos >= 0) r |= ((p >> pos) & 1) << 23;
			if constexpr (24 < width) if constexpr (constexpr auto pos = shift_mask(4, 24); pos >= 0) r |= ((p >> pos) & 1) << 24;
			if constexpr (25 < width) if constexpr (constexpr auto pos = shift_mask(4, 25); pos >= 0) r |= ((p >> pos) & 1) << 25;
			if constexpr (26 < width) if constexpr (constexpr auto pos = shift_mask(4, 26); pos >= 0) r |= ((p >> pos) & 1) << 26;
			if constexpr (27 < width) if constexpr (constexpr auto pos = shift_mask(4, 27); pos >= 0) r |= ((p >> pos) & 1) << 27;
			if constexpr (28 < width) if constexpr (constexpr auto pos = shift_mask(4, 28); pos >= 0) r |= ((p >> pos) & 1) << 28;
			if constexpr (29 < width) if constexpr (constexpr auto pos = shift_mask(4, 29); pos >= 0) r |= ((p >> pos) & 1) << 29;
			if constexpr (30 < width) if constexpr (constexpr auto pos = shift_mask(4, 30); pos >= 0) r |= ((p >> pos) & 1) << 30;
			if constexpr (31 < width) if constexpr (constexpr auto pos = shift_mask(4, 31); pos >= 0) r |= ((p >> pos) & 1) << 31;
		}
		if constexpr (mask(5)) {
			const uint16_t p = (GPIOH->IDR & mask(5)) ^ inverted(5);
			if constexpr (0 < width) if constexpr (constexpr auto pos = shift_mask(5, 0); pos >= 0) r |= ((p >> pos) & 1) << 0;
			if constexpr (1 < width) if constexpr (constexpr auto pos = shift_mask(5, 1); pos >= 0) r |= ((p >> pos) & 1) << 1;
			if constexpr (2 < width) if constexpr (constexpr auto pos = shift_mask(5, 2); pos >= 0) r |= ((p >> pos) & 1) << 2;
			if constexpr (3 < width) if constexpr (constexpr auto pos = shift_mask(5, 3); pos >= 0) r |= ((p >> pos) & 1) << 3;
			if constexpr (4 < width) if constexpr (constexpr auto pos = shift_mask(5, 4); pos >= 0) r |= ((p >> pos) & 1) << 4;
			if constexpr (5 < width) if constexpr (constexpr auto pos = shift_mask(5, 5); pos >= 0) r |= ((p >> pos) & 1) << 5;
			if constexpr (6 < width) if constexpr (constexpr auto pos = shift_mask(5, 6); pos >= 0) r |= ((p >> pos) & 1) << 6;
			if constexpr (7 < width) if constexpr (constexpr auto pos = shift_mask(5, 7); pos >= 0) r |= ((p >> pos) & 1) << 7;
			if constexpr (8 < width) if constexpr (constexpr auto pos = shift_mask(5, 8); pos >= 0) r |= ((p >> pos) & 1) << 8;
			if constexpr (9 < width) if constexpr (constexpr auto pos = shift_mask(5, 9); pos >= 0) r |= ((p >> pos) & 1) << 9;
			if constexpr (10 < width) if constexpr (constexpr auto pos = shift_mask(5, 10); pos >= 0) r |= ((p >> pos) & 1) << 10;
			if constexpr (11 < width) if constexpr (constexpr auto pos = shift_mask(5, 11); pos >= 0) r |= ((p >> pos) & 1) << 11;
			if constexpr (12 < width) if constexpr (constexpr auto pos = shift_mask(5, 12); pos >= 0) r |= ((p >> pos) & 1) << 12;
			if constexpr (13 < width) if constexpr (constexpr auto pos = shift_mask(5, 13); pos >= 0) r |= ((p >> pos) & 1) << 13;
			if constexpr (14 < width) if constexpr (constexpr auto pos = shift_mask(5, 14); pos >= 0) r |= ((p >> pos) & 1) << 14;
			if constexpr (15 < width) if constexpr (constexpr auto pos = shift_mask(5, 15); pos >= 0) r |= ((p >> pos) & 1) << 15;
			if constexpr (16 < width) if constexpr (constexpr auto pos = shift_mask(5, 16); pos >= 0) r |= ((p >> pos) & 1) << 16;
			if constexpr (17 < width) if constexpr (constexpr auto pos = shift_mask(5, 17); pos >= 0) r |= ((p >> pos) & 1) << 17;
			if constexpr (18 < width) if constexpr (constexpr auto pos = shift_mask(5, 18); pos >= 0) r |= ((p >> pos) & 1) << 18;
			if constexpr (19 < width) if constexpr (constexpr auto pos = shift_mask(5, 19); pos >= 0) r |= ((p >> pos) & 1) << 19;
			if constexpr (20 < width) if constexpr (constexpr auto pos = shift_mask(5, 20); pos >= 0) r |= ((p >> pos) & 1) << 20;
			if constexpr (21 < width) if constexpr (constexpr auto pos = shift_mask(5, 21); pos >= 0) r |= ((p >> pos) & 1) << 21;
			if constexpr (22 < width) if constexpr (constexpr auto pos = shift_mask(5, 22); pos >= 0) r |= ((p >> pos) & 1) << 22;
			if constexpr (23 < width) if constexpr (constexpr auto pos = shift_mask(5, 23); pos >= 0) r |= ((p >> pos) & 1) << 23;
			if constexpr (24 < width) if constexpr (constexpr auto pos = shift_mask(5, 24); pos >= 0) r |= ((p >> pos) & 1) << 24;
			if constexpr (25 < width) if constexpr (constexpr auto pos = shift_mask(5, 25); pos >= 0) r |= ((p >> pos) & 1) << 25;
			if constexpr (26 < width) if constexpr (constexpr auto pos = shift_mask(5, 26); pos >= 0) r |= ((p >> pos) & 1) << 26;
			if constexpr (27 < width) if constexpr (constexpr auto pos = shift_mask(5, 27); pos >= 0) r |= ((p >> pos) & 1) << 27;
			if constexpr (28 < width) if constexpr (constexpr auto pos = shift_mask(5, 28); pos >= 0) r |= ((p >> pos) & 1) << 28;
			if constexpr (29 < width) if constexpr (constexpr auto pos = shift_mask(5, 29); pos >= 0) r |= ((p >> pos) & 1) << 29;
			if constexpr (30 < width) if constexpr (constexpr auto pos = shift_mask(5, 30); pos >= 0) r |= ((p >> pos) & 1) << 30;
			if constexpr (31 < width) if constexpr (constexpr auto pos = shift_mask(5, 31); pos >= 0) r |= ((p >> pos) & 1) << 31;
		}
		return r;
	}
};

} // namespace modm::platform