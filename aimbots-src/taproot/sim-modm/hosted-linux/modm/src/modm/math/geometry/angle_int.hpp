// coding: utf-8
// ----------------------------------------------------------------------------
/*
 * Copyright (c) 2024, Thomas Sommer
 *
 * This file is part of the modm project.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
// ----------------------------------------------------------------------------

#pragma once

#include <cmath>
#include <modm/math/utils/integer_traits.hpp>
#include <numbers>

namespace modm
{

/// @brief Represents an absolute angle in a full circle
/// @group modm_math_geometry
template<int Bits>
struct modm_packed IntegerAngle
{
	using T = modm::least_uint<Bits>;
	using DeltaType = std::make_signed_t<modm::least_uint<Bits + 1>>;

	T data;
	static constexpr T max = std::pow(2, Bits) - 1;

	constexpr DeltaType
	getDelta()
	{
		static DeltaType previous{static_cast<DeltaType>(data)};
		DeltaType delta = DeltaType(data) - previous;

		if (delta < -(max / 2))
		{
			delta += max;
		} else if (delta > (max / 2))
		{
			delta -= max;
		}

		previous = data;

		return delta;
	}

	constexpr float
	toDegree() const
	{
		return float(data) * 360 / max;
	}

	constexpr float
	toRadian() const
	{
		return float(data) * 2 * std::numbers::pi_v<float> / max;
	}
};
}  // namespace modm