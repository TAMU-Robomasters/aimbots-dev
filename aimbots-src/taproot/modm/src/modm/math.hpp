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

#ifndef MODM_MATH_HPP
#define	MODM_MATH_HPP

#include "math/algorithm.hpp"
#include "math/algorithm/enumerate.hpp"
#include "math/algorithm/prescaler.hpp"
#include "math/algorithm/range.hpp"
#include "math/filter.hpp"
#include "math/filter/debounce.hpp"
#include "math/filter/fir.hpp"
#include "math/filter/median.hpp"
#include "math/filter/moving_average.hpp"
#include "math/filter/pid.hpp"
#include "math/filter/ramp.hpp"
#include "math/filter/s_curve_controller.hpp"
#include "math/filter/s_curve_generator.hpp"
#include "math/geometry.hpp"
#include "math/geometry/angle.hpp"
#include "math/geometry/circle_2d.hpp"
#include "math/geometry/geometric_traits.hpp"
#include "math/geometry/line_2d.hpp"
#include "math/geometry/line_segment_2d.hpp"
#include "math/geometry/location_2d.hpp"
#include "math/geometry/point_set_2d.hpp"
#include "math/geometry/polygon_2d.hpp"
#include "math/geometry/quaternion.hpp"
#include "math/geometry/ray_2d.hpp"
#include "math/geometry/vector.hpp"
#include "math/geometry/vector1.hpp"
#include "math/geometry/vector2.hpp"
#include "math/geometry/vector3.hpp"
#include "math/geometry/vector4.hpp"
#include "math/interpolation.hpp"
#include "math/interpolation/lagrange.hpp"
#include "math/interpolation/linear.hpp"
#include "math/lu_decomposition.hpp"
#include "math/matrix.hpp"
#include "math/saturated/saturated.hpp"
#include "math/tolerance.hpp"
#include "math/units.hpp"
#include "math/utils.hpp"
#include "math/utils/arithmetic_traits.hpp"
#include "math/utils/bit_constants.hpp"
#include "math/utils/bit_operation.hpp"
#include "math/utils/crc.hpp"
#include "math/utils/endianness.hpp"
#include "math/utils/misc.hpp"
#include "math/utils/operator.hpp"
#endif	// MODM_MATH_HPP