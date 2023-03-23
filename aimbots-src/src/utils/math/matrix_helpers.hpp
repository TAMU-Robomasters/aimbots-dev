#pragma once

#include <complex>
#include <vector>

#include "utils/common_types.hpp"

namespace src::Utils::MatrixHelper {

// returns radians
float xy_angle_between_locations(AngleUnit unit, Matrix<float, 1, 3> v1, Matrix<float, 1, 3> v2);

// Takes radians / degrees, an angle angle, rotates about an elementary axis
Matrix3f rotationMatrix(AngleUnit unit, float angle, int axis);

}  // namespace src::Utils::MatrixHelper