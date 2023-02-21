#pragma once

#include <complex>
#include <vector>

#include "utils/common_types.hpp"


namespace src::Utils::MatrixHelper {

// returns radians
float xy_angle_between_locations(AngleUnit unit, Matrix<float, 1, 3> v1, Matrix<float, 1, 3> v2);

// takes radians, rotates clockwise
// Outdated use rotation_matix(~, ~, 2) instead
Matrix<float, 3, 3> xy_rotation_matrix(AngleUnit unit, float angle);

// Takes radians / degrees, an angle angle, rotates about an elementary axis
Matrix<float, 3, 3> rotation_matrix(AngleUnit unit, float angle, int axis);

}  // namespace src::Utils::MatrixHelper