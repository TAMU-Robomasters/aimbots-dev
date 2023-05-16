#pragma once

#include <complex>
#include <vector>

#include "utils/common_types.hpp"

namespace src::Utils::MatrixHelper {

// returns radians
float xy_angle_between_locations(AngleUnit unit, Matrix<float, 1, 3> v1, Matrix<float, 1, 3> v2);

/**
 *   Constructs a rotation matrix about an elementary (Robot's X, Y, Z Axis)
 *   Rotation matricies are used to determine coordinates before / after a rotation
 *   If axis is not recognized, returns Identity matrix = No Rotation
 *   @param angle Angle to rotate by
 *   @param axis LinearAxis::X_AXIS, LinearAxis::Y_AXIS, LinearAxis::Z_AXIS
 *   @param unit AngleUnit::Degrees or AngleUnit::Radians
 *
 *   @return Matrix3f Rotation Matrix
 */
Matrix3f rotationMatrix(float angle, LinearAxis axis, AngleUnit unit);

}  // namespace src::Utils::MatrixHelper