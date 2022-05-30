#pragma once

#include "utils/common_types.hpp"

// returns radians
float xy_angle_between_locations(AngleUnit unit, Matrix<float, 1, 3> v1, Matrix<float, 1, 3> v2);

// takes radians, rotates clockwise
Matrix<float, 3, 3> xy_rotation_matrix(AngleUnit unit, float angle);