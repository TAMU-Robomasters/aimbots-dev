#include "utils/common_types.hpp"

float v1XDisplay = 0.0f;
float v1YDisplay = 0.0f;
float v2XDisplay = 0.0f;
float v2YDisplay = 0.0f;
float v3XDisplay = 0.0f;
float v3YDisplay = 0.0f;

float xy_angle_between_locations(AngleUnit unit, Matrix<float, 1, 3> v1, Matrix<float, 1, 3> v2) {
    Matrix<float, 1, 3> v3 = v2 - v1;

    v1XDisplay = v1[0][0];
    v1YDisplay = v1[0][1];
    v2XDisplay = v2[0][0];
    v2YDisplay = v2[0][1];
    v3XDisplay = v3[0][0];
    v3YDisplay = v3[0][1];
    float angle = atan2f(v3[0][1], v3[0][0]);
    if (unit == AngleUnit::Degrees) {
        angle = modm::toDegree(angle);
    }
    return angle;
}

Matrix<float, 3, 3> xy_rotation_matrix(AngleUnit unit, float angle) {
    if (unit == AngleUnit::Degrees) {
        angle = modm::toRadian(angle);
    }
    float c = cosf(angle);
    float s = sinf(angle);
    // clang-format off
    float xy_rotation_array[9] = {
        c,    -s,   0.0f,
        s,     c,   0.0f,
        0.0f, 0.0f, 1.0f};
    // clang-format on
    return Matrix<float, 3, 3>(xy_rotation_array);
}