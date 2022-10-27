#include "utils/common_types.hpp"

namespace src::utils::MatrixHelper {

float xy_angle_between_locations(AngleUnit unit, Matrix<float, 1, 3> v1, Matrix<float, 1, 3> v2) {
    Matrix<float, 1, 3> v3 = v2 - v1;

    float angle = atan2f(v3[0][1], v3[0][0]);
    if (unit == AngleUnit::Degrees) {
        angle = modm::toDegree(angle);
    }
    return angle;
}

// Rz(th) / yaw
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

// Ry(th) / pitch
Matrix<float, 3, 3> xz_rotation_matrix(AngleUnit unit, float angle) {
    if (unit == AngleUnit::Degrees) {
        angle = modm::toRadian(angle);
    }
    float c = cosf(angle);
    float s = sinf(angle);
    // clang-format off
    float xz_rotation_array[9] = {
        c,    0.0f,    s,
        0.0f, 1.0f, 0.0f,
        -s,   0.0f,   c};
    // clang-format on
    return Matrix<float, 3, 3>(xz_rotation_array);
}

// Rx(th) / roll
Matrix<float, 3, 3> yz_rotation_matrix(AngleUnit unit, float angle) {
    if (unit == AngleUnit::Degrees) {
        angle = modm::toRadian(angle);
    }
    float c = cosf(angle);
    float s = sinf(angle);
    // clang-format off
    float yz_rotation_array[9] = {
        1.0f, 0.0f, 0.0f,
        0.0f,    c,   -s,
        0.0f,    s,    c};
    // clang-format on
    return Matrix<float, 3, 3>(yz_rotation_array);
}

}  // namespace src::utils::MatrixHelper