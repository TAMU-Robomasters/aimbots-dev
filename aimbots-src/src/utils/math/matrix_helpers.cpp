#include "utils/common_types.hpp"

float xy_angle_between_locations(AngleUnit unit, Matrix<float, 1, 3> v1, Matrix<float, 1, 3> v2) {
    Matrix<float, 1, 3> v3 = v2 - v1;
    float angle = atan2f(v3[0][1], v3[0][0]);
    if (unit == AngleUnit::Degrees) {
        angle = modm::toDegree(angle);
    }
    return angle;
}

Matrix<float, 3, 3> xy_rotation_matrix(AngleUnit unit, float angle) {
    angle = (unit == AngleUnit::Degrees) ? modm::toRadian(angle) : angle;
    float c = cosf(angle);
    float s = sinf(angle);
    static float xy_rotation_array[9] = {
        c, -s, 0.0f,
        s, c, 0.0f,
        0.0f, 0.0f, 1.0f};
    return Matrix<float, 3, 3>(xy_rotation_array);
}