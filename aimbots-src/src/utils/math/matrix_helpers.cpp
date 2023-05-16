#include "utils/common_types.hpp"

namespace src::Utils::MatrixHelper {

float xy_angle_between_locations(AngleUnit unit, Matrix<float, 1, 3> v1, Matrix<float, 1, 3> v2) {
    // Gets the absolute angle from the horizontal in the positive direction of the vector from one point to another
    // Used to determine the absolute, field relative angle to aim at from our robot positon (v1) to another postion (v2)
    // Define the vector from point 2 to point 1
    Matrix<float, 1, 3> v3 = v2 - v1;

    // Calculate absolute angle from positive horizontal
    float angle = atan2f(v3[0][1], v3[0][0]);

    if (unit == AngleUnit::Degrees) {
        angle = modm::toDegree(angle);
    }
    return angle;
}

Matrix3f rotationMatrix(float angle, LinearAxis axis, AngleUnit unit) {
    angle = (unit == AngleUnit::Degrees) ? modm::toRadian(angle) : angle;

    float c = cosf(angle);
    float s = sinf(angle);
    // clang-format off
    float rotation_array[9] = {
        1.0f, 0.0f, 0.0f, 
        0.0f, 1.0f, 0.0f, 
        0.0f, 0.0f, 1.0f};
    // clang-format on
    switch (axis) {
        case X_AXIS: {
            // Create X-axis rotation matrix
            // In ground robots this is the "roll" and does not exist
            // clang-format off
             rotation_array[0] = 1.0f; rotation_array[1] = 0.0f; rotation_array[2] = 0.0f;
             rotation_array[3] = 0.0f; rotation_array[4] = c; rotation_array[5] = -s;
             rotation_array[6] = 0.0f; rotation_array[7] = s; rotation_array[8] = c;
            // clang-format on
            break;
        }
        case Y_AXIS: {
            // Create Y-Axis Rotation Matrix
            // In ground robots this is the "pitch"
            // clang-format off
            rotation_array[0] = c;    rotation_array[1] = 0.0f; rotation_array[2] = s;
            rotation_array[3] = 0.0f; rotation_array[4] = 1.0;  rotation_array[5] = 0.0f;
            rotation_array[6] = -s;   rotation_array[7] = 0.0f; rotation_array[8] = c;
            // clang-format on
            break;
        }
        case Z_AXIS: {
            // Creates a Z-Axis rotation matrix
            // In Ground robots and sentry this is "yaw"
            // clang-format off
            rotation_array[0] = c;    rotation_array[1] = -s;   rotation_array[2] = 0.0f;
            rotation_array[3] = s;    rotation_array[4] = c;    rotation_array[5] = 0.0f;
            rotation_array[6] = 0.0f; rotation_array[7] = 0.0f; rotation_array[8] = 1.0f;
            // clang-format on
            break;
        }
    }
    return Matrix3f(rotation_array);
}

}  // namespace src::Utils::MatrixHelper