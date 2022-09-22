#include "utils/common_types.hpp"

namespace src::utils::MatrixHelper {

float xy_angle_between_locations(AngleUnit unit, Matrix<float, 1, 3> v1, Matrix<float, 1, 3> v2) {
    // Gets the absolute angle from the horizontal in the positive direction of the vector from one point to another
    // Define the vector from point 2 to point 1
    Matrix<float, 1, 3> v3 = v2 - v1;

    // Calculate absolute angle from positive horizontal
    float angle = atan2f(v3[0][1], v3[0][0]);

    if (unit == AngleUnit::Degrees) {
        angle = modm::toDegree(angle);
    }
    return angle;
}

// Outdated use rotation_matix(~, ~, 2) instead
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

Matrix<float, 3, 3> rotation_matrix(AngleUnit unit, float angle, int axis) {
    // Constructs a rotation matrix about an elementary (Robot's X, Y, Z Axis)
    // Takes input of angle unit (Degrees / Rads), Float angle, and axis
    //  0 = X - Axis
    //  1 = Y - Axis
    //  2 = Z - Axis
    // Returns rotation matrix
    // If axis is not recognized, returns Identity matrix = No Rotation
    if (unit == AngleUnit::Degrees) {
        angle = modm::toRadian(angle);
    }
    float c = cosf(angle);
    float s = sinf(angle);
    // clang-format off
            float I_rotation_array[9] = {
                1.0f, 0.0f, 0.0f, 
                0.0f, 1.0f, 0.0f, 
                0.0f, 0.0f, 1.0f};
    // clang-format on
    switch (axis) {
        case 0: {
            // Create X-axis rotation matrix
            // In ground robots this is the "roll" and does not exist
            // In Sentry this is the "pitch"
            // clang-format off
            float x_rotation_array[9] = {
                 1.0f, 0.0f, 0.0f,
                 0.0f,    c,   -s, 
                 0.0f,    s,   c};
            // clang-format on
            return Matrix<float, 3, 3>(x_rotation_array);
            break;
        }
        case 1: {
            // Create Y-Axis Rotation Matirx
            // In ground rotbots this is the "pitch"
            // In sentry this is the "roll" and does not exist
            // clang-format off
            float y_rotation_array[9] = {
                   c, 0.0f,    s, 
                0.0f, 1.0f, 0.0f, 
                  -s, 0.0f,    c};
            // clang-format on
            return Matrix<float, 3, 3>(y_rotation_array);
            break;
        }
        case 2: {
            // Creates a Z-Axis rotation matrix
            // In Ground robots and sentry this is "yaw"
            // clang-format off
            float z_rotation_array[9] = {
                   c,   -s, 0.0f, 
                   s,    c, 0.0f, 
                0.0f, 0.0f, 1.0f};
            // clang-format on
            return Matrix<float, 3, 3>(z_rotation_array);
            break;
        }

            return Matrix<float, 3, 3>(I_rotation_array);
    }
}

}  // namespace src::utils::MatrixHelper