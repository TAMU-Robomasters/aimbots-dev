#pragma once

#include <complex>
#include <vector>

#include "tap/algorithms/cmsis_mat.hpp"

#include "modm/math/geometry/location_2d.hpp"
#include "utils/common_types.hpp"

using Matrix3f_CMSIS = tap::algorithms::CMSISMat<3, 3>;
namespace src::Utils::MatrixHelper {

inline float getZAngleBetweenLocations(modm::Location2D<float> loc1, modm::Location2D<float> loc2, AngleUnit unit) {
    float x = loc2.getX() - loc1.getX();
    float y = loc2.getY() - loc1.getY();

    return unit == AngleUnit::Radians ? -atan2f(x, y) : modm::toDegree(atan2f(x, y));
}

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
inline Matrix3f rotationMatrix(float angle, LinearAxis axis, AngleUnit unit) {
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

// war crimes
inline Matrix3f asInverted(Matrix3f m) {
    // Inverts a 3x3 matrix
    // clang-format off
        float m_array[9] = {
            m[0][0], m[0][1], m[0][2],
            m[1][0], m[1][1], m[1][2],
            m[2][0], m[2][1], m[2][2]};
    // clang-format on
    Matrix3f_CMSIS m_cmsis(m_array);
    return Matrix3f(m_cmsis.inverse().data.data());
}

}  // namespace src::Utils::MatrixHelper