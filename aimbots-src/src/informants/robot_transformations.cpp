#include "robot_transformations.hpp"

#include "src/utils/robot_specific_inc.hpp"

namespace src::Informants {

RobotTransformer::RobotTransformer() {
    // clang-format off
    static constexpr float camera_to_gimbal_array[16] = {
        1, 0, 0, CAMERA_POSITION_RELATIVE_TO_TURRET_ORIGIN[0],
        0, 1, 0, CAMERA_POSITION_RELATIVE_TO_TURRET_ORIGIN[1],
        0, 0, 1, CAMERA_POSITION_RELATIVE_TO_TURRET_ORIGIN[2],
        0, 0, 0, 1
    };

    static constexpr float gimbal_to_chassis_array[16] = {
        1, 0, 0, TURRET_ORIGIN_RELATIVE_TO_CHASSIS_ORIGIN[0],
        0, 1, 0, TURRET_ORIGIN_RELATIVE_TO_CHASSIS_ORIGIN[1],
        0, 0, 1, TURRET_ORIGIN_RELATIVE_TO_CHASSIS_ORIGIN[2],
        0, 0, 0, 1
    };

    static constexpr float chas_rotation_origin_angle = CHASSIS_ORIGIN_RELATIVE_TO_WORLD_ORIGIN[3];

    static const float chassis_to_field_array[16] = {
        cos(chas_rotation_origin_angle), -sin(chas_rotation_origin_angle), 0, CHASSIS_ORIGIN_RELATIVE_TO_WORLD_ORIGIN[0],
        sin(chas_rotation_origin_angle),  cos(chas_rotation_origin_angle), 0, CHASSIS_ORIGIN_RELATIVE_TO_WORLD_ORIGIN[1],
        0,                                0,                               1, CHASSIS_ORIGIN_RELATIVE_TO_WORLD_ORIGIN[2],
        0,                                0,                               0, 1
    };
    // clang-format on

    this->camera_to_gimbal = Matrix<float, 4, 4>(camera_to_gimbal_array);
    this->gimbal_to_chassis = Matrix<float, 4, 4>(gimbal_to_chassis_array);
    this->chassis_to_field = Matrix<float, 4, 4>(chassis_to_field_array);
}

RobotTransformer::~RobotTransformer() {}

Matrix<float, 4, 4> RobotTransformer::invertTransformation(Matrix<float, 4, 4> transform) {
    Matrix<float, 4, 4> inverted_transform = Matrix<float, 4, 4>::zeroMatrix();

    // Invert the rotation matrix
    for (uint_fast8_t i = 0; i < 3; i++) {
        for (uint_fast8_t j = 0; j < 3; j++) {
            inverted_transform.element[j * 4 + i] = transform.element[i * 4 + j];
        }
    }

    // Multiply the transposed rotation matrix by the negative translation vector

    Matrix<float, 3, 3> rT = transform.subMatrix<3, 3>(3, 3).asTransposed();

    Matrix<float, 3, 1> t = transform.subMatrix<3, 1>(3, 0);
    Matrix<float, 3, 1> neg_tT = rT * t;
    neg_tT *= -1;
}

}  // namespace src::Informants