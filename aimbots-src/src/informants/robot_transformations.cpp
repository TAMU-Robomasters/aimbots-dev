#include "robot_transformations.hpp"

#include "utils/robot_specific_inc.hpp"

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

    this->camera_to_gimbal = Matrix4f(camera_to_gimbal_array);
    this->gimbal_to_chassis = Matrix4f(gimbal_to_chassis_array);
    this->chassis_to_field = Matrix4f(chassis_to_field_array);
}

RobotTransformer::~RobotTransformer() {}

}  // namespace src::Informants