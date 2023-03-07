#pragma once
#include "utils/common_types.hpp"

namespace src::Informants {

enum class RobotTransformations {
    CAMERA_TO_GIMBAL,
    GIMBAL_TO_CHASSIS,
    CHASSIS_TO_FIELD,
    GIMBAL_TO_CAMERA,
    CHASSIS_TO_GIMBAL,
    FIELD_TO_CHASSIS
};

class RobotTransformer {
public:
    RobotTransformer();
    ~RobotTransformer();

    Matrix4f invertTransformation(Matrix4f transform);

private:
    Matrix4f camera_to_gimbal;
    Matrix4f gimbal_to_chassis;
    Matrix4f chassis_to_field;

    Matrix4f gimbal_to_camera;
    Matrix4f chassis_to_gimbal;
    Matrix4f field_to_chassis;
};

}  // namespace src::Informants