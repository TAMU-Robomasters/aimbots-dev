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

    Matrix<float, 4, 4> invertTransformation(Matrix<float, 4, 4> transform);

private:
    Matrix<float, 4, 4> camera_to_gimbal;
    Matrix<float, 4, 4> gimbal_to_chassis;
    Matrix<float, 4, 4> chassis_to_field;

    Matrix<float, 4, 4> gimbal_to_camera;
    Matrix<float, 4, 4> chassis_to_gimbal;
    Matrix<float, 4, 4> field_to_chassis;
};

}  // namespace src::Informants