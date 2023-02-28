#pragma once

#include "Informants/CoordinateFrame.hpp"
#include "utils/common_types.hpp"
#include "utils/math/matrix_helpers.hpp"
#include "utils/math/transform_setup.hpp"

namespace src::Informants {
class RobotFrames {
public:
    RobotFrames(
        float YAW_START_ANGLE,
        float PITCH_START_ANGLE,
        float CHASSIS_ANGLE_RELATIVE_TO_WORLD,
        Vector3f TURRET_ORIGIN_RELATIVE_TO_CHASSIS_ORIGIN,
        Vector3f CAMERA_ORIGIN_RELATIVE_TO_TURRET_ORIGIN,
        Vector3f CHASSIS_ORIGIN_RELATIVE_TO_TURRET_ORIGIN);
    ~RobotFrames() = default;
    void updateFrames(float yawAng, float pitchAng, Vector3f deltar);

private:
    // Do fancy math, know where eveyrthing is and what to define
    CoordinateFrame fieldFrame;
    CoordinateFrame chassisFrame;  //"Ground Frame"
    CoordinateFrame gimbalFrame;
    const Vector3f TURRET_ORIGIN_RELATIVE_TO_CHASSIS_ORIGIN;
    CoordinateFrame cameraFrame;
    const Vector3f CAMERA_ORIGIN_RELATIVE_TO_TURRET_ORIGIN;
    CoordinateFrame ballisticsFrame;  // THIS IS NOT A BARREL FRAME THIS IS DIFFERENT

    Vector3f CAMERA_ORIGIN_RELATIVE_TO_CHASSIS_ORIGIN;
    Matrix<float, 3, 3> TURRET_ORIENTATION_RELATIVE_TO_CHASSIS_ORIENTATION;
};
}  // namespace src::Informants