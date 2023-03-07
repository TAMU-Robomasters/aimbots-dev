#pragma once

#include "informants/transformers/coordinate_frame.hpp"
#include "utils/common_types.hpp"
#include "utils/math/matrix_helpers.hpp"
#include "utils/math/transform_setup.hpp"

namespace src::Informants {
class RobotFrames {
public:
    RobotFrames();

    ~RobotFrames() = default;
    void updateFrames(float yawChassisRelative, float pitchChassisRelative, float chassisWorldRelativeAngle, Vector3f deltar);

private:
    // Do fancy math, know where eveyrthing is and what to define
    CoordinateFrame fieldFrame;
    CoordinateFrame chassisFrame;  //"Ground Frame"
    CoordinateFrame gimbalFrame;
    CoordinateFrame cameraFrame;
    CoordinateFrame ballisticsFrame;  // THIS IS NOT A BARREL FRAME THIS IS DIFFERENT (just chassis frame but moved up to barrel height)

    Vector3f chassis_origin_relative_to_world_origin;
    Matrix3f chassis_orientation_relative_to_world_orientation;
    Vector3f camera_origin_relative_to_chassis_origin;
    Matrix3f turret_orientation_relative_to_chassis_orientation;
};
}  // namespace src::Informants