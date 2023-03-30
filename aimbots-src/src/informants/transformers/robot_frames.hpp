#pragma once

#include "informants/transformers/coordinate_frame.hpp"
#include "utils/common_types.hpp"
#include "utils/math/matrix_helpers.hpp"
#include "utils/math/transform_setup.hpp"

namespace src::Informants::Transformers {

enum FrameType { FIELD_FRAME = 0, CHASSIS_FRAME = 1, GIMBAL_FRAME = 2, CAMERA_FRAME = 3, BALLISTICS_FRAME = 4 };
class RobotFrames {
public:
    RobotFrames();
    ~RobotFrames() = default;

    CoordinateFrame& getFrame(FrameType frame) {
        switch (frame) {
            case FIELD_FRAME:
                return fieldFrame;
            case CHASSIS_FRAME:
                return chassisFrame;
            case GIMBAL_FRAME:
                return gimbalFrame;
            case CAMERA_FRAME:
                return cameraFrame;
            case BALLISTICS_FRAME:
                return ballisticsFrame;
            default:
                return chassisFrame;
        }
    }

    void updateFrames(
        float yawChassisRelative,
        float pitchChassisRelative,
        float chassisWorldRelativeAngle,
        Vector3f robotPositionRelativeToStartPosition,
        AngleUnit angleUnit);

private:
    CoordinateFrame fieldFrame;
    CoordinateFrame chassisFrame;  // "Ground Frame
    CoordinateFrame gimbalFrame;
    CoordinateFrame cameraFrame;
    CoordinateFrame ballisticsFrame;
    // ^ THIS IS NOT THE BARREL FRAME THIS IS DIFFERENT (just chassis frame but moved up to barrel height)

    Vector3f chassis_origin_relative_to_world_origin;
    Matrix3f chassis_orientation_relative_to_world_orientation;

    Vector3f camera_origin_relative_to_chassis_origin;
    Matrix3f turret_orientation_relative_to_chassis_orientation;
};
}  // namespace src::Informants::Transformers

// Set Orientation matrix w.r.t field I.E where is robot facing (0 = robot is facing same direction as field coordinates)
// ____________
// |R  |Y     |
// |__  ^     |
// |    |     |
// |    +->X  |
// |        __|
// |          |
// |______|__B|
//
//

// Calculate the orientation relative to chassis
//      FRONT                          ^ Gz
// _______________                  ___|__
// |      ^ y    |           ______/   |  \.
// |      |      |    Gy <- |O|___|----+   |
// |      +-> x  |                 \______/'
// |     /       |           ________|  |_______
// |    z        |          /TAMU #1 > UW >> CU \.
// |_____________|         /_____________________\.
//       BACK                \__/           \__/
//
// Note that for the chassis orientation, the theta is measured turning away from the x as positive
// Note to double check pitch angle correctly by turning up should be positive (towards z)
//      FRONT                _> +      ^ Gz
// _______________          /       ___|__
// |    _ ^ y    |         | ______/   |  \.
// |  /   |      |    Gy <- |O|___|----+   |
// | \/   +-> x  |                 \______/'
// |  +          |           ________|  |_______
// |             |          /   SOUNDS LIKE A   \.
// |_____________|         /_____SKILL_ISSUE_____\.
//       BACK                \__/           \__/
//
//
// Chassis frame is automatically initialized to the ground frame