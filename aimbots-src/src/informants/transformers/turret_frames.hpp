#pragma once

#include "informants/transformers/coordinate_frame.hpp"
// #include "informants/transformers/cartesian_frames.hpp"

#include "utils/common_types.hpp"
#include "utils/math/matrix_helpers.hpp"

//#include "utils/math/transform_setup.hpp"
#include "utils/math/transformations.hpp"

namespace src::Informants::Transformers {

enum TurretFrameType {
    TURRET_GIMBAL_FRAME = 0,
    TURRET_CAMERA_FRAME = 1,
};
class TurretFrames {
public:
    TurretFrames();
    ~TurretFrames() = default;

    CoordinateFrame& getFrame(TurretFrameType frame) {
        switch (frame) {
            case TURRET_GIMBAL_FRAME:
                return gimbalFrame;
                break;
            case TURRET_CAMERA_FRAME:
                return cameraFrame;
                break;
        }
        return gimbalFrame;
    }

    void updateFrames(
        float yawFieldRelative,
        float pitchFieldRelative,
        AngleUnit angleUnit);

    void mirrorPastCameraFrame(float gimbalYawAngle, float gimbalPitchAngle, AngleUnit angleUnit);

private:
    CoordinateFrame gimbalFrame;
    CoordinateFrame cameraFrame;

    Vector3f chassis_origin_relative_to_world_origin;
    Matrix3f chassis_orientation_relative_to_world_orientation;

    Vector3f camera_origin_relative_to_chassis_origin;
    Vector3f camera_origin_relative_to_gimbal_origin;

    Matrix3f gimbal_orientation_relative_to_chassis_orientation;
    Matrix3f gimbal_orientation_relative_to_world_orientation;
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