#pragma once

#include "informants/kinematics/coordinate_frame.hpp"
#include "utils/tools/common_types.hpp"
#include "utils/math/matrix_helpers.hpp"
#include "utils/tools/robot_specific_defines.hpp"

//#include "utils/math/transform_setup.hpp"
#include "utils/math/transformations.hpp"

namespace src::Informants::Transformers {

enum TurretFrameType {
    TURRET_FIELD_FRAME = 0,
    TURRET_CAMERA_FRAME = 1,
};
class TurretFrames {
public:
    TurretFrames();
    ~TurretFrames() = default;

    CoordinateFrame& getFrame(TurretFrameType frame) {
        switch (frame) {
            case TURRET_FIELD_FRAME:
                return turretFieldFrame;
                break;
            case TURRET_CAMERA_FRAME:
                return turretCameraFrame;
                break;
        }
        return turretFieldFrame;
    }

    void updateFrames(float fieldYaw, float fieldPitch, float fieldRoll, AngleUnit angleUnit);

    void mirrorPastCameraFrame(float gimbalYawAngle, float gimbalPitchAngle, AngleUnit angleUnit);

private:
    CoordinateFrame turretFieldFrame;
    CoordinateFrame turretCameraFrame;

    Vector3f chassis_origin_relative_to_world_origin;
    Matrix3f chassis_orientation_relative_to_world_orientation;

    Vector3f camera_origin_relative_to_chassis_origin;
    Vector3f camera_origin_relative_to_gimbal_origin;

    Matrix3f field_orientation_relative_to_chassis_orientation;
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
// | x <- +      |                 \______/'
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