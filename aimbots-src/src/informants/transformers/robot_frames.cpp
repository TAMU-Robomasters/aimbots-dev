#include "informants/transformers/robot_frames.hpp"

#include "informants/transformers/coordinate_frame.hpp"
#include "utils/common_types.hpp"
#include "utils/math/matrix_helpers.hpp"
#include "utils/math/transform_setup.hpp"

#include "utils/robot_specific_inc.hpp"

using namespace src::Utils::MatrixHelper;

namespace src::Informants {
// Look sid Idk how to do this exactly

RobotFrames::RobotFrames() {
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

    // Init Ballistics Frame for Ballistics Math (Offset Vertically, Same Directions)
    ballisticsFrame.setOrigin(-1 * TURRET_ORIGIN_RELATIVE_TO_CHASSIS_ORIGIN);
    gimbalFrame.setOrigin(-1 * TURRET_ORIGIN_RELATIVE_TO_CHASSIS_ORIGIN);

    // update frames to initial values
    updateFrames(YAW_START_ANGLE,YAW_START_ANGLE,CHASSIS_START_ANGLE_WORLD, {0,0,0});
}

void RobotFrames::updateFrames(float yawChassisRelative, float pitchChassisRelative, float chassisWorldRelativeAngle, Vector3f position) {
    chassis_orientation_relative_to_world_orientation = rotationMatrix(AngleUnit::Degrees,chassisWorldRelativeAngle, Z_AXIS);
    turret_orientation_relative_to_chassis_orientation = rotationMatrix(AngleUnit::Degrees, yawChassisRelative, Z_AXIS) * rotationMatrix(AngleUnit::Degrees, pitchChassisRelative, X_AXIS);
    camera_origin_relative_to_chassis_origin = TURRET_ORIGIN_RELATIVE_TO_CHASSIS_ORIGIN + turret_orientation_relative_to_chassis_orientation.asTransposed() * CAMERA_ORIGIN_RELATIVE_TO_TURRET_ORIGIN;
    
    gimbalFrame.setOrientation(turret_orientation_relative_to_chassis_orientation);
    cameraFrame.setOrientation(turret_orientation_relative_to_chassis_orientation);
    cameraFrame.setOrigin(-1 * camera_origin_relative_to_chassis_origin);
    fieldFrame.setOrientation(chassis_orientation_relative_to_world_orientation);
    fieldFrame.setOrigin(chassis_orientation_relative_to_world_orientation*position+CHASSIS_START_POSITION_RELATIVE_TO_WORLD);
    // TODO: Check if correct
}
}  // namespace src::Informants