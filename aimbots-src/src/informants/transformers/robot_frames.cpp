#include "informants/transformers/robot_frames.hpp"

#include "informants/transformers/coordinate_frame.hpp"
#include "utils/common_types.hpp"
#include "utils/math/matrix_helpers.hpp"
#include "utils/math/transform_setup.hpp"
#include "utils/robot_specific_inc.hpp"

using namespace src::Utils::MatrixHelper;

namespace src::Informants::Transformers {
// Look sid Idk how to do this exactly

RobotFrames::RobotFrames() {
    // Init Ballistics Frame for Ballistics Math (Offset Vertically, Same Directions)
    ballisticsFrame.setOrigin(-1 * TURRET_ORIGIN_RELATIVE_TO_CHASSIS_ORIGIN);
    gimbalFrame.setOrigin(-1 * TURRET_ORIGIN_RELATIVE_TO_CHASSIS_ORIGIN);

    // update frames to initial values
    updateFrames(YAW_START_ANGLE, PITCH_START_ANGLE, CHASSIS_START_ANGLE_WORLD, {0, 0, 0}, AngleUnit::Radians);
}

void RobotFrames::updateFrames(
    float yawChassisRelative,
    float pitchChassisRelative,
    float chassisWorldRelativeAngle,
    Vector3f robotPositionRelativeToStartPosition,
    AngleUnit angleUnit) {

    chassis_orientation_relative_to_world_orientation =
        rotationMatrix(angleUnit, chassisWorldRelativeAngle, Z_AXIS);
        
    turret_orientation_relative_to_chassis_orientation = rotationMatrix(angleUnit, yawChassisRelative, Z_AXIS) *
                                                         rotationMatrix(angleUnit, pitchChassisRelative, X_AXIS);
    camera_origin_relative_to_chassis_origin =
        TURRET_ORIGIN_RELATIVE_TO_CHASSIS_ORIGIN +
        turret_orientation_relative_to_chassis_orientation.asTransposed() * CAMERA_ORIGIN_RELATIVE_TO_TURRET_ORIGIN;

    gimbalFrame.setOrientation(turret_orientation_relative_to_chassis_orientation);
    cameraFrame.setOrientation(turret_orientation_relative_to_chassis_orientation);
    cameraFrame.setOrigin(-1 * camera_origin_relative_to_chassis_origin);
    fieldFrame.setOrientation(chassis_orientation_relative_to_world_orientation);
    fieldFrame.setOrigin(
        chassis_orientation_relative_to_world_orientation * robotPositionRelativeToStartPosition +
        CHASSIS_START_POSITION_RELATIVE_TO_WORLD);
    // TODO: Check if correct
}
}  // namespace src::Informants::Transformers