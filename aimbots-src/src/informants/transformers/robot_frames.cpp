#include "informants/transformers/robot_frames.hpp"

#include "informants/transformers/coordinate_frame.hpp"
#include "utils/common_types.hpp"
#include "utils/math/matrix_helpers.hpp"

//#include "utils/math/transform_setup.hpp"
#include "utils/math/transformations.hpp"
#include "utils/robot_specific_inc.hpp"

using namespace src::Utils::MatrixHelper;
namespace src::Informants::Transformers {
// Look sid Idk how to do this exactly

RobotFrames::RobotFrames() {
#ifndef TARGET_TURRET
    // Init Ballistics Frame for Ballistics Math (Offset Vertically, Same Directions)
    ballisticsFrame.setOrigin(TURRET_ORIGIN_RELATIVE_TO_CHASSIS_ORIGIN);
    gimbalFrame.setOrigin(TURRET_ORIGIN_RELATIVE_TO_CHASSIS_ORIGIN);
    Matrix3f chassisIMUOrientation = rotationMatrix(CIMU_CALIBRATION_EULER_X, X_AXIS, AngleUnit::Radians) *
                                     rotationMatrix(CIMU_CALIBRATION_EULER_Y, Y_AXIS, AngleUnit::Radians) *
                                     rotationMatrix(CIMU_CALIBRATION_EULER_Z, Z_AXIS, AngleUnit::Radians);
    chassisIMUFrame.setOrientation(chassisIMUOrientation);

    // update frames to initial values
    updateFrames(YAW_AXIS_START_ANGLE, PITCH_AXIS_START_ANGLE, CHASSIS_START_ANGLE_WORLD, {0, 0, 0}, AngleUnit::Radians);

    cameraAtCVUpdateFrame.setOrientation(gimbal_orientation_relative_to_chassis_orientation);
    cameraAtCVUpdateFrame.setOrigin(camera_origin_relative_to_chassis_origin);
#endif
#ifdef TURRET_IMU
    // gimbal imu frame relative to gimbal (i.e. how is the imu mounted. In an ideal world this is all 0, 0, 0 but
    // hardware kids arent that good.
    Matrix3f gimbalIMUOrientation = rotationMatrix(TIMU_CALIBRATION_EULER_X, X_AXIS, AngleUnit::Radians) *
                                    rotationMatrix(TIMU_CALIBRATION_EULER_Y, Y_AXIS, AngleUnit::Radians) *
                                    rotationMatrix(TIMU_CALIBRATION_EULER_Z, Z_AXIS, AngleUnit::Radians);
    gimbalIMUFrame.setOrientation(gimbal_orientation_relative_to_chassis_orientation * gimbalIMUOrientation);
#endif
}

void RobotFrames::updateFrames(
    float yawChassisRelative,
    float pitchChassisRelative,
    float chassisWorldRelativeAngle,
    Vector3f robotPositionRelativeToStartPosition,
    AngleUnit angleUnit) {
#ifndef TARGET_TURRET
    chassis_orientation_relative_to_world_orientation = rotationMatrix(chassisWorldRelativeAngle, Z_AXIS, angleUnit);

    this->gimbal_orientation_relative_to_chassis_orientation =
        rotationMatrix(yawChassisRelative, Z_AXIS, angleUnit) *
        rotationMatrix(pitchChassisRelative, X_AXIS, angleUnit);  // gimbal to chassis rotation

    // this->camera_origin_relative_to_chassis_origin =
    //     TURRET_ORIGIN_RELATIVE_TO_CHASSIS_ORIGIN + this->gimbal_orientation_relative_to_chassis_orientation *
    //                                                    CAMERA_ORIGIN_RELATIVE_TO_TURRET_ORIGIN;  // also gimbal to chassis

    gimbalFrame.setOrientation(gimbal_orientation_relative_to_chassis_orientation);
    gimbalIMUFrame.setOrientation(gimbal_orientation_relative_to_chassis_orientation * gimbalIMUOrientation);

    // disabling this for performance, we're using the cameraAtCVUpdateFrame for all ballistics math
    // cameraFrame.setOrientation(gimbal_orientation_relative_to_chassis_orientation);
    // cameraFrame.setOrigin(camera_origin_relative_to_chassis_origin);

    fieldFrame.setOrientation(chassis_orientation_relative_to_world_orientation);
    fieldFrame.setOrigin(robotPositionRelativeToStartPosition);
#else
    UNUSED(yawChassisRelative);
    UNUSED(pitchChassisRelative);
    UNUSED(chassisWorldRelativeAngle);
    UNUSED(robotPositionRelativeToStartPosition);
    UNUSED(angleUnit);
#endif
}

float camera_mounting_offset_angle = -0.0f;

void RobotFrames::mirrorPastCameraFrame(float gimbalYawAngle, float gimbalPitchAngle, AngleUnit angleUnit) {
#ifndef TARGET_TURRET
    Matrix3f at_cv_update_gimbal_orientation_relative_to_chassis_orientation =
        rotationMatrix(gimbalYawAngle, Z_AXIS, angleUnit) * rotationMatrix(gimbalPitchAngle, X_AXIS, angleUnit) *
        rotationMatrix(camera_mounting_offset_angle, Z_AXIS, AngleUnit::Degrees);

    Vector3f at_cv_update_camera_origin_relative_to_chassis_origin =
        TURRET_ORIGIN_RELATIVE_TO_CHASSIS_ORIGIN +
        at_cv_update_gimbal_orientation_relative_to_chassis_orientation * CAMERA_ORIGIN_RELATIVE_TO_TURRET_ORIGIN;

    cameraAtCVUpdateFrame.setOrientation(at_cv_update_gimbal_orientation_relative_to_chassis_orientation);
    cameraAtCVUpdateFrame.setOrigin(at_cv_update_camera_origin_relative_to_chassis_origin);
#else
    UNUSED(gimbalYawAngle);
    UNUSED(gimbalPitchAngle);
    UNUSED(angleUnit);
#endif
}
}  // namespace src::Informants::Transformers