#include "informants/transformers/turret_frames.hpp"

#include "informants/transformers/coordinate_frame.hpp"
// #include "informants/transformers/cartesian_frames.hpp"

#include "utils/common_types.hpp"
#include "utils/math/matrix_helpers.hpp"

//#include "utils/math/transform_setup.hpp"
#include "utils/math/transformations.hpp"
#include "utils/robot_specific_inc.hpp"

using namespace src::Utils::MatrixHelper;
namespace src::Informants::Transformers {

TurretFrames::TurretFrames() {
#ifndef TARGET_TURRET
    gimbalFrame.setOrigin(Vector3f(0, 0, 0));
    gimbalFrame.setOrientation(Matrix3f::identityMatrix());

    // update frames to initial values
    updateFrames(0, 0, 0, AngleUnit::Radians);

    cameraFrame.setOrientation(gimbal_orientation_relative_to_world_orientation);
    cameraFrame.setOrigin(camera_origin_relative_to_gimbal_origin);
#endif
}

void TurretFrames::updateFrames(float fieldYaw, float fieldPitch, float fieldRoll, AngleUnit angleUnit) {
#ifndef TARGET_TURRET

    this->gimbal_orientation_relative_to_world_orientation =
        rotationMatrix(fieldYaw, Z_AXIS, angleUnit); /* rotationMatrix(fieldPitch, X_AXIS, angleUnit) *
        rotationMatrix(fieldRoll, Y_AXIS, angleUnit);*/  // gimbal to field rotation

    // Don't need to update gimbal frame origin, should be the center of this coordinate framework
    // gimbalFrame.setOrientation(gimbal_orientation_relative_to_world_orientation);
    gimbalFrame.setOrientation(Matrix3f::identityMatrix());

#else
    UNUSED(yawFieldRelative);
    UNUSED(pitchFieldRelative);
    UNUSED(angleUnit);
#endif
}

float camera_offset_angle = -0.0f;

void TurretFrames::mirrorPastCameraFrame(float gimbalYawAngle, float gimbalPitchAngle, AngleUnit angleUnit) {
#ifndef TARGET_TURRET
    Matrix3f at_cv_update_gimbal_orientation_relative_to_world_orientation =
        rotationMatrix(gimbalYawAngle, Z_AXIS, angleUnit) * rotationMatrix(gimbalPitchAngle, X_AXIS, angleUnit) *
        rotationMatrix(camera_offset_angle, Z_AXIS, AngleUnit::Degrees);

    Vector3f at_cv_update_camera_origin_relative_to_gimbal_origin =
        gimbal_orientation_relative_to_world_orientation * CAMERA_ORIGIN_RELATIVE_TO_TURRET_ORIGIN;

    cameraFrame.setOrientation(at_cv_update_gimbal_orientation_relative_to_world_orientation);
    cameraFrame.setOrigin(at_cv_update_camera_origin_relative_to_gimbal_origin);
#else
    UNUSED(gimbalYawAngle);
    UNUSED(gimbalPitchAngle);
    UNUSED(angleUnit);
#endif
}
}  // namespace src::Informants::Transformers