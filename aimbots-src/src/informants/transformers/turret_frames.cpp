#include "informants/transformers/turret_frames.hpp"

#include "informants/transformers/coordinate_frame.hpp"
#include "utils/common_types.hpp"
#include "utils/math/matrix_helpers.hpp"

//#include "utils/math/transform_setup.hpp"
#include "utils/math/transformations.hpp"
#include "utils/robot_specific_inc.hpp"

using namespace src::Utils::MatrixHelper;
namespace src::Informants::Transformers {

TurretFrames::TurretFrames() {
#ifndef TARGET_TURRET
    fieldFrame.setOrigin(Vector3f(0, 0, 0));
    fieldFrame.setOrientation(Matrix3f::identityMatrix());

    // update frames to initial values
    updateFrames(0, 0, 0, AngleUnit::Radians);

    cameraFrame.setOrientation(field_orientation_relative_to_chassis_orientation);
    cameraFrame.setOrigin(camera_origin_relative_to_gimbal_origin);
#endif
}

void TurretFrames::updateFrames(float fieldYaw, float fieldPitch, float fieldRoll, AngleUnit angleUnit) {
    UNUSED(fieldPitch);
    UNUSED(fieldRoll);

#ifndef TARGET_TURRET
    this->field_orientation_relative_to_chassis_orientation =
        rotationMatrix(fieldYaw, Z_AXIS, angleUnit); /* rotationMatrix(fieldPitch, X_AXIS, angleUnit) *
        rotationMatrix(fieldRoll, Y_AXIS, angleUnit);*/  // field to chassis rotation

    // Don't need to update field frame origin, should be the center of this coordinate framework
    // fieldFrame.setOrientation(Matrix3f::identityMatrix());

#else
    UNUSED(fieldYaw);
    UNUSED(angleUnit);
#endif
}

float camera_offset_angle = -0.0f;

void TurretFrames::mirrorPastCameraFrame(float gimbalYawAngle, float gimbalPitchAngle, AngleUnit angleUnit) {
#ifndef TARGET_TURRET
    Matrix3f at_cv_update_field_orientation_relative_to_chassis_orientation =
        rotationMatrix(gimbalYawAngle, Z_AXIS, angleUnit) * rotationMatrix(gimbalPitchAngle, X_AXIS, angleUnit) *
        rotationMatrix(camera_offset_angle, Z_AXIS, AngleUnit::Degrees);

    Vector3f at_cv_update_camera_origin_relative_to_gimbal_origin =
        field_orientation_relative_to_chassis_orientation * CAMERA_ORIGIN_RELATIVE_TO_TURRET_ORIGIN;

    cameraFrame.setOrientation(at_cv_update_field_orientation_relative_to_chassis_orientation);
    cameraFrame.setOrigin(at_cv_update_camera_origin_relative_to_gimbal_origin);
#else
    UNUSED(gimbalYawAngle);
    UNUSED(gimbalPitchAngle);
    UNUSED(angleUnit);
#endif
}
}  // namespace src::Informants::Transformers