#include "informants/kinematics/turret_frames.hpp"

#include "informants/kinematics/coordinate_frame.hpp"
#include "modm/math/geometry/angle.hpp"
#include "utils/tools/common_types.hpp"
#include "utils/math/matrix_helpers.hpp"

#include "utils/tools/robot_specific_defines.hpp"
#include "subsystems/gimbal/gimbal_constants.hpp"
#include "subsystems/chassis/chassis_constants.hpp"

//#include "utils/math/transform_setup.hpp"
#include "utils/math/transformations.hpp"


using namespace src::Utils::MatrixHelper;
namespace src::Informants::Transformers {

TurretFrames::TurretFrames() {
#ifndef TARGET_TURRET
    turretFieldFrame.setOrigin(Vector3f(0, 0, 0));
    turretFieldFrame.setOrientation(Matrix3f::identityMatrix());

    // update frames to initial values
    updateFrames(0, 0, 0, AngleUnit::Radians);

    turretCameraFrame.setOrientation(field_orientation_relative_to_chassis_orientation);
    turretCameraFrame.setOrigin(camera_origin_relative_to_gimbal_origin);
#endif
}

void TurretFrames::updateFrames(float fieldYaw, float fieldPitch, float fieldRoll, AngleUnit angleUnit) {
    UNUSED(fieldPitch);
    UNUSED(fieldRoll);

#ifndef TARGET_TURRET
    /*this->field_orientation_relative_to_chassis_orientation =
        rotationMatrix(fieldYaw, Z_AXIS, angleUnit);*/ /* rotationMatrix(fieldPitch, X_AXIS, angleUnit) *
        rotationMatrix(fieldRoll, Y_AXIS, angleUnit);*/  // field to chassis rotation
    UNUSED(fieldYaw);
    UNUSED(angleUnit);
    //  Don't need to update field frame origin, should be the center of this coordinate framework
    // turretFieldFrame.setOrientation(Matrix3f::identityMatrix());
    // turretFieldFrame.setOrientation(field_orientation_relative_to_chassis_orientation);

#else
    UNUSED(fieldYaw);
    UNUSED(angleUnit);
#endif
}

float camera_yaw_offset_angle_degrees = 0.0f;
float camera_pitch_offset_angle_degrees = 6.13399982;

float gimbalYawDisplay = 0.0f;
float gimbalPitchDisplay = 0.0f;

Matrix3f cameraOrientationMatrixDisplay = Matrix3f::zeroMatrix();

void TurretFrames::mirrorPastCameraFrame(float gimbalYawAngle, float gimbalPitchAngle, AngleUnit angleUnit) {
    gimbalYawDisplay = modm::toDegree(gimbalYawAngle);
    gimbalPitchDisplay = modm::toDegree(gimbalPitchAngle);

#ifndef TARGET_TURRET
    Matrix3f at_cv_update_field_orientation_relative_to_chassis_orientation =
        rotationMatrix(gimbalYawAngle, Z_AXIS, angleUnit) * rotationMatrix(gimbalPitchAngle, X_AXIS, angleUnit);
    Matrix3f camera_orientation_offset_matrix = rotationMatrix(modm::toRadian(camera_yaw_offset_angle_degrees), Z_AXIS, angleUnit) * rotationMatrix(modm::toRadian(camera_pitch_offset_angle_degrees), X_AXIS, angleUnit);
    Vector3f at_cv_update_camera_origin_relative_to_gimbal_origin =
        /*field_orientation_relative_to_chassis_orientation*/
        at_cv_update_field_orientation_relative_to_chassis_orientation * CAMERA_ORIGIN_RELATIVE_TO_TURRET_ORIGIN;

    cameraOrientationMatrixDisplay = at_cv_update_field_orientation_relative_to_chassis_orientation;

    turretCameraFrame.setOrientation(at_cv_update_field_orientation_relative_to_chassis_orientation * camera_orientation_offset_matrix);
    turretCameraFrame.setOrigin(at_cv_update_camera_origin_relative_to_gimbal_origin);
#else
    UNUSED(gimbalYawAngle);
    UNUSED(gimbalPitchAngle);
    UNUSED(angleUnit);
#endif
}
}  // namespace src::Informants::Transformers
