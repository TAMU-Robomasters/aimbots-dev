#include "informants/RobotFrames.hpp"

#include "informants/CoordinateFrame.hpp"
#include "utils/common_types.hpp"
#include "utils/math/matrix_helpers.hpp"
#include "utils/math/transform_setup.hpp"

namespace src::Informants {
// Look sid Idk how to do this exactly
Matrix<float, 3, 3> I = identityMatrix();
Vector3f O = {0, 0, 0};

RobotFrames::RobotFrames(
    float YAW_START_ANGLE,  // What angle do we aim at when starting?
    float PITCH_START_ANGLE,
    float CHASSIS_ANGLE_RELATIVE_TO_WORLD,
    Vector3f TURRET_ORIGIN_RELATIVE_TO_CHASSIS_ORIGIN,
    Vector3f CAMERA_ORIGIN_RELATIVE_TO_TURRET_ORIGIN,
    Vector3f CHASSIS_ORIGIN_RELATIVE_TO_WORLD_ORIGIN) {
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
    Matrix<float, 3, 3> CHASSIS_ORIENTATION_RELATIVE_TO_WORLD_ORIENTATION =
        src::Utils::MatrixHelper::rotationMatrix(AngleUnit::Degrees, CHASSIS_ANGLE_RELATIVE_TO_WORLD, 2);
    // Store here instead of in Robot Constants
    this->TURRET_ORIGIN_RELATIVE_TO_CHASSIS_ORIGIN = TURRET_ORIGIN_RELATIVE_TO_CHASSIS_ORIGIN;
    this->CAMERA_ORIGIN_RELATIVE_TO_CHASSIS_ORIGIN = CAMERA_ORIGIN_RELATIVE_TO_CHASSIS_ORIGIN;

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
    this->TURRET_ORIENTATION_RELATIVE_TO_CHASSIS_ORIENTATION =
        src::Utils::MatrixHelper::rotationMatrix(AngleUnit::Degrees, YAW_START_ANGLE, 2) *
        src::Utils::MatrixHelper::rotationMatrix(AngleUnit::Degrees, PITCH_START_ANGLE, 0);
    // Double check this again but I think its good

    // Calculate where camera is relative to Chassis
    this->CAMERA_ORIGIN_RELATIVE_TO_CHASSIS_ORIGIN =
        TURRET_ORIGIN_RELATIVE_TO_CHASSIS_ORIGIN + this->TURRET_ORIENTATION_RELATIVE_TO_CHASSIS_ORIENTATION.asTranspose() *
                                                       this->CAMERA_ORIGIN_RELATIVE_TO_TURRET_ORIGIN;

    // Init Chassis Frame as Ground Frame
    this->chassisFrame = src::Informants::CoordinateFrame(I, O);
    // Init Ballistics Frame for Ballistics Math (Offset Vertically, Same Directions)
    this->ballisticsFrame = src::Informants::CoordinateFrame(I, -1 * this->TURRET_ORIGIN_RELATIVE_TO_CHASSIS_ORIGIN);

    // Init Gimbal Frame
    this->gimbalFrame = src::Informants::CoordinateFrame(
        this->TURRET_ORIENTATION_RELATIVE_TO_CHASSIS_ORIENTATION,
        -1 * this->TURRET_ORIGIN_RELATIVE_TO_CHASSIS_ORIGIN);
    // Init Camera Frame
    this->cameraFrame = src::Informants::CoordinateFrame(
        this->TURRET_ORIENTATION_RELATIVE_TO_CHASSIS_ORIENTATION,
        -1 * this->CAMERA_ORIGIN_RELATIVE_TO_CHASSIS_ORIGIN);
    // Init Field Frame
    this->fieldFrame = src::Informants::CoordinateFrame(
        CHASSIS_ORIENTATION_RELATIVE_TO_WORLD_ORIENTATION,
        CHASSIS_ORIGIN_RELATIVE_TO_WORLD_ORIGIN);
}

RobotFrames::~RobotFrames() {}

void RobotFrames::updateFrames(float yawAng, float pitchAng, Vector3f deltar) {
    this->TURRET_ORIENTATION_RELATIVE_TO_CHASSIS_ORIENTATION =
        src::Utils::MatrixHelper::rotationMatrix(AngleUnit::Degrees, yawAng, 2) *
        src::Utils::MatrixHelper::rotationMatrix(AngleUnit::Degrees, pitchAng, 0);
    this->CAMERA_ORIGIN_RELATIVE_TO_CHASSIS_ORIGIN =
        TURRET_ORIGIN_RELATIVE_TO_CHASSIS_ORIGIN +
        this->TURRET_ORIENTATION_RELATIVE_TO_CHASSIS_ORIENTATION.asTranspose() * CAMERA_ORIGIN_RELATIVE_TO_TURRET_ORIGIN;
    this->gimbalFrame.setOrientation(TURRET_ORIENTATION_RELATIVE_TO_CHASSIS_ORIENTATION);
    this->cameraFrame.setOrientation(TURRET_ORIENTATION_RELATIVE_TO_CHASSIS_ORIENTATION);
    this->cameraFrame.setOrigin(-1 * this->CAMERA_ORIGIN_RELATIVE_TO_CHASSIS_ORIGIN);

    // this->fieldFrame.setOrientation(???);
    // this->fieldFrame.moveOrigin(-deltar);
}
}  // namespace src::Informants