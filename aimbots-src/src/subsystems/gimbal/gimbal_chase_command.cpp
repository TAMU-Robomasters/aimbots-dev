#include "gimbal_chase_command.hpp"

namespace src::Gimbal {
// feed chassis relative controller for sentry, field relative for ground robots
GimbalChaseCommand::GimbalChaseCommand(src::Drivers* drivers,
                                       GimbalSubsystem* gimbalSubsystem,
                                       GimbalControllerInterface* gimbalController)
    : tap::control::Command(),
      drivers(drivers),
      gimbal(gimbalSubsystem),
      controller(gimbalController)  //
{
    addSubsystemRequirement(dynamic_cast<tap::control::Subsystem*>(gimbal));
}

void GimbalChaseCommand::initialize() {}

float targetPitchAngleDisplay2 = 0.0f;
float targetYawAngleDisplay2 = 0.0f;

float yawOffsetDisplay = 0.0f;
float pitchOffsetDisplay = 0.0f;

float fieldRelativeYawAngleDisplay = 0;
float chassisRelativePitchAngleDisplay = 0;

src::Informants::vision::CVState cvStateDisplay = src::Informants::vision::CVState::FOUND;

bool jetsonOnlineDisplay = false;

void GimbalChaseCommand::execute() {
    float targetYawAngle = 0.0f;
    float targetPitchAngle = 0.0f;

    Matrix<float, 1, 2> visionTargetAngles = Matrix<float, 1, 2>::zeroMatrix();
    src::Informants::vision::CVState cvState;

    jetsonOnlineDisplay = false;
    if (drivers->cvCommunicator.isJetsonOnline()) {
        jetsonOnlineDisplay = true;
        cvState = drivers->cvCommunicator.getLastValidMessage().cvState;
        // if (cvState == src::Informants::vision::CVState::FIRE) {
        visionTargetAngles = drivers->cvCommunicator.getVisionTargetAngles();

        cvStateDisplay = cvState;

        

        targetYawAngle = modm::toDegree(visionTargetAngles[0][src::Informants::vision::yaw]);
        targetPitchAngle = modm::toDegree(visionTargetAngles[0][src::Informants::vision::pitch]);

        yawOffsetDisplay = modm::toDegree(drivers->cvCommunicator.getLastValidMessage().targetYawOffset);
        pitchOffsetDisplay = modm::toDegree(drivers->cvCommunicator.getLastValidMessage().targetPitchOffset);

        fieldRelativeYawAngleDisplay = gimbal->getCurrentFieldRelativeYawAngle(AngleUnit::Degrees);
        chassisRelativePitchAngleDisplay = gimbal->getCurrentChassisRelativePitchAngle(AngleUnit::Degrees);

        targetYawAngleDisplay2 = targetYawAngle;
        targetPitchAngleDisplay2 = targetPitchAngle;
        // }

        //Should be absolute angle, will double check
        controller->runYawController(AngleUnit::Degrees, targetYawAngle, true);
        controller->runPitchController(AngleUnit::Degrees, targetPitchAngle, true);
    }
}

bool GimbalChaseCommand::isReady() { return true; }

bool GimbalChaseCommand::isFinished() const { return false; }

void GimbalChaseCommand::end(bool) {
    gimbal->setYawMotorOutput(0);
    gimbal->setPitchMotorOutput(0);
}
};  // namespace src::Gimbal