#include "gimbal_chase_command.hpp"

#include "utils/ballistics_solver.hpp"

namespace src::Gimbal {
// feed chassis relative controller for sentry, field relative for ground robots
GimbalChaseCommand::GimbalChaseCommand(
    src::Drivers* drivers,
    GimbalSubsystem* gimbalSubsystem,
    GimbalControllerInterface* gimbalController,
    src::Utils::BallisticsSolver* ballisticsSolver)
    : tap::control::Command(),
      drivers(drivers),
      gimbal(gimbalSubsystem),
      controller(gimbalController),
      ballisticsSolver(ballisticsSolver)  //
{
    addSubsystemRequirement(dynamic_cast<tap::control::Subsystem*>(gimbal));
}

void GimbalChaseCommand::initialize() {}

float targetPitchAngleDisplay2 = 0.0f;
float targetYawAngleDisplay2 = 0.0f;

float yawOffsetDisplay = 0.0f;
float pitchOffsetDisplay = 0.0f;

float chassisRelativeYawAngleDisplay = 0;
float chassisRelativePitchAngleDisplay = 0;

float bSolTargetYawDisplay = 0.0f;
float bSolTargetPitchDisplay = 0.0f;
float bSolDistanceDisplay = 0.0f;

float timestampDisplay;

void GimbalChaseCommand::execute() {
    float targetYawAngle = 0.0f;
    float targetPitchAngle = 0.0f;

    std::optional<src::Utils::BallisticsSolver::BallisticsSolution> ballisticsSolution = ballisticsSolver->solve();

    if (ballisticsSolution != std::nullopt) {
        targetYawAngle = ballisticsSolution->yawAngle;
        targetPitchAngle = ballisticsSolution->pitchAngle;

        // ballistics returns angles between [0, 2PI), need to convert idk why
        targetYawAngle = M_PI_2 + modm::toRadian(YAW_START_ANGLE) - targetYawAngle;
        targetPitchAngle += modm::toRadian(PITCH_START_ANGLE);

        bSolTargetYawDisplay = modm::toDegree(targetYawAngle);
        bSolTargetPitchDisplay = modm::toDegree(targetPitchAngle);
        bSolDistanceDisplay = ballisticsSolution->distanceToTarget;
    } else {
        targetYawAngle = modm::toRadian(
            gimbal->getTargetChassisRelativeYawAngle(AngleUnit::Degrees) +
            drivers->controlOperatorInterface.getGimbalYawInput());
        targetPitchAngle = modm::toRadian(
            gimbal->getTargetChassisRelativePitchAngle(AngleUnit::Degrees) +
            drivers->controlOperatorInterface.getGimbalPitchInput());
    }

    targetYawAngleDisplay2 = modm::toDegree(targetYawAngle);
    targetPitchAngleDisplay2 = modm::toDegree(targetPitchAngle);

    // targetYawAngle = modm::toDegree(visionTargetAngles[0][src::Informants::vision::yaw]);
    // targetPitchAngle = modm::toDegree(visionTargetAngles[0][src::Informants::vision::pitch]);
    // targetYawAngle = aimAtAngles.yaw;
    // targetPitchAngle = aimAtAngles.pitch;

    // yawOffsetDisplay = modm::toDegree(drivers->cvCommunicator.getLastValidMessage().targetYawOffset);
    // pitchOffsetDisplay = modm::toDegree(drivers->cvCommunicator.getLastValidMessage().targetPitchOffset);

    // fieldRelativeYawAngleDisplay = gimbal->getCurrentFieldRelativeYawAngle(AngleUnit::Degrees);
    // fieldRelativeYawAngleDisplay = gimbal->getCurrentYawAngleFromChassisCenter(AngleUnit::Degrees);
    chassisRelativePitchAngleDisplay = gimbal->getCurrentChassisRelativePitchAngle(AngleUnit::Degrees);
    chassisRelativeYawAngleDisplay = gimbal->getCurrentYawAngleFromChassisCenter(AngleUnit::Degrees);

    controller->runYawController(AngleUnit::Radians, targetYawAngle, false);
    controller->runPitchController(AngleUnit::Radians, targetPitchAngle, false);
}

bool GimbalChaseCommand::isReady() { return true; }

bool GimbalChaseCommand::isFinished() const { return false; }

void GimbalChaseCommand::end(bool) {
    gimbal->setYawMotorOutput(0);
    gimbal->setPitchMotorOutput(0);
}
};  // namespace src::Gimbal