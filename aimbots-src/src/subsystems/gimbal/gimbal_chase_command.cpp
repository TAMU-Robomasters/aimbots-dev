#include "gimbal_chase_command.hpp"

#include "utils/ballistics_solver.hpp"

namespace src::Gimbal {
// feed chassis relative controller for sentry, field relative for ground robots
GimbalChaseCommand::GimbalChaseCommand(
    src::Drivers* drivers,
    GimbalSubsystem* gimbalSubsystem,
    GimbalControllerInterface* gimbalController,
    src::Utils::Ballistics::BallisticsSolver* ballisticsSolver)
    : tap::control::Command(),
      drivers(drivers),
      gimbal(gimbalSubsystem),
      controller(gimbalController),
      ballisticsSolver(ballisticsSolver)  //
{
    addSubsystemRequirement(dynamic_cast<tap::control::Subsystem*>(gimbal));
}

void GimbalChaseCommand::initialize() {}

float targetPitchAxisAngleDisplay2 = 0.0f;
float targetYawAxisAngleDisplay2 = 0.0f;

float yawOffsetDisplay = 0.0f;
float pitchOffsetDisplay = 0.0f;

float chassisRelativeYawAngleDisplay = 0;
float chassisRelativePitchAngleDisplay = 0;

float bSolTargetYawDisplay = 0.0f;
float bSolTargetPitchDisplay = 0.0f;
float bSolDistanceDisplay = 0.0f;

float gimbalPitchInputDisplay = 0.0f;

float timestampDisplay;

void GimbalChaseCommand::execute() {
    float targetYawAxisAngle = 0.0f;
    float targetPitchAxisAngle = 0.0f;

    std::optional<src::Utils::Ballistics::BallisticsSolver::BallisticsSolution> ballisticsSolution =
        ballisticsSolver->solve();  // returns last solution if no new solution is available

    if (ballisticsSolution != std::nullopt) {
        targetYawAxisAngle = ballisticsSolution->yawAngle;
        targetPitchAxisAngle = ballisticsSolution->pitchAngle;

        // ballistics returns angles between [0, 2PI), need to convert idk why
        // targetYawAxisAngle = M_PI_2 + modm::toRadian(YAW_OFFSET_ANGLE) - targetYawAxisAngle;
        // targetPitchAxisAngle += modm::toRadian(PITCH_OFFSET_ANGLE);

        bSolTargetYawDisplay = modm::toDegree(targetYawAxisAngle);
        bSolTargetPitchDisplay = modm::toDegree(targetPitchAxisAngle);
        bSolDistanceDisplay = ballisticsSolution->distanceToTarget;
    } else {
        // Yaw counterclockwise is positive angle
        targetYawAxisAngle =
            gimbal->getTargetYawAxisAngle(AngleUnit::Radians) - drivers->controlOperatorInterface.getGimbalYawInput();
        // Pitch up is positive angle
        targetPitchAxisAngle =
            gimbal->getTargetPitchAxisAngle(AngleUnit::Radians) + drivers->controlOperatorInterface.getGimbalPitchInput();

        gimbalPitchInputDisplay = drivers->controlOperatorInterface.getGimbalPitchInput();
    }

    targetYawAxisAngleDisplay2 = gimbal->getTargetYawAxisAngle(AngleUnit::Degrees);
    targetPitchAxisAngleDisplay2 = gimbal->getTargetPitchAxisAngle(AngleUnit::Degrees);

    chassisRelativeYawAngleDisplay = gimbal->getCurrentYawAxisAngle(AngleUnit::Degrees);
    chassisRelativePitchAngleDisplay = gimbal->getCurrentPitchAxisAngle(AngleUnit::Degrees);

    controller->runYawController(AngleUnit::Radians, targetYawAxisAngle, false);
    controller->runPitchController(AngleUnit::Radians, targetPitchAxisAngle, false);
}

bool GimbalChaseCommand::isReady() { return true; }

bool GimbalChaseCommand::isFinished() const { return false; }

void GimbalChaseCommand::end(bool) {
    gimbal->setAllDesiredYawMotorOutputs(0);
    gimbal->setAllDesiredPitchOutputs(0);
}

};  // namespace src::Gimbal