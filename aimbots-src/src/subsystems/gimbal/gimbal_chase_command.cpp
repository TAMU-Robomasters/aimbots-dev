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

void GimbalChaseCommand::initialize() {
    // // Set initial target angle to be the current angle for seamless switching between different gimbal commands
    // controller->setTargetYaw(AngleUnit::Radians, gimbal->getCurrentYawAxisAngle(AngleUnit::Radians));
    // controller->setTargetPitch(AngleUnit::Radians, gimbal->getCurrentPitchAxisAngle(AngleUnit::Radians));
}

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
    float quickTurnOffset = 0.0f;

    if (drivers->remote.keyPressed(Remote::Key::Q)) wasQPressed = true;

    if (wasQPressed && !drivers->remote.keyPressed(Remote::Key::Q)) {
        wasQPressed = false;
        quickTurnOffset -= M_PI_2;
    }

    if (drivers->remote.keyPressed(Remote::Key::E)) wasEPressed = true;

    if (wasEPressed && !drivers->remote.keyPressed(Remote::Key::E)) {
        wasEPressed = false;
        quickTurnOffset += M_PI_2;
    }

    float targetYawAxisAngle = 0.0f;
    float targetPitchAxisAngle = 0.0f;

    std::optional<src::Utils::Ballistics::BallisticsSolver::BallisticsSolution> ballisticsSolution =
        ballisticsSolver->solve();  // returns nullopt if no solution is available

    if (ballisticsSolution != std::nullopt) {
        // Convert ballistics solutions to field-relative angles
        targetYawAxisAngle =
            drivers->kinematicInformant.getChassisIMUAngle(src::Informants::AngularAxis::YAW_AXIS, AngleUnit::Radians) +
            ballisticsSolution->yawAngle;

        targetPitchAxisAngle =
            drivers->kinematicInformant.getChassisIMUAngle(src::Informants::AngularAxis::PITCH_AXIS, AngleUnit::Radians) +
            ballisticsSolution->pitchAngle;

        bSolTargetYawDisplay = modm::toDegree(targetYawAxisAngle);
        bSolTargetPitchDisplay = modm::toDegree(targetPitchAxisAngle);
        bSolDistanceDisplay = ballisticsSolution->distanceToTarget;

        controller->setTargetYaw(AngleUnit::Radians, targetYawAxisAngle);
        controller->setTargetPitch(AngleUnit::Radians, targetPitchAxisAngle);

        controller->runYawController(
            src::Utils::Ballistics::TARGET_DISTANCE_TO_YAW_VELOCITY_LIMIT.interpolate(ballisticsSolution->distanceToTarget));
        controller->runPitchController();
    } else {
        // Yaw counterclockwise is positive angle
        targetYawAxisAngle = controller->getTargetYaw(AngleUnit::Radians) + quickTurnOffset -
                             drivers->controlOperatorInterface.getGimbalYawInput();
        // Pitch up is positive angle
        targetPitchAxisAngle =
            controller->getTargetPitch(AngleUnit::Radians) + drivers->controlOperatorInterface.getGimbalPitchInput();

        gimbalPitchInputDisplay = drivers->controlOperatorInterface.getGimbalPitchInput();

        controller->setTargetYaw(AngleUnit::Radians, targetYawAxisAngle);
        controller->setTargetPitch(AngleUnit::Radians, targetPitchAxisAngle);

        controller->runYawController();
        controller->runPitchController();
    }

    targetYawAxisAngleDisplay2 = controller->getTargetYaw(AngleUnit::Degrees);
    targetPitchAxisAngleDisplay2 = controller->getTargetPitch(AngleUnit::Degrees);

    chassisRelativeYawAngleDisplay = gimbal->getCurrentYawAxisAngle(AngleUnit::Degrees);
    chassisRelativePitchAngleDisplay = gimbal->getCurrentPitchAxisAngle(AngleUnit::Degrees);
}

bool GimbalChaseCommand::isReady() { return true; }

bool GimbalChaseCommand::isFinished() const { return false; }

void GimbalChaseCommand::end(bool) {
    gimbal->setAllDesiredYawMotorOutputs(0);
    gimbal->setAllDesiredPitchOutputs(0);
}

};  // namespace src::Gimbal