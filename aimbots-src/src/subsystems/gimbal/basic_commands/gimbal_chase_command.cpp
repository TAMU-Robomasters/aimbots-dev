#include "gimbal_chase_command.hpp"

#include "utils/ballistics/ballistics_solver.hpp"
#ifdef GIMBAL_COMPATIBLE

namespace src::Gimbal {
// feed chassis relative controller for sentry, field relative for ground robots
GimbalChaseCommand::GimbalChaseCommand(
    src::Drivers* drivers,
    GimbalSubsystem* gimbalSubsystem,
    GimbalControllerInterface* gimbalController,
    src::Utils::RefereeHelperTurreted* refHelper,
    src::Utils::Ballistics::BallisticsSolver* ballisticsSolver,
    float defaultLaunchSpeed)
    : tap::control::Command(),
      drivers(drivers),
      gimbal(gimbalSubsystem),
      controller(gimbalController),
      refHelper(refHelper),
      ballisticsSolver(ballisticsSolver),
      defaultLaunchSpeed(defaultLaunchSpeed),
      desiredAngles({0.0f, 0.0f})  //
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

float currBarrelSpeedDisplay = -1;

float timestampDisplay;

float predictedProjectileSpeedDisplay = 0.0f;

float yawRawDisplay;
float pitchRawDisplay;

float yawAtFrameDelayDisplay;
float pitchAtFrameDelayDisplay;

float yawVelocityChaseOffset = 0.0f;

void GimbalChaseCommand::execute() {
    float quickTurnOffset = 0.0f;

    if (drivers->remote.keyPressed(Remote::Key::Q) && !ignoreQuickTurns) wasQPressed = true;

    if (wasQPressed && !drivers->remote.keyPressed(Remote::Key::Q)) {
        wasQPressed = false;
        quickTurnOffset += M_PI_2;
    }

    if (drivers->remote.keyPressed(Remote::Key::E) && !ignoreQuickTurns) wasEPressed = true;

    if (wasEPressed && !drivers->remote.keyPressed(Remote::Key::E)) {
        wasEPressed = false;
        quickTurnOffset -= M_PI_2;
    }

    float targetYawAxisAngle = 0.0f;
    float targetPitchAxisAngle = 0.0f;
    
    float targetYawChangeAngle = 0.0f;
    float targetPitchChangeAngle = 0.0f;

    float currFieldRelativeYawAngle = 0.0f;
    float currFieldRelativePitchAngle = 0.0f;

    float projectileSpeed = refHelper->getPredictedProjectileSpeed().value_or(0.0f);

    if (drivers->cvCommunicator.isJetsonOnline()) { 
        if (drivers->cvCommunicator.getLastValidMessage().cvState) { // update angles if we see target or we want to fire
            desiredAngles = drivers->cvCommunicator.getAutoAimAngles();   
        }
        controller->setTargetYaw(AngleUnit::Radians, desiredAngles.yaw);
        controller->setTargetPitch(AngleUnit::Radians, desiredAngles.pitch);
    
        // controller->runYawController(
        //     src::Utils::Ballistics::YAW_VELOCITY_LIMITER.interpolate(ballisticsSolution->distanceToTarget));
        controller->runYawController(100.0f);
        controller->runPitchController(5.0f);
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

    targetYawAxisAngleDisplay2 = controller->getTargetYaw(AngleUnit::Degrees);      // uncomment later
    targetPitchAxisAngleDisplay2 = controller->getTargetPitch(AngleUnit::Degrees);  // uncomment later

    chassisRelativeYawAngleDisplay = gimbal->getCurrentYawAxisAngle(AngleUnit::Degrees);
    chassisRelativePitchAngleDisplay = gimbal->getCurrentPitchAxisAngle(AngleUnit::Degrees);
}

bool GimbalChaseCommand::isReady() { return true; }

bool GimbalChaseCommand::isFinished() const { return false; }

void GimbalChaseCommand::end(bool) {
    gimbal->setAllDesiredYawMotorOutputs(0);
    gimbal->setAllDesiredPitchMotorOutputs(0);
}

};  // namespace src::Gimbal

#endif