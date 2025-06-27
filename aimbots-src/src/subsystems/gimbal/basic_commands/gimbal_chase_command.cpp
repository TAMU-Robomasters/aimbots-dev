#include "gimbal_chase_command.hpp"

#include "utils/ballistics/ballistics_solver.hpp"
#ifdef GIMBAL_COMPATIBLE

namespace src::Gimbal {
// feed chassis relative controller for sentry, field relative for ground robots
GimbalChaseCommand::GimbalChaseCommand(
    src::Drivers* drivers,
    GimbalSubsystem* gimbalSubsystem,
    GimbalFieldRelativeController* gimbalController,
    GimbalFieldRelativeController* cvGimbalController,
    src::Utils::RefereeHelperTurreted* refHelper,
    src::Utils::Ballistics::BallisticsSolver* ballisticsSolver,
    float defaultLaunchSpeed)
    : tap::control::Command(),
      drivers(drivers),
      gimbal(gimbalSubsystem),
      controller(gimbalController),
      cvController(cvGimbalController),
      refHelper(refHelper),
      ballisticsSolver(ballisticsSolver),
      defaultLaunchSpeed(defaultLaunchSpeed) ,
      yawVelocityFilter(),
      yawBallisticsFilter(0.1),
      pitchVelocityFilter(),
      pitchBallisticsFilter(0.1)  //
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

float chassisRelativeYawVelocityDisplay = 0;
float chassisRelativePitchVelocityDisplay = 0;

float bSolTargetYawDisplay = 0.0f;
float bSolTargetPitchDisplay = 0.0f;
float bSolDistanceDisplay = 0.0f;

float targetYawAxisVelocityDisplay = 0.0f;
float targetYawAxisAccelerationDisplay = 0.0f;

float targetPitchAxisAngleDisplay = 0.0f;
float targetPitchAxisVelocityDisplay = 0.0f;
float targetPitchAxisAccelerationDisplay = 0.0f;

float gimbalPitchInputDisplay = 0.0f;

float currBarrelSpeedDisplay = -1;

float timestampDisplay;

float predictedProjectileSpeedDisplay = 0.0f;

float yawRawDisplay;
float pitchRawDisplay;

float yawAtFrameDelayDisplay;
float pitchAtFrameDelayDisplay;

float yawVelocityChaseOffset = 0.0f;

float yawVelocityDebug = 0.0f;
float yawAccelerationDebug = 0.0f;
bool updateYawDebug = false;

bool trackDebug = false;

bool ballisticsSolutionDisplay = false;

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

    float projectileSpeed = refHelper->getPredictedProjectileSpeed().value_or(defaultLaunchSpeed);
    // projectileSpeed = 30.0f;

    if (projectileSpeed == 0) {
        projectileSpeed = 25;
    }

    predictedProjectileSpeedDisplay = projectileSpeed;

    std::optional<src::Utils::Ballistics::BallisticsSolver::BallisticsSolution> ballisticsSolution =
        ballisticsSolver->solve(projectileSpeed);  // returns nullopt if no solution is available

    if (ballisticsSolution != std::nullopt) {
        ballisticsSolutionDisplay = true;
        // Convert ballistics solutions to field-relative angles
        uint32_t frameCaptureDelay = drivers->cvCommunicator.getLastFrameCaptureDelay();

        std::pair<float, float> fieldTurretAngleAtFrameDelay =
            drivers->kinematicInformant.getGimbalFieldOrientationAtTime(frameCaptureDelay);

        // yawAtFrameDelayDisplay = chassisIMUAngleAtFrameDelay.getZ();
        // pitchAtFrameDelayDisplay = chassisIMUAngleAtFrameDelay.getX();
        yawAtFrameDelayDisplay = modm::toDegree(fieldTurretAngleAtFrameDelay.first);
        pitchAtFrameDelayDisplay = modm::toDegree(fieldTurretAngleAtFrameDelay.second);

        yawRawDisplay =
            drivers->kinematicInformant.getChassisIMUAngle(Informants::AngularAxis::YAW_AXIS, AngleUnit::Radians);
        pitchRawDisplay =
            drivers->kinematicInformant.getChassisIMUAngle(Informants::AngularAxis::PITCH_AXIS, AngleUnit::Radians);

        targetYawAxisAngle =  ballisticsSolution->yawAngle;
        targetPitchAxisAngle = ballisticsSolution->pitchAngle;

        // Smooth out angles on the distance to the target
        // further distance = more smoothing
        //TODO: Not really sure if this does a whole lot since kf should be doing smoothing
        float exponentialTerm = 0.853;
        float a = 6.2;
        float c = 12.1;
        float alpha = (1/exponentialTerm) * log(-(ballisticsSolution->horizontalDistanceToTarget - c) / a);
        yawBallisticsFilter.setAlpha(alpha);
        yawBallisticsFilter.update(targetYawAxisAngle);

        exponentialTerm = 0.853;
        a = 6.2;
        c = 12.1;
        alpha = (1/exponentialTerm) * log(-(ballisticsSolution->horizontalDistanceToTarget - c) / a);
        yawBallisticsFilter.setAlpha(alpha);
        yawBallisticsFilter.update(targetPitchAxisAngle);

        // targetYawAxisAngle = yawBallisticsFilter.getValue();

        // targetYawAxisAngle =
        //     drivers->kinematicInformant.getChassisIMUAngle(Informants::AngularAxis::YAW_AXIS, AngleUnit::Radians) +
        //     ballisticsSolution->yawAngle;
        // targetPitchAxisAngle =
        //     drivers->kinematicInformant.getChassisIMUAngle(Informants::AngularAxis::PITCH_AXIS, AngleUnit::Radians) +
        //     ballisticsSolution->pitchAngle;

        bSolTargetYawDisplay = modm::toDegree(targetYawAxisAngle);
        bSolTargetPitchDisplay = modm::toDegree(targetPitchAxisAngle);
        bSolDistanceDisplay = ballisticsSolution->horizontalDistanceToTarget;

        // Comment when Z axis stops being silly
        // targetPitchAxisAngle =
        //     controller->getTargetPitch(AngleUnit::Radians) + drivers->controlOperatorInterface.getGimbalPitchInput();

        cvController->setTargetYaw(AngleUnit::Radians, targetYawAxisAngle);
        cvController->setTargetPitch(AngleUnit::Radians, targetPitchAxisAngle);
        
        isTargetBeingTracked = drivers->cvCommunicator.isTargetBeingTracked();

        if (!isTargetBeingTracked) {
            yawVelocityFilter.reset();
            yawAccelerationFilter.reset();
            pitchVelocityFilter.reset();
            pitchAccelerationFilter.reset();
        }

        // Angle filtering for feedforward control
        if (previousTargetYawAngle == -1000) {
            previousTargetYawAngle = targetYawAxisAngle;
            lastBallisticsSolutionTimeStamp_uS = currTime_uS;
        }
        else {
            currTime_uS = tap::arch::clock::getTimeMicroseconds();
            dt = (currTime_uS - lastBallisticsSolutionTimeStamp_uS) * 1E-6;
            lastBallisticsSolutionTimeStamp_uS = currTime_uS;
            yawVelocity = calcDerivative(previousTargetYawAngle, targetYawAxisAngle, dt);
            previousTargetYawAngle = targetYawAxisAngle;
            yawVelocity = yawVelocityFilter.processSample(yawVelocity);
            targetYawAxisVelocityDisplay = modm::toDegree(yawVelocity);

            pitchVelocity = calcDerivative(previousTargetPitchAngle, targetPitchAxisAngle, dt);
            previousTargetPitchAngle = targetPitchAxisAngle;
            pitchVelocity = pitchVelocityFilter.processSample(pitchVelocity);
            targetPitchAxisVelocityDisplay = modm::toDegree(pitchVelocity);

            if (previousYawVelocity == -1E6) previousYawVelocity = yawVelocity;
            else {
                yawAcceleration = calcDerivative(previousYawVelocity, yawVelocity, dt);
                previousYawVelocity = yawVelocity;
                yawAcceleration = yawAccelerationFilter.processSample(yawAcceleration);
                targetYawAxisAccelerationDisplay = modm::toDegree(yawAcceleration);

                pitchAcceleration = calcDerivative(previousPitchVelocity, pitchVelocity, dt);
                previousPitchVelocity = pitchVelocity;
                pitchAcceleration = pitchAccelerationFilter.processSample(pitchAcceleration);
                targetPitchAxisAccelerationDisplay = modm::toDegree(pitchAcceleration);
            }
        }

        
        if (updateYawDebug) {
            yawVelocity = yawVelocityDebug;
            yawAcceleration = yawAccelerationDebug;

            updateYawDebug = false;
        }
        // cvController->setTargetVelocityYaw(AngleUnit::Radians, yawVelocity);

        trackDebug = isTargetBeingTracked;
        if (isTargetBeingTracked) {
            cvController->setTargetVelocityYaw(AngleUnit::Radians, yawVelocity);
            cvController->setTargetVelocityPitch(AngleUnit::Radians, pitchVelocity);
            cvController->runYawController(20.0f);
            cvController->runPitchController(6.0f);
        }
        // cvController->runYawVelocityController(5.0f);
        // cvController->runYawController(
        //     src::Utils::Ballistics::YAW_VELOCITY_LIMITER.interpolate(ballisticsSolution->distanceToTarget));
        // cvController->runPitchController(5.0f);
    } else {
        //TODO: maybe make it so that the gimbal stays at the same position
        // Yaw counterclockwise is positive angle
        ballisticsSolutionDisplay = false;
        
        controller->setTargetYaw(
                AngleUnit::Radians,
                controller->getTargetYaw(AngleUnit::Radians) + quickTurnOffset -
                    drivers->controlOperatorInterface.getGimbalYawInput());

        controller->setTargetPitch(
            AngleUnit::Radians,
            controller->getTargetPitch(AngleUnit::Radians) + drivers->controlOperatorInterface.getGimbalPitchInput());
        controller->runYawController(6.0f);
        controller->runPitchController(6.0f);

        // controller->setTargetYaw(AngleUnit::Radians, targetYawAxisAngle);
        // controller->setTargetPitch(AngleUnit::Radians, targetPitchAxisAngle);

        // controller->runYawController();
        // controller->runPitchController();
    }

    targetYawAxisAngleDisplay2 = controller->getTargetYaw(AngleUnit::Degrees);      // uncomment later
    targetPitchAxisAngleDisplay2 = controller->getTargetPitch(AngleUnit::Degrees);  // uncomment later

    chassisRelativeYawAngleDisplay = gimbal->getCurrentYawAxisAngle(AngleUnit::Degrees);
    chassisRelativePitchAngleDisplay = gimbal->getCurrentPitchAxisAngle(AngleUnit::Degrees);

    chassisRelativeYawVelocityDisplay = modm::toDegree(RPM_TO_RADPS(gimbal->getYawMotorRPM(0))); 
    chassisRelativePitchVelocityDisplay = modm::toDegree(RPM_TO_RADPS(gimbal->getPitchMotorRPM(0)));
}

bool GimbalChaseCommand::isReady() { return true; }

bool GimbalChaseCommand::isFinished() const { return false; }

void GimbalChaseCommand::end(bool) {
    gimbal->setAllDesiredYawMotorOutputs(0);
    gimbal->setAllDesiredPitchMotorOutputs(0);
}

};  // namespace src::Gimbal

#endif