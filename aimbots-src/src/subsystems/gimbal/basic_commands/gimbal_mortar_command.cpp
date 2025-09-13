#include <cmath>
#include "gimbal_mortar_command.hpp"
#include "utils/tools/common_types.hpp"

#ifdef GIMBAL_COMPATIBLE
namespace src::Gimbal {

float targetYawAxisAngle = 0.0f;
float targetPitchAxisAngle = 0.0f;

float mortarYawDisplay = 0.0f;
float mortarPitchDisplay = 0.0f;

GimbalMortarCommand::GimbalMortarCommand(
    src::Drivers* drivers, 
    GimbalSubsystem* gimbalSubsystem,
    GimbalFieldRelativeController* controller, 
    // src::Utils::Ballistics::BallisticsSolver* ballisticsSolver,
    // src::Utils::RefereeHelperTurreted* refHelper,
    MortarCommandConfig config
    /*float defaultLaunchSpeed*/)
    : drivers(drivers), 
      gimbal(gimbalSubsystem), 
      controller(controller), 
    //   ballisticsSolver(ballisticsSolver),
    //   refHelper(refHelper),
      config(config)
    //   defaultLaunchSpeed(defaultLaunchSpeed)
    {
        addSubsystemRequirement(dynamic_cast<tap::control::Subsystem*>(gimbal));
    }


void GimbalMortarCommand::execute() {
    // float projectileSpeed = refHelper->getPredictedProjectileSpeed().value_or(defaultLaunchSpeed);
    // predictedProjectileSpeedDisplay = projectileSpeed;
    // std::optional<src::Utils::Ballistics::BallisticsSolver::BallisticsSolution> ballisticsSolution =
    //     ballisticsSolver->solve(projectileSpeed);  // returns nullopt if no solution is available

    // if (ballisticsSolution != std::nullopt) {
    //     targetYawAxisAngle =  ballisticsSolution->yawAngle;
    //     targetPitchAxisAngle = ballisticsSolution->pitchAngle;
    // } else {
        targetYawAxisAngle = 0.0f;
        targetPitchAxisAngle = modm::toRadian(config.pitchAngleDegrees);
    // }
    targetYawAxisAngle += getYawTargetPosition();

    controller->setTargetYaw(AngleUnit::Radians, targetYawAxisAngle);
    controller->setTargetPitch(AngleUnit::Radians, targetPitchAxisAngle);

    mortarYawDisplay = modm::toDegree(targetYawAxisAngle);
    mortarPitchDisplay = modm::toDegree(targetPitchAxisAngle);

    controller->runYawController(6.0f);
    controller->runPitchController(6.0f);
}

float GimbalMortarCommand::getYawTargetPosition() { // in degrees
    float periodMilliseconds = config.yawFrequencyPeriodSeconds * 1000.0f;
    float timeInPeriod = fmod(getRelativeTime(), periodMilliseconds);
    // Sine wave: amplitude * sin(2 * pi * t / T)
    return modm::toRadian(config.yawPositionAmplitudeDegrees) * std::sin(2.0f * M_PI * timeInPeriod / periodMilliseconds);
}

float GimbalMortarCommand::getRelativeTime() const {
    return tap::arch::clock::getTimeMilliseconds() - initTime;
}

void GimbalMortarCommand::initialize() {initTime = tap::arch::clock::getTimeMilliseconds();}

bool GimbalMortarCommand::isReady() { return true; }

bool GimbalMortarCommand::isFinished() const { return false; }

void GimbalMortarCommand::end(bool) {
    gimbal->setAllDesiredYawMotorOutputs(0);
    gimbal->setAllDesiredPitchMotorOutputs(0);
}

} // namespace src::Gimbal
#endif // #ifdef GIMBAL_COMPATIBLE 