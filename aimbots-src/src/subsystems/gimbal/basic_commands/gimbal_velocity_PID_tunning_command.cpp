#include <cmath>
#include "gimbal_velocity_PID_tunning_command.hpp"
#include "utils/tools/common_types.hpp"

#ifdef GIMBAL_COMPATIBLE
namespace src::Gimbal {

// watchable variables
bool updateYawVelocityConfigDebug = false;
float yawVelocityTunningFrequencyDebug = 0.0f;
float yawVelocityTunningAmplitudeDebug = 0.0f;

bool updatePitchVelocityConfigDebug = false;
float pitchVelocityTunningFrequencyDebug = 0.0f;
float pitchVelocityTunningAmplitudeDebug = 0.0f;

GimbalVelocityTunningCommand::GimbalVelocityTunningCommand(
    src::Drivers* drivers, 
    GimbalSubsystem* gimbalSubsystem,
    GimbalFieldRelativeController* controller, 
    GimbalVelocityTunningConfig yawConfig,
    GimbalVelocityTunningConfig pitchConfig)
    : drivers(drivers), 
      gimbal(gimbalSubsystem), 
      controller(controller), 
      yawConfig(yawConfig),
      pitchConfig(pitchConfig)  //

    {
        addSubsystemRequirement(dynamic_cast<tap::control::Subsystem*>(gimbal));
    }

float currGimbalTestTargetYawVelocityDisplay = 0.0f;
float currGimbalTestTargetPitchVelocityDisplay = 0.0f;

void GimbalVelocityTunningCommand::execute() {
    float yawTargetVelocity = getYawTargetVelocity();
    float pitchTargetVelocity = getPitchTargetVelocity();

    currGimbalTestTargetYawVelocityDisplay = yawTargetVelocity;
    currGimbalTestTargetPitchVelocityDisplay = pitchTargetVelocity;

    controller->setTargetVelocityYaw(AngleUnit::Degrees, yawTargetVelocity);
    controller->setTargetVelocityPitch(AngleUnit::Degrees, pitchTargetVelocity);
    controller->runYawVelocityController(5.0f);
    controller->runPitchVelocityController(5.0f);
}

float GimbalVelocityTunningCommand::getYawTargetVelocity() { // in degrees per second
    // For PID tunning through Ozone
    if (updateYawVelocityConfigDebug) {
        yawConfig.frequencyHz = yawVelocityTunningFrequencyDebug;
        yawConfig.velocityAmplitudeDegreesPerSec = yawVelocityTunningAmplitudeDebug;
        updateYawVelocityConfigDebug = false;
    }

    float periodMilliseconds = 1000.0f / yawConfig.frequencyHz;
    
    float timeInPeriod = fmod(getRelativeTime(), periodMilliseconds);
    
    if (timeInPeriod < periodMilliseconds / 2.0f) {
        return yawConfig.velocityAmplitudeDegreesPerSec;
    } else {
        return -yawConfig.velocityAmplitudeDegreesPerSec;
    }
}

float GimbalVelocityTunningCommand::getPitchTargetVelocity() { // in degrees per second
    // For PID tunning through Ozone
    if (updatePitchVelocityConfigDebug) {
        pitchConfig.frequencyHz = pitchVelocityTunningFrequencyDebug;
        pitchConfig.velocityAmplitudeDegreesPerSec = pitchVelocityTunningAmplitudeDebug;
        updatePitchVelocityConfigDebug = false;
    }

    float periodMilliseconds = 1000.0f / pitchConfig.frequencyHz;
    
    float timeInPeriod = fmod(getRelativeTime(), periodMilliseconds);
    
    if (timeInPeriod < periodMilliseconds / 2.0f) {
        return pitchConfig.velocityAmplitudeDegreesPerSec + 15.0f;
    } else {
        return -pitchConfig.velocityAmplitudeDegreesPerSec + 15.0f;
    }
}

void GimbalVelocityTunningCommand::initialize() { initTime = tap::arch::clock::getTimeMilliseconds(); }

bool GimbalVelocityTunningCommand::isReady() { return true; }

bool GimbalVelocityTunningCommand::isFinished() const { return false; }

void GimbalVelocityTunningCommand::end(bool) {
    gimbal->setAllDesiredYawMotorOutputs(0);
    gimbal->setAllDesiredPitchMotorOutputs(0);
}

} // namespace src::Gimbal
#endif // #ifdef GIMBAL_COMPATIBLE