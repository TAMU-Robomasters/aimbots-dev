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

float yawVelocityFeedforwardDisplay = 0.0f;
float pitchVelocityFeedforwardDisplay = 0.0f;

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
    controller->runYawVelocityController();
    controller->runPitchVelocityController();
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
        return pitchConfig.velocityAmplitudeDegreesPerSec;
    } else {
        return -pitchConfig.velocityAmplitudeDegreesPerSec;
    }
}

/**
 * @brief Runs the yaw motor's velocity as a ramping up square wave. 
 * Run motors at 1000 then -1000 then 2000 then -2000 and so on until 25,000.
 * 
 * @param periodSeconds
 * How long the entire ramp sequence is
 */
void GimbalVelocityTunningCommand::runYawVelocityStepOscillation(float periodSeconds) {
    uint32_t periodForSingleStep_ms = periodSeconds * 1E3 / 50.0f;
    float stepSize = 1000.0f;

    for (size_t i = 0; i < 26; i++) {
        uint32_t stepNumber = fmod(getRelativeTime() / periodForSingleStep_ms, 51);
        if ((i == stepNumber / 2) && (stepNumber % 2 == 0)) {
            yawVelocityFeedforwardDisplay = stepSize*i;
            gimbal->setDesiredYawMotorOutput(0, stepSize*i);
        }
        else if ((i == stepNumber / 2) && (stepNumber % 2 == 1)) {
            yawVelocityFeedforwardDisplay = -stepSize*i;
            gimbal->setDesiredYawMotorOutput(0, -stepSize*i);
        }
    }
}

void GimbalVelocityTunningCommand::runPitchVelocityStepOscillation(float periodSeconds) {
    return;
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