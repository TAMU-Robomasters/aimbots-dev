#include <cmath>
#include "feeder_velocity_pid_tunning.hpp"
#include "utils/tools/common_types.hpp"
#include "tap/motor/dji_motor.hpp"

#ifdef FEEDER_COMPATIBLE
namespace src::Feeder {

// watchable variables
bool updateFeederVelocityConfigDebug = false;
float feederVelocityTunningFrequencyDebug = 0.0f;
float feederVelocityTunningAmplitudeDebug = 0.0f;

float currFeederTargetVelocityDisplay = 0.0f;

FeederVelocityTunningCommand::FeederVelocityTunningCommand(
    src::Drivers* drivers, 
    FeederSubsystem* feederSubsystem,
    FeederVelocityTunningConfig feederVelocityConfig)
    : drivers(drivers), 
      feeder(feederSubsystem), 
      feederVelocityConfig(feederVelocityConfig)
    {
        addSubsystemRequirement(dynamic_cast<tap::control::Subsystem*>(feeder));
    }

void FeederVelocityTunningCommand::execute() {
    currFeederTargetVelocityDisplay = getFeederTargetVelocity();
    feeder->ForAllFeederMotors(&FeederSubsystem::setTargetRPM, currFeederTargetVelocityDisplay);
}

float FeederVelocityTunningCommand::getFeederTargetVelocity() { // in degrees
    // For PID tunning through Ozone
    if (updateFeederVelocityConfigDebug) {
        feederVelocityConfig.frequencyHz = feederVelocityTunningFrequencyDebug;
        feederVelocityConfig.VelocityAmplitudeRPM = feederVelocityTunningAmplitudeDebug;
        updateFeederVelocityConfigDebug = false;
    }

    float periodMilliseconds = 1000.0f / feederVelocityConfig.frequencyHz;
    
    float timeInPeriod = fmod(getRelativeTime(), periodMilliseconds);
    
    if (timeInPeriod < periodMilliseconds / 2.0f) {
        return feederVelocityConfig.VelocityAmplitudeRPM;
    } else {
        return 0.0f;
    }
}

void FeederVelocityTunningCommand::initialize() { initTime = tap::arch::clock::getTimeMilliseconds(); }

bool FeederVelocityTunningCommand::isReady() { return true; }

bool FeederVelocityTunningCommand::isFinished() const { return false; }

void FeederVelocityTunningCommand::end(bool) {
    feeder->ForFeederMotorGroup(ALL, &FeederSubsystem::deactivateFeederMotor);
    feeder->setAllDesiredFeederMotorOutputs(0);
}

float FeederVelocityTunningCommand::getRelativeTime() const {
    return tap::arch::clock::getTimeMilliseconds() - initTime;
}

} // namespace src::Feeder
#endif // #ifdef FEEDER_COMPATIBLE
