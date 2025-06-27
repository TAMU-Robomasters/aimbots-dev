#include <cmath>
#include "gimbal_position_PID_tunning_command.hpp"
#include "utils/tools/common_types.hpp"

#ifdef GIMBAL_COMPATIBLE
namespace src::Gimbal {

// watchable variables
bool updateYawPositionConfigDebug = false;
float yawPositionTunningFrequencyDebug = 0.0f;
float yawPositionTunningAmplitudeDebug = 0.0f;

bool updateYawControllerVelocityLimitDebug = false;
float yawControllerVelocityLimitDebug = 0.0f; // degrees per second

GimbalPositionTunningCommand::GimbalPositionTunningCommand(
    src::Drivers* drivers, 
    GimbalSubsystem* gimbalSubsystem,
    GimbalFieldRelativeController* controller, 
    GimbalPositionTunningConfig config)
    : drivers(drivers), 
      gimbal(gimbalSubsystem), 
      controller(controller), 
      config(config)
    {
        addSubsystemRequirement(dynamic_cast<tap::control::Subsystem*>(gimbal));
    }

float currGimbalTargetYawPositionDisplay = 0.0f;

void GimbalPositionTunningCommand::execute() {
    float yawTargetPosition = getYawTargetPosition();

    currGimbalTargetYawPositionDisplay = yawTargetPosition;

    controller->setTargetYaw(AngleUnit::Degrees, yawTargetPosition);

    //TODO: fix this or get rid of it
    // for tuning velocity limit through Ozone
    if (updateYawControllerVelocityLimitDebug) {
        controller->runYawController(modm::toRadian(yawControllerVelocityLimitDebug));
        updateYawControllerVelocityLimitDebug = false;
    } else controller->runYawController(modm::toRadian(100)); 
}

float GimbalPositionTunningCommand::getYawTargetPosition() { // in degrees
    // For PID tunning through Ozone
    if (updateYawPositionConfigDebug) {
        config.yawFrequencyHz = yawPositionTunningFrequencyDebug;
        config.yawPositionAmplitudeDegrees = yawPositionTunningAmplitudeDebug;
        updateYawPositionConfigDebug = false;
    }

    float periodMilliseconds = 1000.0f / config.yawFrequencyHz;
    
    float timeInPeriod = fmod(getRelativeTime(), periodMilliseconds);
    
    if (timeInPeriod < periodMilliseconds / 2.0f) {
        return config.yawPositionAmplitudeDegrees;
    } else {
        return -config.yawPositionAmplitudeDegrees;
    }
}

void GimbalPositionTunningCommand::initialize() { initTime = tap::arch::clock::getTimeMilliseconds(); }

bool GimbalPositionTunningCommand::isReady() { return true; }

bool GimbalPositionTunningCommand::isFinished() const { return false; }

void GimbalPositionTunningCommand::end(bool) {
    gimbal->setAllDesiredYawMotorOutputs(0);
    gimbal->setAllDesiredPitchMotorOutputs(0);
}

float GimbalPositionTunningCommand::getRelativeTime() const {
    return tap::arch::clock::getTimeMilliseconds() - initTime;
}

} // namespace src::Gimbal
#endif // #ifdef GIMBAL_COMPATIBLE 