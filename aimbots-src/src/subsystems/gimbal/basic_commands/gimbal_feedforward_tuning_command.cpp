#include <cmath>
#include "gimbal_feedforward_tunning_command.hpp"
#include "utils/tools/common_types.hpp"

#ifdef GIMBAL_COMPATIBLE
namespace src::Gimbal {


GimbalPositionTunningCommand::GimbalPositionTunningCommand(
    src::Drivers* drivers, 
    GimbalSubsystem* gimbalSubsystem,
    GimbalFieldRelativeController* controller)
    : drivers(drivers), 
      gimbal(gimbalSubsystem), 
      controller(controller)
    {
        addSubsystemRequirement(dynamic_cast<tap::control::Subsystem*>(gimbal));
    }

void GimbalPositionTunningCommand::execute() {
 //   float yawTargetPosition = getYawTargetPosition();
    float pitchTargetPosition = getPitchTargetPosition();

//    currGimbalTargetYawPositionDisplay = yawTargetPosition;
    currGimbalTargetPitchPositionDisplay = pitchTargetPosition;

  //  controller->setTargetYaw(AngleUnit::Degrees, yawTargetPosition);
    controller->setTargetPitch(AngleUnit::Degrees, pitchTargetPosition);

 //   controller->runYawController(); 
    controller->runPitchController();
}

// float GimbalPositionTunningCommand::getYawTargetPosition() { // in degrees
//     // For PID tunning through Ozone
//     if (updateYawPositionConfigDebug) {
//         yawConfig.frequencyHz = yawPositionTunningFrequencyDebug;
//         yawConfig.positionAmplitudeDegrees = yawPositionTunningAmplitudeDebug;
//         updateYawPositionConfigDebug = false;
//     }

//     float periodMilliseconds = 1000.0f / yawConfig.frequencyHz;
    
//     float timeInPeriod = fmod(getRelativeTime(), periodMilliseconds);
    
//     if (timeInPeriod < periodMilliseconds / 2.0f) {
//         return yawConfig.positionAmplitudeDegrees;
//     } else {
//         return -yawConfig.positionAmplitudeDegrees;
//     }
// }
float holdTime; //Place holder

float GimbalPositionTunningCommand::getPitchTargetPosition() { // in degrees

    float periodMilliseconds = 1000.0f / pitchConfig.frequencyHz;
    
    float timeInPeriod = fmod(getRelativeTime(), periodMilliseconds);
    
    if (timeInPeriod < periodMilliseconds / 2.0f) {
        return pitchConfig.positionAmplitudeDegrees;
    } else {
        return -pitchConfig.positionAmplitudeDegrees;
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