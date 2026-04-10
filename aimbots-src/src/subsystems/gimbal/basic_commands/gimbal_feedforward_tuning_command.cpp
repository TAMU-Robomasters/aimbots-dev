#include <cmath>
#include "gimbal_feedforward_tunning_command.hpp"
#include "utils/tools/common_types.hpp"

#ifdef GIMBAL_COMPATIBLE
namespace src::Gimbal {


GimbalFeedforwardTunningCommand::GimbalFeedforwardTunningCommand(
    src::Drivers* drivers, 
    GimbalSubsystem* gimbalSubsystem,
    GimbalFieldRelativeController* controller, 
    GimbalFeedForwardConfig& ffconfig)
    : drivers(drivers), 
      gimbal(gimbalSubsystem), 
      controller(controller),
      ffConfig(ffConfig)
    {
        addSubsystemRequirement(dynamic_cast<tap::control::Subsystem*>(gimbal));
    }

float holdTime; //Place holder
int numPoints
float currGimbalTargetPitchPositionDisplay = 0.0f;

void GimbalFeedforwardTunningCommand::execute() {
    float pitchTargetPosition = 0.0f;
    floa
    if(getRelativeTime < (holdTime * numTimes)){
        pitchTargetPosition = pitchTargets[]//todo, figure out which target depending on time
    }

    currGimbalTargetPitchPositionDisplay = pitchTargetPosition;
    controller->setTargetPitch(AngleUnit::Degrees, pitchTargetPosition);
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

vector<float> GimbalFeedforwardTunningCommand::getPitchTargets() { // in degrees
    vector<float,numPoints> positionTargets;
    float angleDiff = (PITCH_AXIS_SOFTSTOP_HIGH - PITCH_AXIS_SOFTSTOP_LOW)/(numPoints-1);

    for(int i=0;i<numPoints;i++){
        positionTargets[i] = PITCH_AXIS_SOFTSTOP_LOW+(angleDiff*i);
    }
    return positionTargets;    
}

void GimbalFeedforwardTunningCommand::initialize() { 
    initTime = tap::arch::clock::getTimeMilliseconds(); 
    pitchTargets = getPitchTargetPosition();
}

bool GimbalFeedforwardTunningCommand::isReady() { return true; }

bool GimbalFeedforwardTunningCommand::isFinished() const { return false; }

void GimbalFeedforwardTunningCommand::end(bool) {
    gimbal->setAllDesiredYawMotorOutputs(0);
    gimbal->setAllDesiredPitchMotorOutputs(0);
}

float GimbalFeedforwardTunningCommand::getRelativeTime() const {
    return tap::arch::clock::getTimeMilliseconds() - initTime;
}

} // namespace src::Gimbal
#endif // #ifdef GIMBAL_COMPATIBLE 