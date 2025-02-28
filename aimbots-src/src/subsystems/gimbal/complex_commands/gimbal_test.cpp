/*TODO: 
    - test how large the pitch and yaw amplitude can be*/
#include <cmath>
#include "gimbal_test.hpp"
#include "utils/tools/common_types.hpp"

namespace src::Gimbal{
GimbalTestCommand::GimbalTestCommand(
        src::Drivers* drivers, 
        GimbalSubsystem* gimbalSubsystem, 
        GimbalFieldRelativeController* controller, 
        GimbalTestConfig config)
        : drivers(drivers), 
          gimbal(gimbalSubsystem), 
          controller(controller), 
          config(config)
        {
            // ? what does this actually do
            addSubsystemRequirement(dynamic_cast<tap::control::Subsystem*>(gimbal));
        }  

// Display variables
float currGimbalTestTargetYawAngleDisplay = 0.0f;
float currGimbalTestTargetPitchAngleDisplay = 0.0f;

void GimbalTestCommand::execute() {
    float yawTargetAngle = getYawTargetAngle();
    float pitchTargetAngle = getPitchTargetAngle();

    currGimbalTestTargetYawAngleDisplay = modm::toDegree(yawTargetAngle);
    currGimbalTestTargetPitchAngleDisplay = modm::toDegree(pitchTargetAngle); 

    controller->setTargetYaw(AngleUnit::Radians, yawTargetAngle);
    controller->setTargetPitch(AngleUnit::Radians, pitchTargetAngle);

    controller->runYawController();
    controller->runPitchController();
}

float GimbalTestCommand::getYawTargetAngle() {
    float yawTargetAngle = modm::toRadian(config.yawAmplitudeDegree) * cos(2*M_PI*getRelativeTime()/1000) 
                         + modm::toRadian(config.yawOffsetDegree);
    return yawTargetAngle;
}

float GimbalTestCommand::getPitchTargetAngle() {
    float yawTargetAngle = modm::toRadian(config.pitchAmplitudeDegree) * sin(2*M_PI*getRelativeTime()/1000) 
                         + modm::toRadian(config.pitchOffsetDegree);
    return yawTargetAngle;
}

void GimbalTestCommand::initialize() { initTime = tap::arch::clock::getTimeMilliseconds(); }

bool GimbalTestCommand::isReady() { return true; }

bool GimbalTestCommand::isFinished() const { return false; }

void GimbalTestCommand::end(bool) {
    gimbal->setAllDesiredYawMotorOutputs(0);
    gimbal->setAllDesiredPitchMotorOutputs(0);
}

} // namespace src::Gimbal
