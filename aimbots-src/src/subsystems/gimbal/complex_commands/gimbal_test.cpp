/*TODO: 
    - test how large the pitch and yaw amplitude can be*/
#include "gimbal_test.hpp"

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
    
}

void GimbalTestCommand::initialize() { commandStartTime = tap::arch::clock::getTimeMilliseconds(); }

bool GimbalTestCommand::isReady() { return true; }

bool GimbalTestCommand::isFinished() const { return false; }

void GimbalTestCommand::end(bool) {
    gimbal->setAllDesiredYawMotorOutputs(0);
    gimbal->setAllDesiredPitchMotorOutputs(0);
}

} // namespace src::Gimbal
