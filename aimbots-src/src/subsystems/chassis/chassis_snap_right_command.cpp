#include "subsystems/chassis/chassis_snap_right_command.hpp"

namespace src::Chassis {
    ChassisSnapRightCommand::ChassisSnapRightCommand(src::Drivers* drivers, ChassisSubsystem* chassis, src::Gimbal::GimbalSubsystem* gimbal)
        : drivers(drivers),
          chassis(chassis),
          gimbal(gimbal)
    {
        rotationDirection = 1;
        addSubsystemRequirement(dynamic_cast<tap::control::Subsystem*>(chassis));

    }

    void ChassisSnapRightCommand::initialize(){
        float initYaw = gimbal->getCurrentYawAngleFromChassisCenter(AngleUnit::Radians)+M_PI/2;
    }
    
    void ChassisSnapRightCommand::execute() {
        // uint32_t updateCounter = drivers->remote.getUpdateCounter();
        // uint32_t currTime = tap::arch::clock::getTimeMilliseconds();
        // uint32_t dt = lastRInputCallTime - currTime;
        // lastRInputCallTime = currTime;
        // // chassis->setTargetRPM(rotationSpeedRamp.getValue());
        // // gimbal->setTargetRotation(rotationDirection);
        // // if(drivers->remote.keyPressed(Remote::Key::E) ? 1.0f : 0.0f){
        //     // we are trying to speed up
        //     float error = initYaw - gimbal->getCurrentYawAngleFromChassisCenter(AngleUnit::Radians);
        //     float output = rotationSpeedRamp.runController(error, rotationSpeedRamp.runControllerDerivateError(error, dt), dt);
            rotationSpeedRamp.update(gimbal->getCurrentYawAngleFromChassisCenter(AngleUnit::Radians)-initYaw);
            rotationSpeedRamp.getValue();
            
            // }
    }
    
    void ChassisSnapRightCommand::end(bool interrupted) {
        chassis->setTargetRPMs(0.0f, 0.0f, 0.0f);
    }
    
    bool ChassisSnapRightCommand::isReady() {
        return true;
        
    }
    
    bool ChassisSnapRightCommand::isFinished() const {
        return false;
    }
}