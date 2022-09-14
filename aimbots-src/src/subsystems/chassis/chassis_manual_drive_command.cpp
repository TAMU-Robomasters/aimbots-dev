#include "chassis_manual_drive_command.hpp"

#include <subsystems/chassis/chassis_rel_drive.hpp>

namespace src::Chassis {

ChassisManualDriveCommand::ChassisManualDriveCommand(src::Drivers* drivers, ChassisSubsystem* chassis)
    : drivers(drivers),
      chassis(chassis)  //
{
    addSubsystemRequirement(dynamic_cast<tap::control::Subsystem*>(chassis));
}

void ChassisManualDriveCommand::initialize() {}

void ChassisManualDriveCommand::execute() { Movement::Independent::onExecute(drivers, chassis); }

/**
    @brief set the chassis power to 0
*/
void ChassisManualDriveCommand::end(bool) { chassis->setTargetRPMs(0.0f, 0.0f, 0.0f); }

/**
    @brief detmerines if the functuon can be called by the scheduler

    @return true
*/
bool ChassisManualDriveCommand::isReady() { return true; }

/**
    @brief  runs the output command until scheduler interrupt is called

    @return false
*/
bool ChassisManualDriveCommand::isFinished() const { return false; }

}  // namespace src::Chassis