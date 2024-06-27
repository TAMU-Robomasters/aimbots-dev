#include "chassis_idle_command.hpp"

#include "subsystems/chassis/chassis_helper.hpp"

#ifdef CHASSIS_COMPATIBLE

namespace src::Chassis {

ChassisIdleCommand::ChassisIdleCommand(src::Drivers* drivers, ChassisSubsystem* chassis)
    : drivers(drivers),
      chassis(chassis)  //
{
    addSubsystemRequirement(dynamic_cast<tap::control::Subsystem*>(chassis));
}

void ChassisIdleCommand::initialize() {}

void ChassisIdleCommand::execute() { chassis->setTargetRPMs(0.0f, 0.0f, 0.0f); }

void ChassisIdleCommand::end(bool) {}

bool ChassisIdleCommand::isReady() { return true; }

bool ChassisIdleCommand::isFinished() const { return false; }

}  // namespace src::Chassis

#endif  //#ifdef CHASSIS_COMPATIBLE