#include "stop_feeder_command.hpp"

#ifdef FEEDER_COMPATIBLE

namespace src::Feeder {

StopFeederCommand::StopFeederCommand(src::Drivers* drivers, FeederSubsystem* feeder) : drivers(drivers), feeder(feeder) {
    addSubsystemRequirement(dynamic_cast<tap::control::Subsystem*>(feeder));
}

void StopFeederCommand::initialize() { feeder->ForFeederMotorGroup(ALL, &FeederSubsystem::deactivateFeederMotor); }

void StopFeederCommand::execute() { feeder->ForFeederMotorGroup(ALL, &FeederSubsystem::deactivateFeederMotor); }

void StopFeederCommand::end(bool interrupted) { UNUSED(interrupted); }

bool StopFeederCommand::isReady() { return true; }

bool StopFeederCommand::isFinished() const { return false; }

}  // namespace src::Feeder

#endif  // #ifdef FEEDER_COMPATIBLE