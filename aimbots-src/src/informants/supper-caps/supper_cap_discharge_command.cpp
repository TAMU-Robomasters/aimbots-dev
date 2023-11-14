#include "supper_cap_discharge_command.hpp"

namespace src::Informants::SupperCap {

SupperCapDischargeCommand::SupperCapDischargeCommand(src::Drivers* drivers, SupperCapSubsystem* supperCap)
    : drivers(drivers),
      supperCap(supperCap) {
    addSubsystemRequirement(dynamic_cast<tap::control::Subsystem*>(supperCap));
}

void SupperCapDischargeCommand::initialize() {}

void SupperCapDischargeCommand::execute() {
    if (supperCap->getLastMessage().percent < 28) {
        supperCap->setCommand(STOP);
    } else {
        supperCap->setCommand(DISCHARGE);
    }
}

void SupperCapDischargeCommand::end(bool interrupted) {}

bool SupperCapDischargeCommand::isReady() { return true; }

bool SupperCapDischargeCommand::isFinished() const { return false; }

}  // namespace src::Informants::SupperCap
