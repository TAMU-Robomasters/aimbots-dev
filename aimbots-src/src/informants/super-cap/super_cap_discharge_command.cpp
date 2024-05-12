#include "super_cap_discharge_command.hpp"

namespace src::Informants::SuperCap {

SuperCapDischargeCommand::SuperCapDischargeCommand(src::Drivers* drivers, SuperCapSubsystem* superCap)
    : drivers(drivers),
      superCap(superCap) {
    addSubsystemRequirement(dynamic_cast<tap::control::Subsystem*>(superCap));
}

void SuperCapDischargeCommand::initialize() {}

float prec = 0;
void SuperCapDischargeCommand::execute() {
    prec = superCap->getLastMessage().percent;
    if (superCap->getLastMessage().percent < 28) {
        superCap->setCommand(STOP);
    } else {
        superCap->setCommand(DISCHARGE);
    }
}

void SuperCapDischargeCommand::end(bool interrupted) {}

bool SuperCapDischargeCommand::isReady() { return true; }

bool SuperCapDischargeCommand::isFinished() const { return false; }

}  // namespace src::Informants::SuperCap
