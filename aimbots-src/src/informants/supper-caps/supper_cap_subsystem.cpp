#include "supper_cap_subsystem.hpp"

namespace src::Informants::SupperCap {
SupperCapSubsystem::SupperCapSubsystem(src::Drivers* drivers)
    : Subsystem(drivers),
      currentCommand(STOP),
      voltage(0),
      power(0),
      percent(0),
      inputPower(0) {}

void SupperCapSubsystem::initialize() {
    drivers->supperCapCommunicator.setCommand(CHARGE);
}

void SupperCapSubsystem::refresh() {
    lastMessage = drivers->supperCapCommunicator.getLastValidMessage();
    voltage = lastMessage.voltage;
    power = lastMessage.power;
    percent = lastMessage.percent;
    inputPower = lastMessage.inputPower;

    switch (currentCommand) {
        case CHARGE:
            if (percent > 95) {
                drivers->supperCapCommunicator.setCommand(STOP);
                currentCommand = STOP;
            } else {
                drivers->supperCapCommunicator.setCommand(CHARGE);
            }
            // todo set charge amount. tbh idk where its coming from but it is i guess

            break;
        case DISCHARGE:
            if (percent < 28) {
                drivers->supperCapCommunicator.setCommand(CHARGE);
                currentCommand = CHARGE;
            } else {
                drivers->supperCapCommunicator.setCommand(DISCHARGE);
            }
            break;
        case STOP:
            drivers->supperCapCommunicator.setCommand(STOP);
            break;
        default:
            break;
    }
}
}  // namespace src::Informants::SupperCap