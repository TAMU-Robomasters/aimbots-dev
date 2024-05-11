#include "supper_cap_subsystem.hpp"

namespace src::Informants::SupperCap {
SupperCapSubsystem::SupperCapSubsystem(src::Drivers* drivers)
    : Subsystem(drivers),
      currentCommand(STOP),
      voltage(0),
      power(0),
      percent(0),
      inputPower(0) {}

void SupperCapSubsystem::initialize() { drivers->supperCapCommunicator.setCommand(CHARGE); }

float lastVoltage = 0;
float lastPower = 0;
float lastPercent = 0;
float lastInputPower = 0;

char lastCommand = ' ';

void SupperCapSubsystem::refresh() {
    lastMessage = drivers->supperCapCommunicator.getLastValidMessage();
    voltage = lastMessage.voltage;
    power = lastMessage.power;
    percent = lastMessage.percent;
    inputPower = lastMessage.inputPower;

    lastVoltage = voltage;
    lastPower = power;
    lastPercent = percent;
    lastInputPower = inputPower;
    lastCommand = currentCommand;

    switch (currentCommand) {
        case CHARGE:
            if (percent > 95) {
                // drivers->supperCapCommunicator.setCommand(STOP);
                // currentCommand = STOP;
                currentCommand = currentCommand;
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
            currentCommand = CHARGE;
            drivers->supperCapCommunicator.setCommand(CHARGE);
            break;
        default:
            drivers->supperCapCommunicator.setCommand(CHARGE);
            currentCommand = CHARGE;
            break;
    }
}
}  // namespace src::Informants::SupperCap