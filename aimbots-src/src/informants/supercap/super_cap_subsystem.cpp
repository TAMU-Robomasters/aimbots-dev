#include "super_cap_subsystem.hpp"

namespace src::Informants::SuperCap {
SuperCapSubsystem::SuperCapSubsystem(src::Drivers* drivers)
    : Subsystem(drivers),
      currentCommand(STOP),
      voltage(0),
      power(0),
      percent(0),
      inputPower(0) {}

void SuperCapSubsystem::initialize() { drivers->superCapCommunicator.setCommand(CHARGE); }

float lastVoltage = 0;
float lastPower = 0;
float lastPercent = 0;
float lastInputPower = 0;

char lastCommand = ' ';

void SuperCapSubsystem::refresh() {
    lastMessage = drivers->superCapCommunicator.getLastValidMessage();
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
                // drivers->superCapCommunicator.setCommand(STOP);
                // currentCommand = STOP;
                currentCommand = currentCommand;
            } else {
                drivers->superCapCommunicator.setCommand(CHARGE);
            }
            // todo set charge amount. tbh idk where its coming from but it is i guess

            break;
        case DISCHARGE:
            if (percent < 28) {
                drivers->superCapCommunicator.setCommand(CHARGE);
                currentCommand = CHARGE;
            } else {
                drivers->superCapCommunicator.setCommand(DISCHARGE);
            }
            break;
        case STOP:
            currentCommand = CHARGE;
            drivers->superCapCommunicator.setCommand(CHARGE);
            break;
        default:
            drivers->superCapCommunicator.setCommand(CHARGE);
            currentCommand = CHARGE;
            break;
    }
}
}  // namespace src::Informants::SuperCap