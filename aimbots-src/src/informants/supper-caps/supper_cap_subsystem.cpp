#include "supper_cap_subsystem.hpp"

namespace src::Informants::SupperCap {
    SupperCapSubsystem::SupperCapSubsystem(src::Drivers* drivers)
        : Subsystem(drivers),
          currentCommand(STOP),
          voltage(0),
          power(0),
          percent(0),
          inputPower(0)  
    {}


    void SupperCapSubsystem::initialize() {
    }

    void SupperCapSubsystem::refresh() {
        lastMessage = drivers->supperCapCommunicator.getLastValidMessage();
        voltage = lastMessage.voltage;
        power = lastMessage.power;
        percent = lastMessage.percent;
        inputPower = lastMessage.inputPower;

        switch (currentCommand) {
            case CHARGE:
                drivers->supperCapCommunicator.setCommand(CHARGE);
                switch(drivers->refSerial.getRobotData().robotLevel) {
                    case 1:
                        drivers->supperCapCommunicator.setChargeValue(1);
                        break;
                    case 2:
                        drivers->supperCapCommunicator.setChargeValue(2);
                        break;
                    case 3:
                        drivers->supperCapCommunicator.setChargeValue(3);
                        break;
                    default:
                        drivers->supperCapCommunicator.setChargeValue(0);
                        break;
                }
                break;
            case DISCHARGE:
                drivers->supperCapCommunicator.setCommand(DISCHARGE);
                break;
            case STOP:
                drivers->supperCapCommunicator.setCommand(STOP);
                break;
            default:
                break;
        }

    }
}