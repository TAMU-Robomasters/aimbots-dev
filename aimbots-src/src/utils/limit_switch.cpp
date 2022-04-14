#include "utils/limit_switch.hpp"

namespace utils {
    LimitSwitch::LimitSwitch(src::Drivers* drivers, InputPins rxPin) 
        : tap::control::Command(),
            drivers(drivers),
            rxPin(rxPin) {}


        bool LimitSwitch::readSwitch() {
            return drivers->digital.read(rxPin);
        }

        bool LimitSwitch::updateSwitch(){
            bool state = readSwitch();

            if (isStateChanged(state)) {
                previousState = !previousState;
                if (state) {
                    return true;
                }
            }
            return false;
        }

        void LimitSwitch::initialize() {
            return;
        }

        bool LimitSwitch::isReady() {
            return true;
        }

        bool LimitSwitch::isFinished() const {
            return false;
        }

        void LimitSwitch::execute() {
        }


        bool LimitSwitch::isStateChanged(bool currentState) {
            if (currentState != previousState) {
                return true;
            }
            return false;
        }
}   // namespace utils