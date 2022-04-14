#include "utils/limit_switch.hpp"

namespace utils {
    LimitSwitch::LimitSwitch(InputPins rxPin) {}


        bool LimitSwitch::readSwitch() {
            return tap::gpio::Digital::read(rxPin);
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

        bool LimitSwitch::isStateChanged(bool currentState) {
            if (currentState != previousState) {
                return true;
            }
            return false;
        }
}   // namespace utils