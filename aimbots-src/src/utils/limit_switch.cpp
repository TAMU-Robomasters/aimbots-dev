#include "utils/limit_switch.hpp"

namespace utils {
    LimitSwitch::LimitSwitch(src::Drivers* drivers, InputPins rxPin) 
        :   drivers(drivers),
            rxPin(rxPin) {}


        bool LimitSwitch::readSwitch() {
            return drivers->digital.read(rxPin);
        }

        void LimitSwitch::updateSwitch() {
            prevSwitchState = currSwitchState;

            currSwitchState = static_cast<LimitSwitchState>(readSwitch());
        }

        bool LimitSwitch::isRising() const{
            return (currSwitchState == LimitSwitchState::PRESSED) && (prevSwitchState == LimitSwitchState::RELEASED);
        }

        bool LimitSwitch::isFalling() const{
            return (currSwitchState == LimitSwitchState::RELEASED) && (prevSwitchState == LimitSwitchState::PRESSED);
        }
}   // namespace utils