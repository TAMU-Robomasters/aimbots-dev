#pragma once

// #include "tap/communication/gpio/pwm.hpp"
// 
#include "drivers.hpp"
#include "utils/common_types.hpp"
#include "tap/communication/gpio/digital.hpp" //maybe not
#include "tap/control/command.hpp"

namespace utils {

class LimitSwitch : public tap::control::Command  {
        public:
            LimitSwitch(src::Drivers* drivers, InputPins rxPin);
            // bool updateSwitch();
            bool readSwitch();

            bool isReady() override;

            void initialize() override;

            void execute() override;

            // void end(bool interrupted) override;

            bool isFinished() const override;
            
            bool updateSwitch();
        private:
            InputPins rxPin;
            src::Drivers* drivers;

            bool previousState;

            bool isStateChanged(bool currentState);

};

} // namespace utils