#include "clocking.hpp"

#include "tap/architecture/clock.hpp"
#include "tap/communication/gpio/pwm.hpp"

#include "drivers.hpp"

namespace utils::CV_Clocking {

    void startSignal(src::Drivers* drivers){
        drivers->pwm.setTimerFrequency(tap::gpio::Pwm::Timer::TIMER4, 5000);
        drivers->pwm.write(0.5,tap::gpio::Pwm::C1);
        drivers->pwm.start(tap::gpio::Pwm::Timer::TIMER4);
    };

}