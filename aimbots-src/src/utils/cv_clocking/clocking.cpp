#include "clocking.hpp"

#include "tap/architecture/clock.hpp"
#include "tap/communication/gpio/pwm.hpp"

#include "drivers.hpp"

namespace utils::CV_Clocking {

    CVClocker::CVClocker(src::Drivers* drivers) : drivers(drivers) {}

    void CVClocker::startSignal(){
        drivers->pwm.setTimerFrequency(tap::gpio::Pwm::Timer::TIMER2, 5000);
        drivers->pwm.write(0.5,tap::gpio::Pwm::PWM1);
        drivers->pwm.start(tap::gpio::Pwm::Timer::TIMER2);
    };

    void CVClocker::stopSignal(){
        drivers->pwm.writeAllZeros();
    };

}