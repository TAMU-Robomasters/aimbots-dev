#include "clocking.hpp"

#include "tap/architecture/clock.hpp"
#include "tap/communication/gpio/pwm.hpp"

#include "drivers.hpp"


namespace utils::CV_Clocking {

    CVClocker::CVClocker(src::Drivers* drivers) : drivers(drivers) {}

  //  uint8_t resetDisplay = 0;

    void CVClocker::startSignal(){
        if(resetTimer.isExpired()){
            drivers->pwm.setTimerFrequency(tap::gpio::Pwm::Timer::TIMER2, 5000);
            drivers->pwm.write(0.5,tap::gpio::Pwm::PWM1);
            drivers->pwm.start(tap::gpio::Pwm::Timer::TIMER2);
      //      resetDisplay = 1;
        }//else{resetDisplay = 0;}
    };

    void CVClocker::stopSignal(){
        drivers->pwm.writeAllZeros();
    };

    void CVClocker::reset(){
        this->stopSignal();
        resetTimer.restart(1000);
    }

}