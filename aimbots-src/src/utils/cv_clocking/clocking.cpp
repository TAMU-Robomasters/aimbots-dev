#include "clocking.hpp"

#include "tap/architecture/clock.hpp"
#include "tap/communication/gpio/pwm.hpp"

#include "drivers.hpp"


namespace utils::CV_Clocking {

    CVClocker::CVClocker(src::Drivers* drivers) : drivers(drivers) {}

  //  uint8_t resetDisplay = 0;
  bool bootUp = true;
  bool toggle = false;
  long long increment = 0;

    void CVClocker::startSignal(){
        if(resetTimer.isExpired()){
            drivers->pwm.write(0.5,tap::gpio::Pwm::PWM1);
            drivers->pwm.start(tap::gpio::Pwm::Timer::TIMER2);
        } else if(bootUp){
            drivers->pwm.setTimerFrequency(tap::gpio::Pwm::Timer::TIMER2, 10000);
            drivers->pwm.write(0.5,tap::gpio::Pwm::PWM1);
            drivers->pwm.start(tap::gpio::Pwm::Timer::TIMER2);
            resetTimer.restart(1);
        }
    };

    void CVClocker::stopSignal(){
        drivers->pwm.writeAllZeros();
    };

    void CVClocker::reset(){
        this->stopSignal();
        if(resetTimer.isExpired()){
            resetTimer.restart(1000);
        }
        bootUp = false;
    }

    void CVClocker::forceHigh(){
        drivers->pwm.setTimerFrequency(tap::gpio::Pwm::Timer::TIMER2, 10000);
        drivers->pwm.write(1.0,tap::gpio::Pwm::PWM1);
        drivers->pwm.start(tap::gpio::Pwm::Timer::TIMER2);
    }

    void CVClocker::togglePWM(){
        if(toggle){
            this->forceHigh();
            toggle = false;
            increment++;
        }else{
            this->stopSignal();
            toggle = true;
        }
    }

}