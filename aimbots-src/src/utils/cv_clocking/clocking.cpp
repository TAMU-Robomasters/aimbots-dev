#include "clocking.hpp"

#include "tap/architecture/clock.hpp"
#include "tap/communication/gpio/pwm.hpp"
#include "tap/communication/gpio/digital.hpp"

#include "drivers.hpp"


namespace utils::CV_Clocking {

    CVClocker::CVClocker(src::Drivers* drivers) : drivers(drivers) {}

  //  uint8_t resetDisplay = 0;
  bool bootUp = true;
  bool toggle = false;
  bool toggleFast = false;
  int increment = 0;
  int incrementFast = 0;
  bool displayFast = false;
  uint32_t lastTime = 0;
  uint32_t lastTimeFast = 0;
  float freq = 0.0f;
  float freqFast = 0.0f;

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
        this->ticks = 0;
    }

    void CVClocker::forceHigh(){
        drivers->pwm.setTimerFrequency(tap::gpio::Pwm::Timer::TIMER2, 10000);
        drivers->pwm.write(1.0,tap::gpio::Pwm::PWM1);
        drivers->pwm.start(tap::gpio::Pwm::Timer::TIMER2);
    }

    // void CVClocker::togglePWM(){
    //     // if(toggle){
    //     //     this->forceHigh();
    //     //     toggle = false;
    //     //     increment++;
    //     // }else{
    //     //     this->stopSignal();
    //     //     toggle = true;
    //     // }
    // }

    void CVClocker::togglePWM(bool fast){
        displayFast = fast;
        if(fast){
            if(toggleFast){
             //   this->forceHigh();
                toggleFast = false;
                incrementFast++;
                freqFast = 1/(float(tap::arch::clock::getTimeMicroseconds()-lastTimeFast))*1000000;
                lastTimeFast = tap::arch::clock::getTimeMicroseconds();
            }else{
           //     this->stopSignal();
                toggleFast = true;
            }
        }else{
            if(toggle){
         //       this->forceHigh();
                toggle = false;
                increment++;
                freq = 1/(float(tap::arch::clock::getTimeMicroseconds()-lastTime))*1000000;
                lastTime = tap::arch::clock::getTimeMicroseconds();
            }else{
           //     this->stopSignal();
                toggle = true;
            }
        }
    }

    void CVClocker::toogleClock(){
        if(resetTimer.isExpired()){
            if(toggle){
                toggle = false;
                this->ticks++;
                drivers->digital.set(tap::gpio::Digital::PE13, true);
            }else{
                toggle = true;
                drivers->digital.set(tap::gpio::Digital::PE13, false);
            }
        }else if(bootUp){
            if(toggle){
                toggle = false;
                this->ticks++;
                drivers->digital.set(tap::gpio::Digital::PE13, true);
            }else{
                toggle = true;
                drivers->digital.set(tap::gpio::Digital::PE13, false);
            }
            resetTimer.restart(1);
        }
    }

    uint32_t CVClocker::getTicks(){
        return this->ticks;
    }

}