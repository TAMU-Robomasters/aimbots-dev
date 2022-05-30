#pragma once

#include "tap/communication/gpio/pwm.hpp"

#include "utils/common_types.hpp"

namespace utils {

class UltrasonicDistanceSensor {
   public:
    UltrasonicDistanceSensor(uint16_t txPin, uint16_t rxPin)
        : txPin(txPin),
          rxPin(rxPin) {}

    void write(uint8_t data) {
    }

    uint8_t read();

   private:
    uint16_t txPin;
    uint16_t rxPin;
    const uint16_t MAX_DISTANCE {450};  // cm
};

}