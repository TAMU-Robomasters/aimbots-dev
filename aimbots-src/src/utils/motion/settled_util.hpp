#pragma once

#include "tap/architecture/timeout.hpp"

namespace src::utils {

    class SettledUtil {
       private:
        tap::arch::MilliTimeout errorTimeout;
        tap::arch::MilliTimeout derivativeTimeout;

       public:
        static bool isSettled(float error, float errorTolerance) {
            return static_cast<float>(fabs(error)) < errorTolerance;
        }

        bool isSettled(float error, float errTolerance, float derivative, float derivTolerance, float derivToleranceTime) {
            if (static_cast<float>(fabs(error)) < errTolerance) {
                if (static_cast<float>(fabs(derivative)) < derivTolerance) {
                    // if timeout has expired and it's still running, then it's settled
                    if (derivativeTimeout.isExpired()) {
                        return true;
                    } else if (derivativeTimeout.isStopped()) {
                        // otherwise if the timeout has been stopped, then it must've previously left the settled range and needs restarting
                        derivativeTimeout.restart(derivToleranceTime);
                    }
                } else {
                    // if the derivative is out of the tolerance range, then stop the timeout
                    derivativeTimeout.stop();
                }
            }
            return false;
        }
    };
}