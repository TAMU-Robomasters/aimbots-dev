#pragma once

#include <tap/algorithms/smooth_pid.hpp>

namespace src::utils {

struct SmoothPIDWrapper {
    float lastTime;
    float error;
    float prevError;
    float errorDerivative;

    tap::arch::MilliTimeout errorTimeout;
    tap::arch::MilliTimeout derivativeTimeout;

    tap::algorithms::SmoothPid pid;

    SmoothPIDWrapper(const tap::algorithms::SmoothPidConfig &config) : pid(config) {}

    float runController(float error, float derivativeInput) {
        this->error = error;
        this->errorDerivative = derivativeInput;
        float currTime = static_cast<float>(tap::arch::clock::getTimeMilliseconds());
        float dt = currTime - lastTime;
        lastTime = currTime;
        return pid.runController(error, derivativeInput, dt);
    }

    float runControllerDerivateError(float error) {
        this->error = error;
        float currTime = static_cast<float>(tap::arch::clock::getTimeMilliseconds());
        float dt = currTime - lastTime;
        lastTime = currTime;

        errorDerivative = (error - prevError) / dt;

        return pid.runControllerDerivateError(error, dt);
    }

    float isSettled(float errTolerance) {
        return static_cast<float>(fabs(error)) < errTolerance;
    }

    float isSettled(float errTolerance, float derivTolerance, float derivToleranceTime) {
        if (static_cast<float>(fabs(error)) < errTolerance) {
            if (static_cast<float>(fabs(errorDerivative)) < derivTolerance) {
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

    float getError() {
        return this->error;
    }

    float getDerivative() {
        return this->errorDerivative;
    }

    float getOutput() {
        return pid.getOutput();
    }
};

}  // namespace src::utils