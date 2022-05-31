#pragma once

#include <tap/algorithms/smooth_pid.hpp>

namespace src::utils {

struct SmoothPIDWrapper {
    float lastTime;
    float error;
    tap::algorithms::SmoothPid pid;

    SmoothPIDWrapper(const tap::algorithms::SmoothPidConfig &config) : pid(config) {}

    float runController(float error, float derivativeInput) {
        this->error = error;
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
        return pid.runControllerDerivateError(error, dt);
    }

    float isSettled(float errTolerance) {
        return static_cast<float>(fabs(error)) < errTolerance;
    }

    float isSettled(float errTolerance, float /*derivTolerance*/, float /*derivToleranceTime*/) {
        return static_cast<float>(fabs(error)) < errTolerance;
    }

    float getError() {
        return this->error;
    }

    float getOutput() {
        return pid.getOutput();
    }
};

}  // namespace src::utils