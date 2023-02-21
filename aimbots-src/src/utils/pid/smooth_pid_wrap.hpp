#pragma once

#include <tap/algorithms/smooth_pid.hpp>

#include "utils/motion/settled_util.hpp"

namespace src::Utils {

struct SmoothPIDWrapper {
    float lastTime;
    float error;
    float prevError;
    float errorDerivative;

    src::Utils::motion::SettledUtil settledUtil;
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

    bool isSettled(float errTolerance) { return settledUtil.isSettled(this->error, errTolerance); }

    bool isSettled(float errTolerance, float derivTolerance, float derivToleranceTime) {
        return settledUtil.isSettled(this->error, errTolerance, this->errorDerivative, derivTolerance, derivToleranceTime);
    }

    float getError() { return this->error; }

    float getDerivative() { return this->errorDerivative; }

    float getOutput() { return pid.getOutput(); }
};

}  // namespace src::Utils