#pragma once

#include <tap/algorithms/smooth_pid.hpp>

namespace src::utils {

struct SmoothPIDWrapper {
    float lastTime;
    tap::algorithms::SmoothPid pid;

    SmoothPIDWrapper(const tap::algorithms::SmoothPidConfig &config) : pid(config) {}

    float runController(float error, float derivativeInput) {
        float currTime = static_cast<float>(tap::arch::clock::getTimeMilliseconds());
        float dt = currTime - lastTime;
        lastTime = currTime;
        return pid.runController(error, derivativeInput, dt);
    }

    float runControllerDerivateError(float error) {
        float currTime = static_cast<float>(tap::arch::clock::getTimeMilliseconds());
        float dt = currTime - lastTime;
        lastTime = currTime;
        return pid.runControllerDerivateError(error, dt);
    }

    float getOutput() {
        return pid.getOutput();
    }
};

}  // namespace src::utils