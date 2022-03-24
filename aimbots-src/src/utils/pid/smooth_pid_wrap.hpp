#pragma once

#include <tap/algorithms/smooth_pid.hpp>

namespace src::utils {

struct SmoothPIDWrapper {
    uint32_t lastTime;
    tap::algorithms::SmoothPid pid;

    SmoothPIDWrapper(
        float kp,
        float ki,
        float kd,
        float maxICumulative,
        float maxOutput,
        float tQDerivativeKalman,
        float tRDerivativeKalman,
        float tQProportionalKalman,
        float tRProportionalKalman,
        float errDeadzone = 0.0f)
        : pid(kp,
              ki,
              kd,
              maxICumulative,
              maxOutput,
              tQDerivativeKalman,
              tRDerivativeKalman,
              tQProportionalKalman,
              tRProportionalKalman,
              errDeadzone) {}

    void runController(float error, float rotationalSpeed) {
        float time = static_cast<float>(tap::arch::clock::getTimeMilliseconds());
        float dt = time - lastTime;
        pid.runController(error, rotationalSpeed, dt);
        lastTime = time;
    }

    void runControllerDerivateError(float error) {
        float time = static_cast<float>(tap::arch::clock::getTimeMilliseconds());
        float dt = time - lastTime;
        pid.runControllerDerivateError(error, dt);
        lastTime = time;
    }
}

}  // namespace src::utils