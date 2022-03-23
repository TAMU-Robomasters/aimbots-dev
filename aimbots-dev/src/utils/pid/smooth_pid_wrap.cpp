#include "smooth_pid_wrap.hpp"
#include "tap/algorithms/math_user_utils.hpp"

using namespace tap::algorithms;

namespace src::algorithms {

    void SmoothPIDWrap::initialize(float time) {
        this->lastTime = time;
    }

    float SmoothPIDWrap::runController(float error, float errorDerivative, float dt) {
        if (abs(error) < errDeadzone) {
            error = 0.0f;
        }

        // p
        currErrorP = kp * proportionalKalman.filterData(error);
        // i
        currErrorI = limitVal<float>(
            currErrorI + ki * proportionalKalman.getLastFiltered() * dt,
            -maxICumulative,
            maxICumulative);
        // d
        currErrorD = -kd * derivativeKalman.filterData(errorDerivative);
        // total
        output = limitVal<float>(currErrorP + currErrorI + currErrorD, -maxOutput, maxOutput);
        return output;
    }

    float SmoothPIDWrap::runControllerDerivateError(float error, float dt) {
        if (compareFloatClose(dt, 0.0f, 1E-5)) {
            dt = 1.0f;
        }
        float errorDerivative = (error - prevError) / dt;
        prevError = error;
        return runController(error, errorDerivative, dt);
    }

    float SmoothPIDWrap::getOutput() { return output; }

    void SmoothPIDWrap::reset() {
        this->output = 0.0f;
        this->currErrorP = 0.0f;
        this->currErrorI = 0.0f;
        this->currErrorD = 0.0f;
        this->prevError = 0.0f;
        this->derivativeKalman.reset();
        this->proportionalKalman.reset();
    }

}  // namespace src::algorithms