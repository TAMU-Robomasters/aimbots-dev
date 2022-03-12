#pragma once

#include <tap/algorithms/smooth_pid.hpp>

namespace src::algorithms {

    class SmoothPIDWrap {
       public:
        SmoothPIDWrap(
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
            : kp(kp),
              ki(ki),
              kd(kd),
              maxICumulative(maxICumulative),
              maxOutput(maxOutput),
              errDeadzone(errDeadzone),
              proportionalKalman(tQProportionalKalman, tRProportionalKalman),
              derivativeKalman(tQDerivativeKalman, tRDerivativeKalman) {
        }

        void initialize(float time);

        float runController(float error, float rotationalSpeed, float dt);

        float runControllerDerivateError(float error, float dt);

        float getOutput();

        void reset();

        inline void setP(float p) { kp = p; }
        inline void setI(float i) { ki = i; }
        inline void setD(float d) { kd = d; }

       private:
        // gains and constants, to be set by the user
        float kp = 0.0f;
        float ki = 0.0f;
        float kd = 0.0f;
        float maxICumulative = 0.0f;
        float maxOutput = 0.0f;
        float errDeadzone = 0.0f;

        // while these could be local, debugging pid is much easier if they are not
        float currErrorP = 0.0f;
        float currErrorI = 0.0f;
        float currErrorD = 0.0f;
        float output = 0.0f;
        float prevError = 0.0f;

        float lastTime = 0.0f;
        float dT = 0.0f;

        tap::algorithms::ExtendedKalman proportionalKalman;
        tap::algorithms::ExtendedKalman derivativeKalman;
    };
}  // namespace src::algorithms