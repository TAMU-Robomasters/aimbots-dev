/*
 * Copyright (c) 2020-2021 Advanced Robotics at the University of Washington <robomstr@uw.edu>
 *
 * This file is part of Taproot.
 *
 * Taproot is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Taproot is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Taproot.  If not, see <https://www.gnu.org/licenses/>.
 */

#ifndef TAPROOT_SMOOTH_PID_HPP_
#define TAPROOT_SMOOTH_PID_HPP_

#include <cstdint>

#include "tap/algorithms/extended_kalman.hpp"

namespace tap
{
namespace algorithms
{
struct SmoothPidConfig
{
    float kp = 0.0f;
    float ki = 0.0f;
    float kd = 0.0f;
    float maxICumulative = 0.0f;
    float maxOutput = 0.0f;
    float tQDerivativeKalman = 1.0f;   /**< The system noise covariance for the kalman filter that
                                        * is applied to the derivative of the error. */
    float tRDerivativeKalman = 0.0f;   /**< The measurement noise covariance for the kalman filter
                                        * that is applied to the derivative of the error. */
    float tQProportionalKalman = 1.0f; /**< The system noise covariance for the kalman filter that
                                        *  is applied to the proportional error. */
    float tRProportionalKalman = 0.0f; /**< The measurement noise covariance for the kalman filter
                                        * that is applied to the proportional error. */
    float errDeadzone = 0.0f;          /**< Within [-errDeadzone, errDeadzone], the PID controller
                                        * error will be set to 0. */
    float errorDerivativeFloor = 0.0f; /**< Minimum error value at which the PID controller will
                                        * compute and apply the derivative term. */
};

class SmoothPid
{
public:
    SmoothPid(const SmoothPidConfig &pidConfig);

    /**
     * Runs the PID controller. Should be called frequently for best results.
     *
     * @param[in] error The error (in user-defined units) between some target and measured value.
     * @param[in] errorDerivative The derivative of the error passed in above (in user-defined units
     * / time).
     * @param[in] dt The difference in time between the time this function is being called and the
     * last time this function was called.
     */
    virtual float runController(float error, float errorDerivative, float dt);

    float runControllerDerivateError(float error, float dt);

    float getOutput();

    void reset();

    inline void setP(float p) { config.kp = p; }
    inline void setI(float i) { config.ki = i; }
    inline void setD(float d) { config.kd = d; }
    inline void setMaxICumulative(float maxICumulative) { config.maxICumulative = maxICumulative; }
    inline void setMaxOutput(float maxOutput) { config.maxOutput = maxOutput; }
    inline void setErrDeadzone(float errDeadzone) { config.errDeadzone = errDeadzone; }

private:
    // gains and constants, to be set by the user
    SmoothPidConfig config;

    // while these could be local, debugging pid is much easier if they are not
    float currErrorP = 0.0f;
    float currErrorI = 0.0f;
    float currErrorD = 0.0f;
    float output = 0.0f;
    float prevError = 0.0f;

    tap::algorithms::ExtendedKalman proportionalKalman;
    tap::algorithms::ExtendedKalman derivativeKalman;
};

}  // namespace algorithms

}  // namespace tap

#endif  // TAPROOT_SMOOTH_PID_HPP_
