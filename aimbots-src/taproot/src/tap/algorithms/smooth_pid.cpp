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

#include "smooth_pid.hpp"

#include "tap/algorithms/math_user_utils.hpp"

using namespace tap::algorithms;

namespace tap
{
namespace algorithms
{
SmoothPid::SmoothPid(const SmoothPidConfig &pidConfig)
    : config(pidConfig),
      proportionalKalman(pidConfig.tQProportionalKalman, pidConfig.tRProportionalKalman),
      derivativeKalman(pidConfig.tQDerivativeKalman, pidConfig.tRDerivativeKalman)
{
}

float SmoothPid::runController(float error, float errorDerivative, float dt)
{
    if (abs(error) < config.errDeadzone)
    {
        error = 0.0f;
    }

    // p
    currErrorP = config.kp * proportionalKalman.filterData(error);
    // i
    currErrorI = limitVal<float>(
        currErrorI + config.ki * proportionalKalman.getLastFiltered() * dt,
        -config.maxICumulative,
        config.maxICumulative);
    // d
    currErrorD = -config.kd * derivativeKalman.filterData(errorDerivative);
    if (fabs(error) < config.errorDerivativeFloor)
    {
        // the error is less than some amount, so round derivative output to 0
        // done to avoid high frequency control oscilations in some systems
        currErrorD = 0.0f;
    }
    // total
    output =
        limitVal<float>(currErrorP + currErrorI + currErrorD, -config.maxOutput, config.maxOutput);
    return output;
}

float SmoothPid::runControllerDerivateError(float error, float dt)
{
    if (compareFloatClose(dt, 0.0f, 1E-5))
    {
        dt = 1.0f;
    }
    float errorDerivative = (error - prevError) / dt;
    prevError = error;
    return runController(error, errorDerivative, dt);
}

float SmoothPid::getOutput() { return output; }

void SmoothPid::reset()
{
    this->output = 0.0f;
    this->currErrorP = 0.0f;
    this->currErrorI = 0.0f;
    this->currErrorD = 0.0f;
    this->prevError = 0.0f;
    this->derivativeKalman.reset();
    this->proportionalKalman.reset();
}

}  // namespace algorithms

}  // namespace tap
