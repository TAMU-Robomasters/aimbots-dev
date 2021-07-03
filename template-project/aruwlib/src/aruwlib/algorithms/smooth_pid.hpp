/*
 * Copyright (c) 2020-2021 Advanced Robotics at the University of Washington <robomstr@uw.edu>
 *
 * This file is part of aruwlib.
 *
 * aruwlib is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * aruwlib is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with aruwlib.  If not, see <https://www.gnu.org/licenses/>.
 */

#ifndef SMOOTH_PID_HPP_
#define SMOOTH_PID_HPP_

#include <cstdint>

#include "aruwlib/algorithms/extended_kalman.hpp"

namespace aruwlib
{
namespace algorithms
{
class SmoothPid
{
public:
    SmoothPid(
        float kp,
        float ki,
        float kd,
        float maxICumulative,
        float maxOutput,
        float tQDerivativeKalman,
        float tRDerivativeKalman,
        float tQProportionalKalman,
        float tRProportionalKalman)
        : kp(kp),
          ki(ki),
          kd(kd),
          maxICumulative(maxICumulative),
          maxOutput(maxOutput),
          proportionalKalman(tQProportionalKalman, tRProportionalKalman),
          derivativeKalman(tQDerivativeKalman, tRDerivativeKalman)
    {
    }

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

    // while these could be local, debugging pid is much easier if they are not
    float currErrorP = 0.0f;
    float currErrorI = 0.0f;
    float currErrorD = 0.0f;
    float output = 0.0f;
    float prevError = 0.0f;

    aruwlib::algorithms::ExtendedKalman proportionalKalman;
    aruwlib::algorithms::ExtendedKalman derivativeKalman;
};

}  // namespace algorithms

}  // namespace aruwlib

#endif
