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

#include "ramp.hpp"

#include <cmath>

#include "math_user_utils.hpp"

namespace tap
{
namespace algorithms
{
Ramp::Ramp(float initialValue) : target(initialValue), value(initialValue), targetReached(true) {}

void Ramp::reset(float val)
{
    target = val;
    value = val;
    targetReached = true;
}

void Ramp::setTarget(float target)
{
    if (!compareFloatClose(target, this->target, RAMP_EPSILON))
    {
        this->target = target;
        this->targetReached = false;
    }
}

void Ramp::setValue(float value) { this->value = value; }

void Ramp::update(float increment)
{
    increment = copysign(increment, target - value);
    float targetValueDifference = copysign(target - value, increment);
    value = fabs(targetValueDifference) > fabs(increment) ? value + increment : target;
    targetReached = compareFloatClose(value, target, RAMP_EPSILON);
}

float Ramp::getValue() const { return this->value; }

bool Ramp::isTargetReached() const { return targetReached; }

float Ramp::getTarget() const { return target; }

}  // namespace algorithms

}  // namespace tap
