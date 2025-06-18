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

#ifndef TAPROOT_M3508_CONSTANTS_HPP_
#define TAPROOT_M3508_CONSTANTS_HPP_

#include "motor_constants.hpp"

namespace tap::motor
{
class M3508Constants : public MotorConstants
{
public:
    /**
     * According to the C620 documentation, the current output range is [-16384, 16384] scaled to
     * [-20, 20] amps.
     */
    inline float convertOutputToCurrent(float output) const override
    {
        return output * 20000.0f / 16384.0f;
    }

    inline float getTorqueConstant() const override { return 0.3; }
};
}  // namespace tap::motor

#endif  // TAPROOT_M3508_CONSTANTS_HPP_
