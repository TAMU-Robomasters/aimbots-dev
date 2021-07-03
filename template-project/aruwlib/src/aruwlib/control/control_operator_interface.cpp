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

#include "control_operator_interface.hpp"

#include "aruwlib/algorithms/math_user_utils.hpp"
#include "aruwlib/architecture/clock.hpp"
#include "aruwlib/drivers.hpp"

using namespace aruwlib;
using namespace aruwlib::algorithms;

namespace aruwlib
{
namespace control
{
float ControlOperatorInterface::getChassisXInput()
{
    uint32_t updateCounter = drivers->remote.getUpdateCounter();
    uint32_t currTime = aruwlib::arch::clock::getTimeMilliseconds();
    if (prevUpdateCounterX != updateCounter)
    {
        chassisXInput.update(drivers->remote.getChannel(Remote::Channel::LEFT_VERTICAL), currTime);
        prevUpdateCounterX = updateCounter;
    }

    chassisXKeyInputFiltered = algorithms::lowPassFilter(
        chassisXKeyInputFiltered,
        drivers->remote.keyPressed(Remote::Key::W) - drivers->remote.keyPressed(Remote::Key::S),
        CHASSIS_X_KEY_INPUT_FILTER_ALPHA);

    return limitVal<float>(
        chassisXInput.getInterpolatedValue(currTime) + chassisXKeyInputFiltered,
        -1.0f,
        1.0f);
}

float ControlOperatorInterface::getChassisYInput()
{
    uint32_t updateCounter = drivers->remote.getUpdateCounter();
    uint32_t currTime = aruwlib::arch::clock::getTimeMilliseconds();
    if (prevUpdateCounterY != updateCounter)
    {
        chassisYInput.update(
            drivers->remote.getChannel(Remote::Channel::LEFT_HORIZONTAL),
            currTime);
        prevUpdateCounterY = updateCounter;
    }

    chassisYKeyInputFiltered = algorithms::lowPassFilter(
        chassisYKeyInputFiltered,
        drivers->remote.keyPressed(Remote::Key::D) - drivers->remote.keyPressed(Remote::Key::A),
        CHASSIS_Y_KEY_INPUT_FILTER_ALPHA);

    return limitVal<float>(
        chassisYInput.getInterpolatedValue(currTime) + chassisYKeyInputFiltered,
        -1.0f,
        1.0f);
}

float ControlOperatorInterface::getChassisRInput()
{
    uint32_t currTime = aruwlib::arch::clock::getTimeMilliseconds();
    uint32_t updateCounter = drivers->remote.getUpdateCounter();
    if (prevUpdateCounterR != updateCounter)
    {
        chassisRInput.update(
            drivers->remote.getChannel(Remote::Channel::RIGHT_HORIZONTAL),
            currTime);
        prevUpdateCounterR = updateCounter;
    }

    chassisRKeyInputFiltered = algorithms::lowPassFilter(
        chassisRKeyInputFiltered,
        drivers->remote.keyPressed(Remote::Key::Q) - drivers->remote.keyPressed(Remote::Key::E),
        CHASSIS_R_KEY_INPUT_FILTER_ALPHA);

    return limitVal<float>(
        chassisRInput.getInterpolatedValue(currTime) + chassisRKeyInputFiltered,
        -1.0f,
        1.0f);
}

float ControlOperatorInterface::getTurretYawInput()
{
    return -drivers->remote.getChannel(Remote::Channel::RIGHT_HORIZONTAL) +
           static_cast<float>(limitVal<int16_t>(
               -drivers->remote.getMouseX(),
               -USER_MOUSE_YAW_MAX,
               USER_MOUSE_YAW_MAX)) *
               USER_MOUSE_YAW_SCALAR;
}

float ControlOperatorInterface::getTurretPitchInput()
{
    return drivers->remote.getChannel(Remote::Channel::RIGHT_VERTICAL) +
           static_cast<float>(limitVal<int16_t>(
               -drivers->remote.getMouseY(),
               -USER_MOUSE_PITCH_MAX,
               USER_MOUSE_PITCH_MAX)) *
               USER_MOUSE_PITCH_SCALAR;
}

float ControlOperatorInterface::getSentinelSpeedInput()
{
    return drivers->remote.getChannel(aruwlib::Remote::Channel::LEFT_HORIZONTAL) *
           USER_STICK_SENTINEL_DRIVE_SCALAR;
}
}  // namespace control

}  // namespace aruwlib
