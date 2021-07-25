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

#ifndef CONTROL_OPERATOR_INTERFACE_HPP_
#define CONTROL_OPERATOR_INTERFACE_HPP_

#include "tap/algorithms/linear_interpolation.hpp"
#include "tap/util_macros.hpp"

namespace tap
{
class Drivers;
namespace control
{
/**
 * A class for interfacing with the remote IO inside of Commands. While the
 * CommandMapper handles the scheduling of Commands, this class is used
 * inside of Commands to interact with the remote. Filtering and normalization
 * is done in this class.
 */
class ControlOperatorInterface
{
public:
    static constexpr int16_t USER_MOUSE_YAW_MAX = 1000;
    static constexpr int16_t USER_MOUSE_PITCH_MAX = 1000;
    static constexpr float USER_MOUSE_YAW_SCALAR = (1.0f / USER_MOUSE_YAW_MAX);
    static constexpr float USER_MOUSE_PITCH_SCALAR = (1.0f / USER_MOUSE_PITCH_MAX);
    static constexpr float USER_STICK_SENTINEL_DRIVE_SCALAR = 5000.0f;
    static constexpr float CHASSIS_X_KEY_INPUT_FILTER_ALPHA = 0.05f;
    static constexpr float CHASSIS_Y_KEY_INPUT_FILTER_ALPHA = 0.05f;
    static constexpr float CHASSIS_R_KEY_INPUT_FILTER_ALPHA = 0.05f;

    ControlOperatorInterface(Drivers *drivers) : drivers(drivers) {}
    DISALLOW_COPY_AND_ASSIGN(ControlOperatorInterface)
    mockable ~ControlOperatorInterface() = default;

    /**
     * @return the value used for chassis movement forward and backward, between -1 and 1.
     */
    mockable float getChassisXInput();

    /**
     * @return the value used for chassis movement side to side, between -1 and 1.
     */
    mockable float getChassisYInput();

    /**
     * @return the value used for chassis rotation, between -1 and 1.
     */
    mockable float getChassisRInput();

    /**
     * @return the value used for turret yaw rotation, between about -1 and 1
     *      this value can be greater or less than (-1, 1) since the mouse input has no
     *      clear lower and upper bound.
     */
    mockable float getTurretYawInput();

    /**
     * @returns the value used for turret pitch rotation, between about -1 and 1
     *      this value can be greater or less than (-1, 1) since the mouse input has no
     *      clear lower and upper bound.
     */
    mockable float getTurretPitchInput();

    /**
     * @returns the value used for sentiel drive speed, between
     *      [-USER_STICK_SENTINEL_DRIVE_SCALAR, USER_STICK_SENTINEL_DRIVE_SCALAR].
     */
    mockable float getSentinelSpeedInput();

private:
    Drivers *drivers;

    uint32_t prevUpdateCounterX = 0;
    uint32_t prevUpdateCounterY = 0;
    uint32_t prevUpdateCounterR = 0;

    algorithms::LinearInterpolation chassisXInput;
    algorithms::LinearInterpolation chassisYInput;
    algorithms::LinearInterpolation chassisRInput;

    float chassisXKeyInputFiltered = 0;
    float chassisYKeyInputFiltered = 0;
    float chassisRKeyInputFiltered = 0;
};  // class ControlOperatorInterface

}  // namespace control

}  // namespace tap

#endif  // CONTROL_OPERATOR_INTERFACE_HPP_
