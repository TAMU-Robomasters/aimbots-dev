/*
 * Copyright (c) 2020-2022 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#ifndef TAPROOT_IMU_HEATER_HPP_
#define TAPROOT_IMU_HEATER_HPP_

#include "tap/util_macros.hpp"

#include "modm/math/filter/pid.hpp"

namespace tap
{
class Drivers;
}

namespace tap::communication::sensors::imu_heater
{
class ImuHeater
{
public:
    /**
     * Normal operating temperature is ~40 degrees C, and RM manual says the optimal operating
     * temperature is ~15-20 degrees C above the normal operating temperature of the board.
     */
    static constexpr float IMU_DESIRED_TEMPERATURE = 50.0f;

    ImuHeater(Drivers *drivers);
    DISALLOW_COPY_AND_ASSIGN(ImuHeater)
    ~ImuHeater() = default;

    /**
     * Configures the imu heater timer's frequency.
     */
    void initialize();

    /**
     * Runs a PID controller to regulate the temperature of the IMU.
     *
     * @param[in] temperature The temperature of the mpu6500, units degrees C.
     */
    void runTemperatureController(float temperature);

private:
    /**
     * PID constants for temperature control.
     */
    static constexpr float TEMPERATURE_PID_P = 1.0f;
    static constexpr float TEMPERATURE_PID_I = 0.0f;
    static constexpr float TEMPERATURE_PID_D = 20.0f;
    static constexpr float TEMPERATURE_PID_MAX_ERR_SUM = 0.0f;
    static constexpr float TEMPERATURE_PID_MAX_OUT = 1.0f;

    /**
     * PWM frequency of the timer associated with the GPIO pin that is in charge
     * of controlling the temperature of the IMU.
     */
    static constexpr float HEATER_PWM_FREQUENCY = 1000.0f;

    Drivers *drivers;

    modm::Pid<float> imuTemperatureController;
};
}  // namespace tap::communication::sensors::imu_heater

#endif  // TAPROOT_MPU6500_HEATER_HPP_
