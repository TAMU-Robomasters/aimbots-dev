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

#include "imu_heater.hpp"

#include "tap/drivers.hpp"

#include "imu_heater_constants.hpp"

namespace tap::communication::sensors::imu_heater
{
ImuHeater::ImuHeater(Drivers *drivers)
    : drivers(drivers),
      imuTemperatureController(
          TEMPERATURE_PID_P,
          TEMPERATURE_PID_I,
          TEMPERATURE_PID_D,
          TEMPERATURE_PID_MAX_ERR_SUM,
          TEMPERATURE_PID_MAX_OUT)
{
}

void ImuHeater::initialize()
{
    // Initialize the heater timer frequency
    drivers->pwm.setTimerFrequency(bound_ports::IMU_HEATER_TIMER, HEATER_PWM_FREQUENCY);
}

void ImuHeater::runTemperatureController(float temperature)
{
    if (temperature < 0)
    {
        drivers->pwm.write(0.0f, tap::gpio::Pwm::ImuHeater);
        return;
    }

    // Run PID controller to find desired output, output units PWM frequency
    imuTemperatureController.update(IMU_DESIRED_TEMPERATURE - temperature);

    // Set heater PWM output, limit output so it is not < 0
    drivers->pwm.write(
        std::max(0.0f, imuTemperatureController.getValue()),
        tap::gpio::Pwm::ImuHeater);
}
}  // namespace tap::communication::sensors::imu_heater
