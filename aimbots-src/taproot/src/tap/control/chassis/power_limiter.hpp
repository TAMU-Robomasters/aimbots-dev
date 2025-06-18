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

#ifndef TAPROOT_POWER_LIMITER_HPP_
#define TAPROOT_POWER_LIMITER_HPP_

#include "tap/communication/gpio/analog.hpp"
#include "tap/communication/sensors/current/current_sensor_interface.hpp"

namespace tap
{
class Drivers;
}

namespace tap::control::chassis
{
/**
 * A utility to limit the power consumption of a chassis system according to its live current draw
 * and available power buffer (from the referee system). Assumes the motors being used are M3508s
 * and that the referee system is connected via UART.
 *
 * This class currently requires the referee system connected via its default UART port
 * and an object that implements the current sensor interface (specified upon construction).
 * The following issues address improved versatility of this class:
 * - https://gitlab.com/aruw/controls/taproot/-/issues/45
 *
 * This class will report a fraction between [0, 1] that you should then multiply your motors by
 * after running a control loop and before sending to the motors.
 *
 * Here is an example of how to use this class. This example assumes you have `leftWheel` and
 * `rightWheel` `DjiMotor` objects declared as well as a `PowerLimiter` instance called
 * `powerLimiter`. It is assumed you call this function repeatedly at the same rate that you run
 * your chassis controller.
 *
 * ```
 * // ... run some chassis control algorithm ...
 *
 * float powerLimitFrac = powerLimiter.getPowerLimitRatio();
 * leftWheel.setDesiredOutput(powerLimitFrac * leftWheel.getOutputDesired());
 * rightWheel.setDesiredOutput(powerLimitFrac * rightWheel.getOutputDesired());
 * ```
 *
 * If the referee system is not connected (valid data has not recently arrived), this class
 * does nothing.
 */
class PowerLimiter
{
public:
    /**
     * Constructs a power limiter helper object.
     *
     * @param[in] drivers Global drivers object.
     * @param[in] currentSensor `CurrentSensorInterface` that will be used when power limiting.
     *      The current sensor should be connected in parallel with the main chassis power line.
     * @param[in] energyBufferLimitThreshold Energy in Joules. The amount of energy left in the
     *      energy buffer before power limiting is applied.
     * @param[in] energyBufferCritThreshold Energy in Joules. If the amount of energy in the energy
     *      buffer is equal or less than this value, the power limiting fraction will be 0.
     */
    PowerLimiter(
        const tap::Drivers *drivers,
        tap::communication::sensors::current::CurrentSensorInterface *currentSensor,
        float startingEnergyBuffer,
        float energyBufferLimitThreshold,
        float energyBufferCritThreshold);

    /**
     * A function to be called repeatedly (in a subsystem's refresh function, for example). Checks
     * the voltage through the referee system and the current using the current sensor to calculate
     * current power consumption, which is used to update the energy buffer by integrating power.
     * Once the energy buffer is known, it is used in conjunction with the constants passed in
     * through the constructor to compute a fraction that can be used to perform power limiting.
     *
     * @note Must be called immediately *after* setpoints are configured. This function returns a
     * value between [0, 1] that you should then multiply the desired output of your motors by. See
     * class comment for more details.
     *
     * @note Tested with a normal four-wheel mecanum chassis and a two-wheel sentry chassis.
     */
    float getPowerLimitRatio();

private:
    const tap::Drivers *drivers;
    tap::communication::sensors::current::CurrentSensorInterface *currentSensor;
    const float startingEnergyBuffer;
    const float energyBufferLimitThreshold;
    const float energyBufferCritThreshold;

    float energyBuffer;
    float consumedPower;
    uint32_t prevTime;
    uint32_t prevRobotDataReceivedTimestamp;

    /**
     * Computes the chassis power and the energy remaining in the energy buffer.
     */
    void updatePowerAndEnergyBuffer();
};
}  // namespace tap::control::chassis

#endif  // TAPROOT_POWER_LIMITER_HPP_
