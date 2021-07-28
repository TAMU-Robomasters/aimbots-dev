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

#ifndef POWER_LIMITOR_HPP_
#define POWER_LIMITOR_HPP_

#include "tap/communication/gpio/analog.hpp"
#include "tap/motor/dji_motor.hpp"
#include "tap/motor/motor_constants.hpp"

namespace tap
{
class Drivers;
}

namespace tap::control::chassis
{
/**
 * A utility object that a chassis may use to provide power limiting. Assumes the motor
 * being used is an M3508 and the current sensor is a TODO.
 */
class PowerLimiter
{
public:
    /**
     * Calibrated current sensor's mv to ma ratio used to convert analog value to an actual
     * current. The current sensor we are using is linear, so only the slope and y-intercept
     * are needed.
     */
    static constexpr float CURRENT_SENSOR_MV_PER_MA = 11.47f;
    /**
     * Voltage (in mV) that the current sensor reads 0 mA.
     */
    static constexpr float CURRENT_SENSOR_ZERO_MA = 3088.7f;

    /**
     * A low pass filter is used to remove spikes, this is the alpha parameter that defines the low
     * pass filter.
     */
    static constexpr float CURRENT_SENSOR_LOW_PASS_ALPHA = 0.1f;

    /**
     * Constructs a power limiter helper object.
     *
     * @param[in] drivers Global drivers object.
     * @param[in] currentPin The pin connected to a current sensor connected in series with the
     *      chassis.
     * @param[in] energyBufferLimitThreshold Energy in Joules. The amount of energy left in the
     *      energy buffer before harsh power limiting is applied.
     * @param[in] energyBufferCritThreshold Energy in Joules. The amount of energy left in the
     *      energy buffer that limiting is capped at. When harsh limiting is applied it is a
     *      fraction of the amount of energy left in the buffer divided by the limiting threshold,
     *      so the fraction will never be smaller than ENERGY_BUFFER_CRIT_THRESHOLD /
     *      ENERGY_BUFFER_LIMIT_THRESHOLD.
     * @param[in] powerConsumptionThreshold Power in Watts. When the power consumed is within this
     *      threshold of the power consumption limit, power limiting is applied.
     * @param[in] currentAllocatedForEnergyBufferLimiting Current in mA. The amount of
     *      current allocated for use by the robot before eating into the
     *      energy buffer. A fraction of the allocated current will be used when the energy buffer
     *      is closed to being fully consumed.
     * @param[in] motorConstants An object that contains motor constants specific to the
     *      motor being used in power limiting.
     */
    PowerLimiter(
        const tap::Drivers *drivers,
        tap::gpio::Analog::Pin currentPin,
        float maxEnergyBuffer,
        float energyBufferLimitThreshold,
        float energyBufferCritThreshold,
        float powerConsumptionThreshold,
        float currentAllocatedForEnergyBufferLimiting,
        const tap::motor::MotorConstants &motorConstants);

    /**
     * A function to be called repeatedly (in a subsystem's refresh function, for example), that
     * performs power limiting on the motors passed in.
     *
     * @note Four wheeled varient designed for a normal mecanum wheeled chassis.
     */
    void performPowerLimiting(tap::motor::DjiMotor *motors[], int numMotors);

private:
    const tap::Drivers *drivers;
    const tap::gpio::Analog::Pin currentPin;
    const float maxEnergyBuffer;
    const float energyBufferLimitThreshold;
    const float energyBufferCritThreshold;
    const float powerConsumptionThreshold;
    const float currentAllocatedForEnergyBufferLimiting;
    const tap::motor::MotorConstants &motorConstants;
    float prevChassisCurrent;
    float energyBuffer;
    float consumedPower;
    uint32_t prevTime;

    /**
     * @return The chassis current, in mA.
     */
    float getChassisCurrent();

    /**
     * Computes the chassis power and the energy remaining in the energy buffer.
     */
    void updatePowerAndEnergyBuffer();
};
}  // namespace tap::control::chassis

#endif  // POWER_LIMITOR_HPP_
