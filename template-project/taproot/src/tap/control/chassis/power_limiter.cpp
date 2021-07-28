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

#include "power_limiter.hpp"

#include "tap/algorithms/math_user_utils.hpp"
#include "tap/drivers.hpp"

using namespace tap::algorithms;
using namespace tap::motor;
using std::max;

namespace tap::control::chassis
{
PowerLimiter::PowerLimiter(
    const tap::Drivers *drivers,
    tap::gpio::Analog::Pin currentPin,
    float maxEnergyBuffer,
    float energyBufferLimitThreshold,
    float energyBufferCritThreshold,
    float powerConsumptionThreshold,
    float currentAllocatedForEnergyBufferLimiting,
    const tap::motor::MotorConstants &motorConstants)
    : drivers(drivers),
      currentPin(currentPin),
      maxEnergyBuffer(maxEnergyBuffer),
      energyBufferLimitThreshold(energyBufferLimitThreshold),
      energyBufferCritThreshold(energyBufferCritThreshold),
      powerConsumptionThreshold(powerConsumptionThreshold),
      currentAllocatedForEnergyBufferLimiting(currentAllocatedForEnergyBufferLimiting),
      motorConstants(motorConstants),
      prevChassisCurrent(0),
      energyBuffer(maxEnergyBuffer),
      consumedPower(0),
      prevTime(0)
{
}

void PowerLimiter::performPowerLimiting(tap::motor::DjiMotor *motors[], int numMotors)
{
    if (!drivers->refSerial.getRefSerialReceivingData())
    {
        return;
    }

    updatePowerAndEnergyBuffer();

    const auto &chassis = drivers->refSerial.getRobotData().chassis;

    /**
     * Current in mA. The amount of current allocated for use when not eating into the energy
     * buffer. A fraction of the allocated current will be used when the power consumption limit is
     * being close to exceeded.
     */
    const float CURRENT_ALLOCATED_FOR_POWER_CONSUMPTION_LIMITING =
        1000.0f * chassis.powerConsumptionLimit / chassis.volt;

    /**
     * The fraction from 0-1 determined by how much of the energy buffer has been consumed
     * and that regulates what fraction of currentAllocatedForEnergyBufferLimiting
     * may be requested by the motors at any time. When the power buffer is not eaten into,
     * defaults to 1.
     */
    float energyBufferFrac = 1.0f;
    /**
     * The fraction from 0-1 determined by how much of the available power (used before the
     * energy buffer is used) has been consumed
     * and that regulates what fraction of CURRENT_ALLOCATED_FOR_POWER_CONSUMPTION_LIMITING
     * may be requested by the motors at any time.
     */
    float powerConsumptionFrac = 1.0f;

    if (energyBuffer < energyBufferLimitThreshold)
    {
        // If we have eaten through the majority of our energy buffer, do harsher limiting
        // Cap the penalty at energyBufferCritThreshold / energyBufferLimitThreshold.
        energyBufferFrac = static_cast<float>(max(energyBuffer, energyBufferCritThreshold)) /
                           energyBufferLimitThreshold;
        powerConsumptionFrac = 0.0f;
    }
    else
    {
        /**
         * Note: chassis.powerConsumptionLimit is the power limit we are allowed to consume.
         * This varies based on robot level. chassis.power is the actual power that the chassis
         * motors are consuming.
         *
         * Also note that energyBufferFrac is always 1 when entering this conditional
         */
        if (consumedPower + powerConsumptionThreshold > chassis.powerConsumptionLimit)
        {
            // If we are above a the power consumption limit but we haven't eaten into our
            // energy buffer enough to enter the if statement of the outermost conditional, do
            // less strict limiting
            if (consumedPower < chassis.powerConsumptionLimit)
            {
                // Only do minimal limiting because we are not close to going over chassis power
                powerConsumptionFrac = (chassis.powerConsumptionLimit - consumedPower) /
                                       (chassis.powerConsumptionLimit - powerConsumptionThreshold);
            }
            else
            {
                // Limit strictly
                powerConsumptionFrac = 0.0f;
            }
        }
    }

    /**
     * Current limit is a combination of the fraction of current avaiable based on limiting
     * from the power consumption and energy buffer
     */
    float chassisCurrentLimit =
        powerConsumptionFrac * CURRENT_ALLOCATED_FOR_POWER_CONSUMPTION_LIMITING +
        energyBufferFrac * currentAllocatedForEnergyBufferLimiting;

    float chassisCurrentOutput = 0.0f;
    for (int i = 0; i < numMotors; i++)
    {
        chassisCurrentOutput += fabsf(motors[i]->getOutputDesired());
    }
    chassisCurrentOutput = motorConstants.convertOutputToCurrent(chassisCurrentOutput);

    if (!compareFloatClose(0, chassisCurrentOutput, 1E-5) &&
        chassisCurrentOutput > chassisCurrentLimit)
    {
        // Limit all motors by some fraction of the total current limit / the current limit
        float currentLimitFrac = chassisCurrentLimit / chassisCurrentOutput;
        for (int i = 0; i < numMotors; i++)
        {
            motors[i]->setDesiredOutput(motors[i]->getOutputDesired() * currentLimitFrac);
        }
    }

#ifdef POWER_LIMIT_VARIANT
    /**
     * This makes more sense than the top power limiting stuff
     * but doesn't work well yet.
     */
    // The amount of power available to use in the energy buffer given you want to use it in
    // `timeToEatEnergyBuffer` seconds.
    float powerProvidedByEnergyBuffer =
        max(0.0f, (energyBuffer - energyBufferThreshold) / timeToEatEnergyBuffer);
    // The power limit that you cannot go over.
    float powerLimit = chassis.powerConsumptionLimit + powerProvidedByEnergyBuffer;

    // The amount of power being consumed by the motors
    float chassisPowerDesired = fabsf(consumedPower);

    // Only apply limiting if desired power exceeds the power limit
    if (chassisPowerDesired > powerLimit)
    {
        for (int i = 0; i < numMotors; i++)
        {
            motors[i]->setDesiredOutput(
                motors[i]->getOutputDesired() * powerLimit / chassisPowerDesired);
        }
    }
#endif
}

void PowerLimiter::updatePowerAndEnergyBuffer()
{
    const auto &chassisData = drivers->refSerial.getRobotData().chassis;
    float current = getChassisCurrent();
    float newChassisPower = drivers->refSerial.getRobotData().chassis.volt * current / 1'000'000.0f;

    // See rules manual for reasoning behind the energy buffer calculation
    float dt = tap::arch::clock::getTimeMilliseconds() - prevTime;
    prevTime = tap::arch::clock::getTimeMilliseconds();
    energyBuffer -= (consumedPower - chassisData.powerConsumptionLimit) * dt / 1000.0f;

    // To avoid large deviation from true power buffer, do this.
    if (!tap::algorithms::compareFloatClose(energyBuffer, chassisData.powerBuffer, 5))
    {
        energyBuffer = chassisData.powerBuffer;
    }

    if (energyBuffer > maxEnergyBuffer)
    {
        energyBuffer = maxEnergyBuffer;
    }

    consumedPower = newChassisPower;
}

float PowerLimiter::getChassisCurrent()
{
    prevChassisCurrent = lowPassFilter(
        prevChassisCurrent,
        (drivers->analog.read(currentPin) - CURRENT_SENSOR_ZERO_MA) * CURRENT_SENSOR_MV_PER_MA,
        CURRENT_SENSOR_LOW_PASS_ALPHA);

    return prevChassisCurrent;
}
}  // namespace tap::control::chassis
