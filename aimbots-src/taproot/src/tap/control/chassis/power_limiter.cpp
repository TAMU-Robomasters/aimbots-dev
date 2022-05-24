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
    tap::communication::sensors::current::CurrentSensorInterface *currentSensor,
    float startingEnergyBuffer,
    float energyBufferLimitThreshold,
    float energyBufferCritThreshold)
    : drivers(drivers),
      currentSensor(currentSensor),
      startingEnergyBuffer(startingEnergyBuffer),
      energyBufferLimitThreshold(energyBufferLimitThreshold),
      energyBufferCritThreshold(energyBufferCritThreshold),
      energyBuffer(startingEnergyBuffer),
      consumedPower(0.0f),
      prevTime(0),
      prevRobotDataReceivedTimestamp(0)
{
}

float PowerLimiter::getPowerLimitRatio()
{
    if (!drivers->refSerial.getRefSerialReceivingData())
    {
        return 1.0f;
    }

    updatePowerAndEnergyBuffer();

    if (energyBuffer < energyBufferLimitThreshold)
    {
        // If we have eaten through the majority of our energy buffer, do harsher limiting
        // Cap the penalty at energyBufferCritThreshold / energyBufferLimitThreshold.
        return limitVal(
            static_cast<float>(energyBuffer - energyBufferCritThreshold) /
                energyBufferLimitThreshold,
            0.0f,
            1.0f);
    }
    else
    {
        return 1.0f;
    }
}

void PowerLimiter::updatePowerAndEnergyBuffer()
{
    const auto &robotData = drivers->refSerial.getRobotData();
    const auto &chassisData = robotData.chassis;
    const float current = currentSensor->getCurrentMa();
    const float newChassisPower = chassisData.volt * current / 1'000'000.0f;

    // Manually compute energy buffer using consumedPower read from current sensor.
    // See rules manual for reasoning behind the energy buffer calculation.
    const float dt = tap::arch::clock::getTimeMilliseconds() - prevTime;
    prevTime = tap::arch::clock::getTimeMilliseconds();
    energyBuffer -= (consumedPower - chassisData.powerConsumptionLimit) * dt / 1000.0f;

    if (robotData.robotDataReceivedTimestamp != prevRobotDataReceivedTimestamp)
    {
        energyBuffer = chassisData.powerBuffer;
        prevRobotDataReceivedTimestamp = robotData.robotDataReceivedTimestamp;
    }

    consumedPower = newChassisPower;
}

}  // namespace tap::control::chassis
